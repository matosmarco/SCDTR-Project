#include "network.h"
#include "command.h"
#include "config.h"
#include "mcp2515.h"
#include "calibrationMatrix.h"
#include "calibrationFSM.h"
#include "metrics.h"
#include "box.h"
#include "led.h"
#include "ldr.h"
#include "luminaire.h"
#include "pid.h" 
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

// Variables declared in fase2.ino
extern String inputBuffer;
extern LED led;
extern LDR ldr;
extern Box box;
extern Metrics metrics;
extern Luminaire luminaire;
extern pid pid; 

// System variables and RTOS variables
extern uint8_t node_address;
extern float global_energy_cost;
extern QueueHandle_t canTxQueue;
extern QueueHandle_t canRxQueue;

// Calibration
extern CalibrationMatrix myMatrix;
extern CalibrationFSM* fsm;
extern TaskHandle_t calibTaskHandle;

// MCP2515 CAN Bus
extern MCP2515 can0;
extern struct can_frame canMsgTx, canMsgRx;
extern MCP2515::ERROR err;

// Network and streaming variables
extern bool net_streaming;
extern char net_stream_var;
extern uint8_t net_stream_dest;
extern bool net_sending_buffer;
extern char net_buffer_var;
extern int net_buffer_index;
extern uint8_t net_buffer_dest;

    
float global_K_matrix[MAX_NODES][MAX_NODES] = {0.0f}; // Matrix to store all gains (1 to MAX_NODES)
int expected_k_elements = 0; // variable to flag the number of elements to be sent

// Functions
void checkSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
          processCommand(inputBuffer, led, ldr, box, metrics, luminaire, pid);
      }
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
}
void processNetworkMessages() {
  // Verify message in the Queue
  int msgCount = uxQueueMessagesWaiting(canRxQueue);
  if (msgCount == 0) return;

  // Local buffer
  ProtocolMsg_t msgBuffer[QUEUE_SIZE];
  int actualCount = 0;

  while (xQueueReceive(canRxQueue, &msgBuffer[actualCount], 0) == pdPASS) {
    actualCount++;
    if (actualCount >= 10) break; // to avoid overflow
  }

  //Insertion sort - lowest ID
  for (int i = 1; i < actualCount; i++) {
      ProtocolMsg_t key = msgBuffer[i];
      int j = i - 1;
      while (j >= 0 && msgBuffer[j].src_id > key.src_id) {
          msgBuffer[j + 1] = msgBuffer[j];
          j = j - 1;
      }
      msgBuffer[j + 1] = key;
  }

  // Processing messages by priority
  for (int i = 0; i < actualCount; i++) {
    ProtocolMsg_t msgIn = msgBuffer[i];

    // Wake-up message (entering in the network)
    if (msgIn.cmd == 'w' || msgIn.cmd == 'W') {
        myMatrix.addNode(msgIn.src_id);
        
        if (msgIn.cmd == 'w') {
            if (calibTaskHandle != NULL) {
                xTaskNotifyGive(calibTaskHandle); 
                xTaskNotify(calibTaskHandle, EVENT_NETWORK_CHANGE, eSetBits);
            }
            ProtocolMsg_t replyMsg = {node_address, msgIn.src_id, 'W', ' ', 0.0f};
            xQueueSend(canTxQueue, &replyMsg, 0);
        }
    }

    // Passive calibration and token ring ('c')
    else if (msgIn.cmd == 'c') {
        if (msgIn.subcmd == '1' && msgIn.src_id != node_address) {
            float y_total = ldr.readLux();
            float d_k = box.fixed_background(ldr); 
            float k_ij = y_total - d_k;
            if (k_ij < 0) k_ij = 0; 
            
            myMatrix.updateGain(msgIn.src_id, k_ij);
            Serial.printf("Measured cross-coupling gain from node %d: %.4f\n", msgIn.src_id, k_ij);
        } 
        else if (msgIn.subcmd == '0' && msgIn.src_id != node_address) {
            if (calibTaskHandle != NULL) {
                xTaskNotify(calibTaskHandle, EVENT_TOKEN_DONE, eSetBits);
            }
        }
    }

    // Distributed control- exchange values of k_ij
    else if (msgIn.cmd == 'K') {
       	// Individual element of the matrix. subcmd is the ID of the column
        uint8_t col_id = (uint8_t)msgIn.subcmd;
        float gain_value = msgIn.value;
            
        // "ID:gain" print
        Serial.printf("%d:%.4f ", col_id, gain_value);
            
        // 2. GUARDA NA MATRIZ GLOBAL PARA O ADMM
        if (msgIn.src_id > 0 && msgIn.src_id <= MAX_NODES && col_id > 0 && col_id <= MAX_NODES) { 
        	global_K_matrix[msgIn.src_id - 1][col_id - 1] = gain_value;
        }        

     }


    // ---------------------------------------------------------
    // D. COMANDOS GERAIS DA REDE E GUIÃO
    // ---------------------------------------------------------
    else if (msgIn.cmd == 'R') {    
      metrics.reset();
      box.calibrate_background(led, ldr);
      box.identify_static_gain(led, ldr);
      Serial.printf("Node %d restarted via Broadcast!\n", node_address);
    }
    else if (msgIn.cmd == 'o') {
      char occ_state = (char)msgIn.value;
      if (occ_state == 'o') { luminaire.state = LuminaireState::OFF; luminaire.reference = 0; } 
      else if (occ_state == 'l') { luminaire.state = LuminaireState::LOW; luminaire.reference = 15; } 
      else if (occ_state == 'h') { luminaire.state = LuminaireState::HIGH; luminaire.reference = 30; }
    }
    else if (msgIn.cmd == 'U') luminaire.low_bound = msgIn.value;
    else if (msgIn.cmd == 'O') luminaire.high_bound = msgIn.value;  
    else if (msgIn.cmd == 'C') global_energy_cost = msgIn.value;

    // ---------------------------------------------------------
    // E. GET REQUESTS ('g') & ANSWERS ('A')
    // ---------------------------------------------------------
    else if (msgIn.cmd == 'g') {
      
      // -- RESPOSTA AO PEDIDO DE VETOR K ('k') --
      if (msgIn.subcmd == 'k') {
          int n = myMatrix.getNumNodes(); 
          
          // 1. Envia marcador de INÍCIO ('S') COM O TAMANHO DO VETOR (n)
          //ProtocolMsg_t startK = {node_address, msgIn.src_id, 'K', 'S', (float)n};
          //xQueueSend(canTxQueue, &startK, 0);
          
          // 2. Envia os elementos do vetor
          const uint8_t* nodes = myMatrix.getNodesArray();
          for (int i = 0; i < n; i++) {
              ProtocolMsg_t replyK = {node_address, msgIn.src_id, 'K', (char)nodes[i], myMatrix.getGain(nodes[i])};
              xQueueSend(canTxQueue, &replyK, 0);
          }
          
          // 3. Envia marcador de FIM ('E')
          //ProtocolMsg_t endK = {node_address, msgIn.src_id, 'K', 'E', 0.0f};
          //xQueueSend(canTxQueue, &endK, 0);
      } 
      // -- RESTANTES COMANDOS GERAIS ('y', 'u', 'r', etc) --
      else {
          ProtocolMsg_t response;
          response.dest_id = msgIn.src_id; 
          response.src_id = node_address;
          response.cmd = 'A';              
          response.subcmd = msgIn.subcmd;

          if (msgIn.subcmd == 'y') response.value = ldr.readLux();
          else if (msgIn.subcmd == 'u') response.value = luminaire.current_u;
          else if (msgIn.subcmd == 'r') response.value = luminaire.reference;
          else if (msgIn.subcmd == 'v') response.value = ldr.readVoltage();
          else if (msgIn.subcmd == 'd') response.value = box.get_background(ldr);
          else if (msgIn.subcmd == 'p') response.value = metrics.getInstantaneousPower(luminaire.current_u);
          else if (msgIn.subcmd == 't') response.value = millis() / 1000.0f;
          else if (msgIn.subcmd == 'E') response.value = metrics.energy;
          else if (msgIn.subcmd == 'V') response.value = metrics.getAverageVisibility();
          else if (msgIn.subcmd == 'F') response.value = metrics.flicker; 
          else if (msgIn.subcmd == 'U') response.value = luminaire.low_bound;
          else if (msgIn.subcmd == 'O') response.value = luminaire.high_bound;
          else if (msgIn.subcmd == 'C') response.value = global_energy_cost;
          else if (msgIn.subcmd == 'L') {
              if (luminaire.state == LuminaireState::HIGH) response.value = luminaire.high_bound;
              else response.value = luminaire.low_bound; 
          }
          else if (msgIn.subcmd == 'o') {
              char s = 'o';
              if (luminaire.state == LuminaireState::LOW) s = 'l';
              else if (luminaire.state == LuminaireState::HIGH) s = 'h';
              response.value = (float)s;
          }

          xQueueSend(canTxQueue, &response, 0);
      }
    }
    
    // ---------------------------------------------------------
    // PROCESSAMENTO DE RESPOSTAS NORMAIS ('A')
    // ---------------------------------------------------------
    else if (msgIn.cmd == 'A') {
      Serial.println("\n==================================");
      Serial.print(">>> RECEIVED FINAL ANSWER: ");
      if (msgIn.subcmd == 'o') {
          Serial.printf("%c %d %c", msgIn.subcmd, msgIn.src_id, (char)msgIn.value);
      } else {
          Serial.printf("%c %d %.4f", msgIn.subcmd, msgIn.src_id, msgIn.value);
      }
      Serial.println("\n==================================\n");
    }

    // ---------------------------------------------------------
    // F. STREAMING E METRICS BUFFER ('s', 'S', 'D', 'b', 'B')
    // ---------------------------------------------------------
    else if (msgIn.cmd == 's') { 
        net_streaming = true;
        net_stream_var = msgIn.subcmd;
        net_stream_dest = msgIn.src_id;
    }
    else if (msgIn.cmd == 'S') { 
        net_streaming = false;
    }
    else if (msgIn.cmd == 'D') { 
        Serial.printf("s %c %d %.4f %lu\n", msgIn.subcmd, msgIn.src_id, msgIn.value, millis());
    }
    
    else if (msgIn.cmd == 'b') { 
        if (msgIn.value == -1.0f) {        
            Serial.printf("b %c %d ", msgIn.subcmd, msgIn.src_id);
        } else if (msgIn.value == -2.0f) { 
            Serial.println();
        } else {                           
            net_sending_buffer = true;
            net_buffer_var = msgIn.subcmd;
            net_buffer_index = 0;
            net_buffer_dest = msgIn.src_id;
            
            ProtocolMsg_t startMsg = {node_address, msgIn.src_id, 'b', msgIn.subcmd, -1.0f};
            xQueueSend(canTxQueue, &startMsg, 0);
        }
    }
    else if (msgIn.cmd == 'B') { 
        Serial.printf("%.2f,", msgIn.value);
    }
  } 
}
// CAN communication functions - core 1
void sendMessage() {
  ProtocolMsg_t msgOut;
  if (xQueueReceive(canTxQueue, &msgOut, 0) == pdPASS) {
    canMsgTx.can_id = node_address;
    canMsgTx.can_dlc = 8; 
    
    canMsgTx.data[1] = msgOut.dest_id;
    canMsgTx.data[0] = msgOut.src_id;
    canMsgTx.data[2] = msgOut.cmd;
    canMsgTx.data[3] = msgOut.subcmd;
    
    memcpy(&canMsgTx.data[4], &msgOut.value, sizeof(float));
    err = can0.sendMessage(&canMsgTx);

    if (err == MCP2515::ERROR_OK) {
        if (msgOut.dest_id == 255) {
            Serial.printf("[CAN TX] Successfully BROADCASTED '%c %c' to the network\n", msgOut.cmd, msgOut.subcmd);
        } else {
            Serial.printf("[CAN TX] Successfully sent '%c %c' to node %d\n", msgOut.cmd, msgOut.subcmd, msgOut.dest_id);
        }
    } else {
        Serial.printf("[CAN ERROR] Failed to send! MCP2515 Error Code: %d\n", err);
    }
  }
}

void readMessage() {
  while((err = can0.readMessage(&canMsgRx)) == MCP2515::ERROR_OK){
    uint8_t target_dest = canMsgRx.data[1];
    uint8_t sender = canMsgRx.data[0];
    
    //Serial.printf("[CAN RX] Physically received a message from node %d to node %d\n", sender, target_dest);
    if (target_dest == node_address || target_dest == 255) {
      ProtocolMsg_t msgIn;
      msgIn.dest_id = canMsgRx.data[1]; 
      msgIn.src_id  = canMsgRx.data[0];
      msgIn.cmd     = canMsgRx.data[2];
      msgIn.subcmd  = canMsgRx.data[3];
      memcpy(&msgIn.value, &canMsgRx.data[4], sizeof(float));
      xQueueSend(canRxQueue, &msgIn, 0);
    }
  }
}

// CAN error handling - core 1
void handleCANErrors() {
    uint8_t eflg = can0.getErrorFlags();
    static uint8_t last_eflg = 0; 

    if (eflg & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) {
        can0.clearRXnOVRFlags(); 
        can0.clearInterrupts();  
        if (!(last_eflg & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR))) {
            Serial.println("\n[RECUPERAÇÃO] Flags de Overflow limpas. Rede restaurada!");
        }
    }

    if (eflg != last_eflg) {
        last_eflg = eflg; 
        if (eflg != 0) {
            Serial.println("\n[CAN DIAGNOSTICS] Alteração no estado físico do Bus:");
            if (eflg & MCP2515::EFLG_TXBO)   Serial.println(" -> ERRO CRÍTICO: Bus-Off (Muitas falhas de TX)!"); 
            if (eflg & MCP2515::EFLG_TXEP)   Serial.println(" -> AVISO: TX Error-Passive (TEC >= 128)");
            if (eflg & MCP2515::EFLG_RXEP)   Serial.println(" -> AVISO: RX Error-Passive (REC >= 128)");
            if (eflg & MCP2515::EFLG_TXWAR)  Serial.println(" -> AVISO: TX Error Warning (TEC >= 96)");
            if (eflg & MCP2515::EFLG_RXWAR)  Serial.println(" -> AVISO: RX Error Warning (REC >= 96)");
            Serial.println("------------------------------------------------");
            can0.clearERRIF();
            can0.clearMERR();
        } else {
            Serial.println("\n[CAN DIAGNOSTICS] A rede estabilizou. Erros físicos resolvidos.");
        }
    }

    if (eflg & MCP2515::EFLG_TXBO) {
        Serial.println("\n[RECUPERAÇÃO] A reiniciar o controlador MCP2515 para sair de Bus-Off...");
        can0.reset();
        can0.setBitrate(CAN_1000KBPS);
        can0.setNormalMode();
        last_eflg = 0; 
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}
