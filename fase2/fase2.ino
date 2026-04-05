#include "led.h"
#include "ldr.h"
#include "box.h"
#include "command.h"
#include "metrics.h"
#include "pid.h"
#include "config.h"
#include "mcp2515.h"
#include "pico/unique_id.h"

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

// FreeRTOS thread-safe queues for inter-core communication
QueueHandle_t canTxQueue;
QueueHandle_t canRxQueue; 

// Network control variables 
bool net_streaming = false;
char net_stream_var = 'y';
uint8_t net_stream_dest = 0;

bool net_sending_buffer = false;
char net_buffer_var = 'y';
int net_buffer_index = 0;
uint8_t net_buffer_dest = 0;

// Network topology and calibration variables
const int MAX_NODES = 10;
uint8_t network_nodes[MAX_NODES];  
int num_nodes = 0;

// Synchronization and Calibration Variables
bool calibration_done = false;
bool needs_calibration = true; // Starts at true state to force initial calibration at startup
float system_gains[MAX_NODES]; // Array to store the cross-coupling (K_ij) and self-coupling (K_ii) gains
int current_token_index = 0;

// Add node to the illumination system
void addNode(uint8_t id) {
    for(int i = 0; i < num_nodes; i++) {
        if(network_nodes[i] == id) return;
    }
    if(num_nodes < MAX_NODES) {
        network_nodes[num_nodes++] = id;
        Serial.printf("Topology: Node %d discovered! Total: %d\n", id, num_nodes);
    }
}

// Sort the nodes (Insertion Sort) to ensure all nodes follow the same order
void sortNetworkNodes() {
    for (int i = 1; i < num_nodes; i++) {
        uint8_t key = network_nodes[i];
        int j = i - 1;

        // Move elements of network_nodes[0..i-1] that are
        // greater than key to one position ahead of their current position
        while (j >= 0 && network_nodes[j] > key) {
            network_nodes[j + 1] = network_nodes[j];
            j = j - 1;
        }
        network_nodes[j + 1] = key;
    }
}

// Global objects 
LED led;
LDR ldr;
Box box;
Metrics metrics(0.01963); 
Luminaire luminaire;
pid pid(0.01);         
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};
float global_energy_cost = 1.0f;

pico_unique_board_id_t pico_board_id; 
uint8_t node_address; 
struct can_frame canMsgTx, canMsgRx;
MCP2515::ERROR err;

unsigned long last_time = 0;
const unsigned long sample_time = 10;
String inputBuffer = "";              
unsigned long last_micros = 0; 
long max_jitter = 0;

// FreeRTOS task: Distributed controller (core 0)
void controllerTask(void *pvParameters) {
    /*
    // 1. Wait for the automatic calibration to finish before starting the controller
    while (!calibration_done) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    Serial.println(">>> Calibration finished. Starting Distributed Controller (100Hz)...");

    // 2. Configure the exact timer for 100Hz (10ms)
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 

    // 3. Infinite Control Loop
    for (;;) {
        // FreeRTOS ensures this loop wakes up EXACTLY every 10ms
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // =========================================================
        // PLACEHOLDER: DISTRIBUTED CONTROLLER LOGIC
        // =========================================================
        
        // TODO: Insert Consensus math and Control algorithm here.

    }
    */
}

// FreeRTOS Task: Calibration orchestration (core 0)

//TODO: estou a meio da função
void calibration(void *pvParameters){
  for(;;){
    // If the network detected new nodes or the node just started
    if(needs_calibration){
      Serial.println("[CALIBRATION TASK] Waiting for Topology stabilization (5s)");

      // Pause for consolidation, if any more nodes are connecting in the same time interval
      // at the same time leaves the time to exchange all the wake-up messages
      vTaskDelay(pdMS_TO_TICKS(5000));

      // Preparing the new round of calibration
      sortNetworkNodes();
      current_token_index = 0;

      // Guarantees that the controller task is suspended, 
      // since we don't have the parameters necessary to control the system
      calibration_done = false;

      Serial.printf("[CALIBRATION TASK] Starting! Total of nodes in the network: %d\n", num_nodes);
      
      while(!calibration_done){
        if(num_nodes > 0 && network_nodes[current_token_index] == node_address){
          Serial.println("Own gain calibration (K_ii gain)");
          // Set own led off
          led.setDuty(0.0f);
          vTaskDelay(pdMS_TO_TICKS(2000)); // 2 seconds to stabilize ldr readings
          float d_k = ldr.readLux();
          // Turn on the led for a 100% duty cycle
          led.setDuty(1.0f);
          vTaskDelay(pdMS_TO_TICKS(2000)); // 2 seconds to stabilize ldr readings
          
          float y_total = ldr.readLux();
          float k_ii = y_total - d_k;

          // Protection to bad readings
          if(k_ii < 0){
            k_ii = 0;
            Serial.println("[ERROR] Bad readings during Kii computation");  
          }
          system_gains[current_token_index] = k_ii;
          Serial.printf("Measured own gain (K_ii): %.4f\n", k_ii);
          
          //

        }
      
      }
    }
  
  }
}








/*
void calibrationTask(void *pvParameters) {
    // Wake-up wait: wait 5 seconds for the network 
    vTaskDelay(pdMS_TO_TICKS(5000)); // vTaskDelay frees the CPU for other tasks during this time

    // Sorting: To know whose turn it is
    sortNetworkNodes();
    Serial.println("Starting Automatic Network Calibration...");

    while (!calibration_done) {
        // Check if it's our turn (Token Passing)
        if (num_nodes > 0 && network_nodes[current_token_index] == node_address) {
            Serial.println("My turn! Turning on the light for calibration.");
            led.setDuty(1.0f); // Turn on the light to 100%
            
            // Notify the network that our light is on ('c' '1')
            ProtocolMsg_t msgMeasure = {node_address, 255, 'c', '1', 0.0f};
            xQueueSend(canTxQueue, &msgMeasure, 0);

            // Wait 2 seconds for the physical LDR to stabilize.
            // Core 0 does NOT block here, it continues to read the CAN Bus
            vTaskDelay(pdMS_TO_TICKS(2000)); 

            // Measure our own self-coupling gain (K_ii)
            float y_total = ldr.readLux();
            float d_k = box.fixed_background(ldr);
            float k_ii = y_total - d_k;
            if (k_ii < 0) k_ii = 0;
            system_gains[current_token_index] = k_ii;
            Serial.printf("My measured self-coupling gain (K_ii): %.4f\n", k_ii);

            // Turn off the light and notify the network that we finished the turn ('c' '0')
            led.setDuty(0.0f);
            ProtocolMsg_t msgEnd = {node_address, 255, 'c', '0', 0.0f};
            xQueueSend(canTxQueue, &msgEnd, 0);

            // Pass the token to the next node
            current_token_index++;
        }
        
        // If all nodes have calibrated, end the routine
        if (num_nodes > 0 && current_token_index >= num_nodes) {
            calibration_done = true;
        }
        
        // Avoid occupying the CPU at 100% while waiting for our turn
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }

    Serial.println(">>> Global Calibration Finished! Starting PID Controller...");
    vTaskDelete(NULL); // Delete this FreeRTOS task to free up RAM
}
*/



// Core 0: Setup and Loop 
void setup() {
  // Create Queues BEFORE any blocking call (Serial or Task)
  canTxQueue = xQueueCreate(QUEUE_SIZE, sizeof(ProtocolMsg_t));
  canRxQueue = xQueueCreate(QUEUE_SIZE, sizeof(ProtocolMsg_t));
  
  // Create the RTOS Task for Calibration
  //xTaskCreate(calibrationTask, "CalibTask", 2048, NULL, 1, NULL);
  xTaskCreate(calibration, "FullCalib", 4096, NULL, 0, NULL); //TODO
  // Create the RTOS Task for Controller (Priority 2)
  xTaskCreate(controllerTask, "CtrlTask", 4096, NULL, 2, NULL);

  Serial.begin(115200);
  while(!Serial) delay(10);
  
  analogReadResolution(12); 
  led.initPWM();
  Serial.println("System Initializing...");
  box.calibrate_background(led, ldr);

  // Wait for Core 1 to create the node_address
  while(node_address == 0) {
      vTaskDelay(1);
  }
  
  addNode(node_address);
  
  // Wake-Up Broadcast
  ProtocolMsg_t wakeupMsg = {node_address, 255, 'w', ' ', 0.0f};
  xQueueSend(canTxQueue, &wakeupMsg, 0);
  Serial.printf("Node %d Wake-Up Broadcasted!\n", node_address);
  
  last_time = millis();
}

void loop() {
  checkSerial();
  processNetworkMessages();

  // The PID Controller ONLY starts after the calibration task indicates it's done
  if (calibration_done) {
      unsigned long current_time = millis();
      if (current_time - last_time >= sample_time) {
        last_time += sample_time; 
        runControlLoop(); 
      }
  }

  vTaskDelay(1); // Keep FreeRTOS happy and responsive
}

// Core 1: Setup and loop (CAN Bus Network)
void setup1() {
  pico_get_unique_board_id(&pico_board_id);
  node_address = 0;
  for (int i = 0; i < 8; i++) {
      node_address ^= pico_board_id.id[i];
  }
  if (node_address == 0) node_address = 1;
  if (node_address == 255) node_address = 254;
  Serial.print(node_address);
  luminaire.setId(node_address); 
  
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode(); 
}

void loop1() {
  sendMessage();
  readMessage();
  handleCANErrors();
  vTaskDelay(1);
}

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
  // Check how many messages are in the queue
  int msgCount = uxQueueMessagesWaiting(canRxQueue);
  if (msgCount == 0) return; // Exit immediately if there are no messages

  // Extract all messages from the queue to a local buffer (max 10)
  ProtocolMsg_t msgBuffer[QUEUE_SIZE];
  int actualCount = 0;

  while (xQueueReceive(canRxQueue, &msgBuffer[actualCount], 0) == pdPASS) {
    actualCount++;
    if (actualCount >= 10) break; // Extra safety bound
  }

  // Priority sorting (Insertion sort - lowest ID first)
  // Perfect and highly efficient algorithm for very small arrays (N <= 10)
  for (int i = 1; i < actualCount; i++) {
      ProtocolMsg_t key = msgBuffer[i];
      int j = i - 1;

      // Slide messages with a higher ID one position forward
      // to make room for the current message (key)
      while (j >= 0 && msgBuffer[j].src_id > key.src_id) {
          msgBuffer[j + 1] = msgBuffer[j];
          j = j - 1;
      }
      // Insert the message in its correct priority position
      msgBuffer[j + 1] = key;
  }

  for (int i = 0; i < actualCount; i++) {
    ProtocolMsg_t msgIn = msgBuffer[i];

    // Synchronous calibration commands (Passive Listening)
    if (msgIn.cmd == 'c') {
        if (msgIn.subcmd == '1' && msgIn.src_id != node_address) {
            // Another node turned on its light to 100%.
            // Measure its influence on us (K_ij)
            float y_total = ldr.readLux();
            float d_k = box.fixed_background(ldr);
            float k_ij = y_total - d_k;
            if (k_ij < 0) k_ij = 0; // Protection against negative noise
            
            // Store the gain at the emitting node's position
            for(int n = 0; n < num_nodes; n++) {
                if(network_nodes[n] == msgIn.src_id) {
                    system_gains[n] = k_ij;
                    break;
                }
            }
            Serial.printf("Measured cross-coupling gain from node %d: %.4f\n", msgIn.src_id, k_ij);
        } 
        else if (msgIn.subcmd == '0' && msgIn.src_id != node_address) {
            // The other node notified that it finished its turn.
            // Advance the token.
            current_token_index++;
            if (current_token_index >= num_nodes) calibration_done = true;
        }
    }
  }

  // 4. PROCESS MESSAGES IN THE NEW PRIORITY ORDER
  for (int i = 0; i < actualCount; i++) {
    ProtocolMsg_t msgIn = msgBuffer[i];

    // Restart command (Broadcast 'R')
    if (msgIn.cmd == 'R') {
      metrics.reset();
      box.calibrate_background(led, ldr);
      box.identify_static_gain(led, ldr);
      Serial.printf("Node %d restarted via Broadcast!\n", node_address);
    }
    
    // Set occupancy command ('o')
    else if (msgIn.cmd == 'o') {
      char occ_state = (char)msgIn.value;
      if (occ_state == 'o') {
          luminaire.state = LuminaireState::OFF;
          luminaire.reference = 0;
      } 
      else if (occ_state == 'l') {
          luminaire.state = LuminaireState::LOW;
          luminaire.reference = 15;
      } 
      else if (occ_state == 'h') {
          luminaire.state = LuminaireState::HIGH;
          luminaire.reference = 30;
      }
    }

    // Other set commands ('u', 'r', 'a', 'f', 'U', 'O', 'C')

    else if (msgIn.cmd == 'u') luminaire.current_u = msgIn.value;
    else if (msgIn.cmd == 'r') luminaire.reference = msgIn.value;
    else if (msgIn.cmd == 'a') pid.anti_windup = (msgIn.value > 0.5f);
    else if (msgIn.cmd == 'f') luminaire.feedback_on = (msgIn.value > 0.5f);
    else if (msgIn.cmd == 'U') luminaire.low_bound = msgIn.value;
    else if (msgIn.cmd == 'O') luminaire.high_bound = msgIn.value;  
    else if (msgIn.cmd == 'C') global_energy_cost = msgIn.value;

    // Read requests (GET 'g')
    else if (msgIn.cmd == 'g') {
      ProtocolMsg_t response;
      response.dest_id = msgIn.src_id; // Reply to the requester (The ID with priority)
      response.src_id = node_address;
      response.cmd = 'A';              // Answer
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
      else if (msgIn.subcmd == 'a') response.value = pid.anti_windup ? 1.0f : 0.0f;
      else if (msgIn.subcmd == 'f') response.value = luminaire.feedback_on ? 1.0f : 0.0f;

      xQueueSend(canTxQueue, &response, 0);
    }
    
    // Incoming Answers('A') - Our previous questions answered
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

    // Stream and buffer (requests and incoming data)
    
    // Streaming
    else if (msgIn.cmd == 's') { // Start stream request
        net_streaming = true;
        net_stream_var = msgIn.subcmd;
        net_stream_dest = msgIn.src_id;
    }
    else if (msgIn.cmd == 'S') { // Stop stream request
        net_streaming = false;
    }
    else if (msgIn.cmd == 'D') { // Incoming Stream Data Payload ('D')
        Serial.printf("s %c %d %.4f %lu\n", msgIn.subcmd, msgIn.src_id, msgIn.value, millis());
    }
    
    // BUFFER 
    else if (msgIn.cmd == 'b') { // Buffer Requests OR Markers
        if (msgIn.value == -1.0f) {        // 1. INCOMING Start Marker
            Serial.printf("b %c %d ", msgIn.subcmd, msgIn.src_id);
        } else if (msgIn.value == -2.0f) { // 2. INCOMING Stop Marker
            Serial.println();
        } else {                           // 3. Actual REQUEST to start sending buffer
            net_sending_buffer = true;
            net_buffer_var = msgIn.subcmd;
            net_buffer_index = 0;
            net_buffer_dest = msgIn.src_id;
            
            // Send Start Marker (-1.0) back to the requester
            ProtocolMsg_t startMsg = {node_address, msgIn.src_id, 'b', msgIn.subcmd, -1.0f};
            xQueueSend(canTxQueue, &startMsg, 0);
        }
    }
    else if (msgIn.cmd == 'B') { // INCOMING Buffer Payload Data ('B')
        Serial.printf("%.2f,", msgIn.value);
    }

    // ---------------------------------------------------------
    // 7. WAKE-UP & DISCOVERY HANDSHAKE ('w' e 'W')
    // ---------------------------------------------------------
    else if (msgIn.cmd == 'w') { // Someone broadcasted a wake-up message
        addNode(msgIn.src_id);
        // Reply DIRECTLY to that node (not broadcast) so it knows we exist
        ProtocolMsg_t replyMsg = {node_address, msgIn.src_id, 'W', ' ', 0.0f};
        xQueueSend(canTxQueue, &replyMsg, 0);
    }
    else if (msgIn.cmd == 'W') { // Someone replied directly to our Wake-Up broadcast
        addNode(msgIn.src_id);
    }

  } // End of the processing loop
}


// CAN communication functions - core 1
void sendMessage() {
  ProtocolMsg_t msgOut;
  // If Core 0 put messages in the TX queue, Core 1 dequeues and sends them.
  // Since the queue was created at the very beginning of setup(), this is safe!
  if (xQueueReceive(canTxQueue, &msgOut, 0) == pdPASS) {
    canMsgTx.can_id = node_address;
    canMsgTx.can_dlc = 8; 
    
    canMsgTx.data[1] = msgOut.dest_id;
    canMsgTx.data[0] = msgOut.src_id;
    canMsgTx.data[2] = msgOut.cmd;
    canMsgTx.data[3] = msgOut.subcmd;
    
    // Safely copy the 4 bytes of the Float into the CAN payload array
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
  // Read from the MCP2515 Hardware buffer
  while((err = can0.readMessage(&canMsgRx)) == MCP2515::ERROR_OK){
    
    uint8_t target_dest = canMsgRx.data[1];
    uint8_t sender = canMsgRx.data[0];
    
    Serial.printf("[CAN RX] Physically received a message from node %d to node %d\n", sender, target_dest);
    // Only process the message if it is for our ID or if it's Broadcast (255)
    if (target_dest == node_address || target_dest == 255) {
      ProtocolMsg_t msgIn;
      msgIn.dest_id = canMsgRx.data[1]; 
      msgIn.src_id  = canMsgRx.data[0];
      msgIn.cmd     = canMsgRx.data[2];
      msgIn.subcmd  = canMsgRx.data[3];
      // Reconstruct the Float from the received CAN bytes
      memcpy(&msgIn.value, &canMsgRx.data[4], sizeof(float));
      // Enqueue to the RX queue for Core 0 to process when it's ready
      xQueueSend(canRxQueue, &msgIn, 0);
    }
  }
}

// Control loop function (PID and Sensors)
void runControlLoop() {
  unsigned long current_micros = micros();
  if (last_micros > 0) {
    long period = (long) (current_micros - last_micros);
    long dev = abs(period - 10000); 
    if (dev > max_jitter) max_jitter = dev;
  }
  last_micros = current_micros;
  float y_k = ldr.readLux();
  float K = box.get_gain(); 
  float d_k = box.fixed_background(ldr);
  float u_k;
  float u_ff = 0;
  if (luminaire.feedback_on) {
    if (luminaire.feedforward_on && K > 0.001f) { 
      u_ff = (luminaire.reference - d_k) / K;
    } else{
      u_ff = 0.0f;
    }

    float u_pid = pid.compute_control(luminaire.reference, y_k);
    u_k = u_pid + u_ff;
    u_k = constrain(u_k, 0.0f, 1.0f); 
    float u_feedback = u_k - u_ff;
    pid.housekeep(luminaire.reference, y_k, u_feedback);
  } else {
    u_k = luminaire.current_u;
    pid.housekeep(luminaire.reference, y_k, u_k);
  }
  
  led.setDuty(u_k);
  luminaire.current_u = u_k;
  // Update the current actuation variable
  metrics.update(u_k, y_k, luminaire.reference, 0.01f);
  // Local streaming
  if (luminaire.streaming) {
    Serial.print("s ");
    Serial.print(luminaire.stream_var);
    Serial.print(" ");
    Serial.print(luminaire.getId()); 
    Serial.print(" "); 

    if (luminaire.stream_var == 'b') {
        Serial.print(y_k, 2);
        Serial.print(" ");
        Serial.print(u_k, 4);
        Serial.print(" "); 
        Serial.print(luminaire.reference, 2);  
    } else {
        float val = (luminaire.stream_var == 'y') ?
        y_k : u_k;
        Serial.print(val, 4);
    }

    Serial.print(" ");
    Serial.println(millis());
  }

  // Network streaming (100Hz)
  if (net_streaming) {
      ProtocolMsg_t streamMsg;
      streamMsg.dest_id = net_stream_dest;
      streamMsg.src_id = node_address;
      streamMsg.cmd = 'D'; // 'D' indicates Data payload for streaming
      streamMsg.subcmd = net_stream_var;
      streamMsg.value = (net_stream_var == 'y') ? y_k : u_k;
      
      xQueueSend(canTxQueue, &streamMsg, 0);
  }

  // NETWORK BUFFER DRIP-FEED (PREVENTING MCP2515 RX OVERFLOW)
  if (net_sending_buffer) {
      // Mod 11: "Wait some time before sending more messages to the same node"
      // We send a maximum of 2 messages per 10ms control loop to avoid RX overflow
      int burst_limit = 2;
      while (burst_limit > 0 && net_buffer_index < 6000) { 
          ProtocolMsg_t bufMsg;
          bufMsg.dest_id = net_buffer_dest;
          bufMsg.src_id = node_address;
          bufMsg.cmd = 'B'; // 'B' indicates Buffer Payload
          bufMsg.subcmd = net_buffer_var;
          bufMsg.value = metrics.getBufferValue(net_buffer_var, net_buffer_index);
          
          if (xQueueSend(canTxQueue, &bufMsg, 0) == pdPASS) {
              net_buffer_index++;
              burst_limit--;
          } else {
              break;
              // TX Queue full (TX Buffer Busy). Wait for the next 10ms cycle.
          }
      }
      
      // When all 6000 elements are successfully sent
      if (net_buffer_index >= 6000) {
          net_sending_buffer = false;
          // Send Stop Marker (-2.0)
          ProtocolMsg_t endMsg = {node_address, net_buffer_dest, 'b', net_buffer_var, -2.0f};
          xQueueSend(canTxQueue, &endMsg, 0);
      }
  }

}


// CAN error handling - core 1
void handleCANErrors() {
    uint8_t eflg = can0.getErrorFlags();
    
    // Variável estática para memorizar o estado físico da rede no ciclo anterior
    // Isto substitui a regra do tempo! Só avisamos quando o estado MUDA.
    static uint8_t last_eflg = 0; 

    // 1. Limpeza de Overflows (Prioridade de Hardware - Slide 9 e 10)
    // Se os buffers encherem, o chip tem de ser limpo imediatamente para não perder pacotes.
    if (eflg & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) {
        can0.clearRXnOVRFlags(); 
        can0.clearInterrupts();  
        
        // Imprime o aviso de overflow APENAS se for a primeira vez que deteta este bloqueio
        if (!(last_eflg & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR))) {
            Serial.println("\n[RECUPERAÇÃO] Flags de Overflow limpas. Rede restaurada!");
        }
    }

    // 2. Máquina de Estados de Diagnóstico (Deteção Instantânea e Sem Loops)
    if (eflg != last_eflg) {
        last_eflg = eflg; // Atualiza a memória para o novo estado

        if (eflg != 0) {
            Serial.println("\n[CAN DIAGNOSTICS] Alteração no estado físico do Bus:");
            
            // Slide 10: Verificação de todas as flags relevantes do registo EFLG
            if (eflg & MCP2515::EFLG_TXBO)   Serial.println(" -> ERRO CRÍTICO: Bus-Off (Muitas falhas de TX)!"); 
            if (eflg & MCP2515::EFLG_TXEP)   Serial.println(" -> AVISO: TX Error-Passive (TEC >= 128)");
            if (eflg & MCP2515::EFLG_RXEP)   Serial.println(" -> AVISO: RX Error-Passive (REC >= 128)");
            if (eflg & MCP2515::EFLG_TXWAR)  Serial.println(" -> AVISO: TX Error Warning (TEC >= 96)");
            if (eflg & MCP2515::EFLG_RXWAR)  Serial.println(" -> AVISO: RX Error Warning (REC >= 96)");
            
            Serial.println("------------------------------------------------");
            
            // Limpa as flags de interrupção (MERRF e ERRIF) para o chip poder atualizar o EFLG no futuro
            can0.clearERRIF();
            can0.clearMERR();
        } else {
            // Se o eflg voltou a 0, significa que o hardware recuperou sozinho!
            Serial.println("\n[CAN DIAGNOSTICS] A rede estabilizou. Erros físicos resolvidos.");
        }
    }

    // 3. Recuperação Extrema (Self-Healing de Bus-Off - Slide 5)
    // Quando o chip entra em Bus-Off, ele desliga-se fisicamente da rede e não recupera sozinho.
    // A única solução fiável é reiniciar o próprio MCP2515.
    if (eflg & MCP2515::EFLG_TXBO) {
        Serial.println("\n[RECUPERAÇÃO] A reiniciar o controlador MCP2515 para sair de Bus-Off...");
        can0.reset();
        can0.setBitrate(CAN_1000KBPS);
        can0.setNormalMode();
        last_eflg = 0; // Forçamos o reset da memória para garantir que o sistema avalia a rede limpa
        
        // Pequeno delay (10ms) para o hardware do MCP2515 "respirar" antes do próximo ciclo
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}
