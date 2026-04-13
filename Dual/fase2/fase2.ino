#include "led.h"
#include "ldr.h"
#include "box.h"
#include "command.h"
#include "metrics.h"
#include "config.h"
#include "mcp2515.h"
#include "pico/unique_id.h"
#include "dualdecomposition.h" // Distributed algorithm
#include "pid.h"
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include "calibrationMatrix.h"
#include "calibrationFSM.h"
#include "network.h"
#include "admm.h"

// FreeRTOS thread-safe queues for inter-core communication
QueueHandle_t canTxQueue;
QueueHandle_t canRxQueue; 

// Calibration FSM and Matrix
CalibrationMatrix myMatrix(0); // Will set ID later
CalibrationFSM* fsm = nullptr;
TaskHandle_t calibTaskHandle = NULL;

// Dual Decomposition (Distributed Solver)
DualDecomposition* dualDecomp = nullptr; 
bool distributed_ctrl_active = true; // Toggle between Distributed and Open-Loop

// ADMM
ADMM* admmDecomp = nullptr;
// Network control variables 
bool net_streaming = false;
char net_stream_var = 'y';
uint8_t net_stream_dest = 0;

bool net_sending_buffer = false;
char net_buffer_var = 'y';
int net_buffer_index = 0;
uint8_t net_buffer_dest = 0;

extern float global_K_matrix[MAX_NODES][MAX_NODES];
// Global objects 
LED led;
LDR ldr;
Box box;
pid pid(0.01);
Metrics metrics(0.01963); 
Luminaire luminaire;      
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};
float global_energy_cost = 1.0f;

pico_unique_board_id_t pico_board_id; 
uint8_t node_address = 0; 
struct can_frame canMsgTx, canMsgRx;
MCP2515::ERROR err;

String inputBuffer = "";              
unsigned long last_micros = 0;
long max_jitter = 0;

unsigned long last_seen_time[256] {0};

static float dd_u_setpoint = 0.0f;  // last converged duty (0.0 to 1.0)
static float admm_lux_reference = 0.0f; // NEW: The expected lux for the PID to follow!

// FreeRTOS task: Distributed controller (core 0)
void controllerTask_dd(void *pvParameters) {
    TickType_t xLastWakeTime;

    // Control loop: 100 Hz for LED output (smooth hold of setpoint)
    const TickType_t xCtrlPeriod = pdMS_TO_TICKS(10);
    
    // Optimization loop: 5 Hz (every 200ms) — reduces flicker, matches CAN round-trip
    const TickType_t xOptPeriod  = pdMS_TO_TICKS(100);
    TickType_t xLastOptTime      = xTaskGetTickCount();

    static bool matrix_requested = false;

    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xCtrlPeriod);

        if (fsm == nullptr || !fsm->isCalibrated) continue;

        // --- Matrix exchange (one-shot on first calibration) ---
        if (!matrix_requested) {
            Serial.println("\n[DD] Requesting K-matrix from neighbours...");
            ProtocolMsg_t reqK = {node_address, 255, 'g', 'k', 0.0f};
            xQueueSend(canTxQueue, &reqK, 0);

            // Store own row in global matrix
            int my_idx = myMatrix.findNodeIndex(node_address);
            int n = myMatrix.getNumNodes();
            const uint8_t* nodes = myMatrix.getNodesArray();
            if (my_idx != -1) {
                for (int i = 0; i < n; i++) {
                    int col_idx = myMatrix.findNodeIndex(nodes[i]);
                    if (col_idx != -1)
                        global_K_matrix[my_idx][col_idx] = myMatrix.getGain(nodes[i]);
                }
            }

            matrix_requested = true;
            vTaskDelay(pdMS_TO_TICKS(2000)); // wait for neighbours to reply
            Serial.println("[DD] Matrix ready. Starting optimization.");
            continue;
        }

        // --- Optimization step at 5 Hz ---
        TickType_t now = xTaskGetTickCount();
        if ((now - xLastOptTime) >= xOptPeriod) {
            xLastOptTime = now;

            if (dualDecomp != nullptr && distributed_ctrl_active) {
                dualDecomp->set_c(global_energy_cost);

                int my_idx = myMatrix.findNodeIndex(node_address);
                float k_ii = (my_idx != -1) ? global_K_matrix[my_idx][my_idx] : 0.0f;
                // LÊ A REALIDADE AQUI (Luz atual do sensor)
                float y_meas = ldr.readLux();
                float L      = luminaire.reference;

                // Primal update
                float u_dd_100 = dualDecomp->compute_u(k_ii, myMatrix);
                Serial.printf("u %.4f\n", u_dd_100); //TOCHECK
                // Dual update (Agora Adaptativo!)
                dualDecomp->compute_l(y_meas, L);
                Serial.printf("LAMBDA %.4f\n", dualDecomp->get_l()); //TOCHECK
                // Convert 0-100 scale to 0.0-1.0 for LED
                dd_u_setpoint = constrain(u_dd_100 / 100.0f, 0.0f, 1.0f);

                // Broadcast u to neighbours (0-100 scale, consistent with algorithm)
                ProtocolMsg_t uMsg = {node_address, 255, 'D', 'u', u_dd_100};
                xQueueSend(canTxQueue, &uMsg, 0);

                // Send prices to each neighbour
                const uint8_t* nodes = myMatrix.getNodesArray();
                int n = myMatrix.getNumNodes();
                for (int i = 0; i < n; i++) {
                    uint8_t other_id = nodes[i];
                    if (other_id != node_address) {
                        float p_ij = dualDecomp->compute_price_for(other_id, myMatrix);
                        ProtocolMsg_t priceMsg = {node_address, other_id, 'P', ' ', p_ij};
                        xQueueSend(canTxQueue, &priceMsg, 0);
                    }
                }
            }
        }

        // --- Apply setpoint at 100 Hz (steady, no flicker) ---
        runControlLoop();
    }
}

void controllerTask_admm(void *pvParameters) {
    TickType_t xLastWakeTime;

    // Malha de controlo (PID): 100 Hz para manter a estabilidade física
    const TickType_t xCtrlPeriod = pdMS_TO_TICKS(10);
    // Malha de otimização (ADMM): ~8 Hz (120ms) para permitir propagação na rede CAN
    const TickType_t xOptPeriod  = pdMS_TO_TICKS(120);
    TickType_t xLastOptTime      = xTaskGetTickCount();

    static bool matrix_requested = false;
    
    // Desfasa o arranque inicial para evitar colisões no CAN Bus
    vTaskDelay(pdMS_TO_TICKS(node_address * 5));
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xCtrlPeriod);

        if (fsm == nullptr || !fsm->isCalibrated) continue;

        // --- Inicialização da Matriz K (Executa uma vez após calibração) ---
        if (!matrix_requested) {
            Serial.println("\n[ADMM] A solicitar Matriz K aos vizinhos...");
            ProtocolMsg_t reqK = {node_address, 255, 'g', 'k', 0.0f};
            xQueueSend(canTxQueue, &reqK, 0);

            int my_idx = myMatrix.findNodeIndex(node_address);
            int n = myMatrix.getNumNodes();
            const uint8_t* nodes = myMatrix.getNodesArray();
            if (my_idx != -1) {
                for (int i = 0; i < n; i++) {
                    int col_idx = myMatrix.findNodeIndex(nodes[i]);
                    if (col_idx != -1)
                        global_K_matrix[my_idx][col_idx] = myMatrix.getGain(nodes[i]);
                }
            }
            matrix_requested = true;
            vTaskDelay(pdMS_TO_TICKS(2000)); // Espera pelas respostas dos vizinhos
            continue;
        }

        // --- Passo de Otimização ADMM ---
        TickType_t now = xTaskGetTickCount();
        if ((now - xLastOptTime) >= xOptPeriod) {
            xLastOptTime = now;
            
            if (distributed_ctrl_active && admmDecomp != nullptr) {
                float L = luminaire.reference;
                
                // 1. Atualizar parâmetros de custo e ganhos
                admmDecomp->update_params(global_energy_cost, myMatrix, global_K_matrix);
                
                // 2. Ler o sensor real (Luz atual na secretária)
                float y_meas = ldr.readLux();
                
                // 3. Executar iteração de consenso (PGD + Projeções)
                float u_admm = admmDecomp->consensus_iterate(y_meas, L);
                
                // 4. Atualizar setpoint base (0.0 a 1.0) para uso em caso de falha do PID
                dd_u_setpoint = constrain(u_admm / 100.0f, 0.0f, 1.0f);

                // =======================================================
                // CORREÇÃO 1: CÁLCULO DA LUZ ESPERADA PARA O PID
                // =======================================================
                int n = myMatrix.getNumNodes();
                const float* current_u_array = admmDecomp->get_u();
                const uint8_t* nodes = myMatrix.getNodesArray();
                int my_idx = myMatrix.findNodeIndex(node_address);
                
                // Começamos com a luz de fundo (background) medida no setup
                float expected_lux = box.fixed_background(ldr);
                
                if (my_idx != -1) {
                    for (int i = 0; i < n; i++) {
                        // IMPORTANTE: Usa a verdade física: o que o vizinho i disse 
                        // que está a fazer no LED dele
                        float u_real = admmDecomp->get_physical_u(i);
                        expected_lux += (global_K_matrix[my_idx][i] / 100.0f) * u_real;
                    }
                }
                // O PID agora persegue a luz que DEVERIA existir fisicamente
                admm_lux_reference = expected_lux;

                // =======================================================
                // CORREÇÃO 2: CRITÉRIO DE CONVERGÊNCIA GLOBAL E HEARTBEAT
                // =======================================================
                static float last_u_array[MAX_NODES];
                bool converged = true;

                // Verificamos se todo o vetor de estimativas estabilizou
                for (int i = 0; i < n; i++) {
                    if (abs(current_u_array[i] - last_u_array[i]) > 0.2f) {
                        converged = false;
                        break;
                    }
                }

                // FIX: HEARTBEAT (Bater do coração)
                // Impede que o vizinho fique preso a tentar ajudar-nos para sempre
                // se a nossa última mensagem de "já não preciso de ajuda" se perder.
                static int heartbeat_counter = 0;
                heartbeat_counter++;

                // Transmite se não convergiu OU a cada 10 iterações (~1.2 segundos)
                if (!converged || heartbeat_counter > 10) {
                    if (heartbeat_counter > 10) heartbeat_counter = 0;

                    // 5. Transmitir estimativas via CAN (Comando 'M')
                    for (int i = 0; i < n; i++) {
                        ProtocolMsg_t msgADMM = {node_address, 255, 'M', (char)nodes[i], current_u_array[i]};
                        xQueueSend(canTxQueue, &msgADMM, pdMS_TO_TICKS(5));
                        
                        // Atualiza memória de convergência
                        last_u_array[i] = current_u_array[i];
                        vTaskDelay(pdMS_TO_TICKS(2)); // Pequena pausa para não saturar o bus
                    }
                }
            }
        }

        // --- Execução da malha de controlo PID (100 Hz) ---
        runControlLoop_admm();
    }
}
// FreeRTOS Task: Calibration orchestration (core 0)
void calibrationTask(void *pvParameters) {
    calibTaskHandle = xTaskGetCurrentTaskHandle();

    // Trigger initial discovery on startup
    xTaskNotify(calibTaskHandle, EVENT_NETWORK_CHANGE, eSetBits);

    for (;;) {
        // Block and wait for events (or timeout every 100ms to keep FSM ticking)
        uint32_t events = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        
        if (fsm != nullptr) {
            fsm->process(events);
        }
    }
}
    
// Core 0: Setup and Loop 
void setup() {
  // Create Queues BEFORE any blocking call (Serial or Task)
  canTxQueue = xQueueCreate(QUEUE_SIZE, sizeof(ProtocolMsg_t));
  canRxQueue = xQueueCreate(QUEUE_SIZE, sizeof(ProtocolMsg_t));

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

  // Initialize Dual Decomposition solver
  dualDecomp = new DualDecomposition(node_address, global_energy_cost, 0.05f, 0.004f);

  // Initialize ADMM solver (rho = 0.02, como no Matlab)
  //admmDecomp = new ADMM(node_address, 0.05f);

  // Initialize matrix and State machine (FSM) with the right ID
  myMatrix.setMyId(node_address);
  luminaire.setId(node_address);
  fsm = new CalibrationFSM(myMatrix, led, ldr, box, node_address);

  // Task Creation
  xTaskCreate(controllerTask_dd, "CtrlTask DD", 4096, NULL, 2, NULL);
  
  xTaskCreate(calibrationTask, "CalibTask ADMM", 4096, NULL, 1, NULL);

  //xTaskCreate(controllerTask_admm, "CtrlTask", 4096, NULL, 2, NULL);

  // Add node to the matrix
  myMatrix.addNode(node_address);
  
  // Wake-Up Broadcast
  ProtocolMsg_t wakeupMsg = {node_address, 255, 'w', ' ', 0.0f};
  xQueueSend(canTxQueue, &wakeupMsg, 0);
  Serial.printf("Node %d Wake-Up Broadcasted!\n", node_address);
}

void loop() {
  checkSerial();
  processNetworkMessages();

  // Read time using FreeRTOS non-blocking function
  unsigned long current_time = (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS);

  // Presence ping - every 2 seconds
  static unsigned long last_heartbeat = 0;
  if (current_time - last_heartbeat > 2000) {
      last_heartbeat = current_time;
      ProtocolMsg_t pingMsg = {node_address, 255, 'H', ' ', 0.0f}; 
      xQueueSend(canTxQueue, &pingMsg, 0);
  }

  // Watchdog (Clean inactive nodes - 5 seconds timeout)
  if (fsm != nullptr) {
      const uint8_t* nodes = myMatrix.getNodesArray();
      int numNodes = myMatrix.getNumNodes();
      
      for (int i = 0; i < numNodes; i++) {
          uint8_t id = nodes[i];
          if (id != node_address) {
            if (current_time - last_seen_time[id] > 5000) { 
                Serial.printf("\n TOPOLOGY: Node %d left the network (Timeout)!\n", id);
                myMatrix.removeNode(id);
                
                if (net_stream_dest == id) net_streaming = false;
                if (net_buffer_dest == id) net_sending_buffer = false;

                // Colocar a zeros a linha e a coluna do nó que desapareceu
                for(int j = 0; j < MAX_NODES; j++) {
                    global_K_matrix[id - 1][j] = 0.0f; 
                    global_K_matrix[j][id - 1] = 0.0f; 
                }

                break;
            }
          }
      }
  }

  vTaskDelay(1); // Keeps FreeRTOS breathing (Idle task execution)
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
  Serial.print("Assigned Node Address: ");
  Serial.println(node_address);
  luminaire.setId(node_address); 
  
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode(); 
}

void loop1() {
  sendMessage();
  readMessage();
  handleCANErrors();
  //vTaskDelay(1);
}

void runControlLoop() {
    unsigned long current_micros = micros();
    if (last_micros > 0) {
        long period = (long)(current_micros - last_micros);
        long dev = abs(period - 10000);
        if (dev > max_jitter) max_jitter = dev;
    }
    last_micros = current_micros;

    float y_k = ldr.readLux();
    float K   = box.get_gain();
    float d_k = box.fixed_background(ldr);
    float u_k;

    if (fsm != nullptr && fsm->isCalibrated && dualDecomp != nullptr && distributed_ctrl_active) {
        // Simply apply the setpoint computed by the optimization task.
        // No re-running the algorithm here — that would cause 100Hz oscillation.
        u_k = dd_u_setpoint;

    } else {
        // FALLBACK: feedforward or manual
        if (luminaire.feedforward_on && K > 0.001f && luminaire.reference > 0) {
            u_k = (luminaire.reference - d_k) / K;
        } else {
            u_k = luminaire.current_u;
        }
        u_k = constrain(u_k, 0.0f, 1.0f);
    }

    led.setDuty(u_k);
    luminaire.current_u = u_k;

    metrics.update(u_k, y_k, luminaire.reference, 0.01f);

    /*// Local streaming
    if (luminaire.streaming) {
        Serial.print("s ");
        Serial.print(luminaire.stream_var);
        Serial.print(" ");
        Serial.print(luminaire.getId());
        Serial.print(" ");
        if (luminaire.stream_var == 'b') {
            Serial.print(y_k, 2); Serial.print(" ");
            Serial.print(u_k, 4); Serial.print(" ");
            Serial.print(luminaire.reference, 2);
        } else {
            float val = (luminaire.stream_var == 'y') ? y_k : u_k;
            Serial.print(val, 4);
        }
        Serial.print(" ");
        Serial.println(millis());
    }

    // Network streaming
    if (net_streaming) {
        ProtocolMsg_t streamMsg;
        streamMsg.dest_id = net_stream_dest;
        streamMsg.src_id  = node_address;
        streamMsg.cmd     = 'D';
        streamMsg.subcmd  = net_stream_var;
        streamMsg.value   = (net_stream_var == 'y') ? y_k : u_k;
        xQueueSend(canTxQueue, &streamMsg, 0);
    }*/

    // =======================================================
    // STREAMING LOCAL (Otimizado para o Serial Plotter)
    // =======================================================
    if (luminaire.streaming) {
        if (luminaire.stream_var == 'a') {
            // Formato perfeito para ver gráficos no Arduino IDE (Ferramentas -> Serial Plotter)
            // Multiplico o u_k por 100 para a escala do gráfico ficar visível com a referência
            Serial.printf("Lux:%.2f Duty:%.2f Ref:%.2f\n", y_k, u_k * 100.0f, luminaire.reference);
        } else {
            Serial.print("s "); Serial.print(luminaire.stream_var); Serial.print(" ");
            Serial.print(luminaire.getId()); Serial.print(" ");
            float val = (luminaire.stream_var == 'y') ? y_k : u_k;
            Serial.print(val, 4); Serial.print(" "); Serial.println(millis());
        }
    }

    // =======================================================
    // STREAMING DE REDE (CAN Bus)
    // =======================================================
    if (net_streaming) {
        ProtocolMsg_t streamMsg;
        streamMsg.dest_id = net_stream_dest;
        streamMsg.src_id  = node_address;
        streamMsg.cmd     = 'D';
        
        if (net_stream_var == 'a') {
            // O CAN Bus só leva 1 valor por pacote, por isso enviamos 3 pacotes seguidos muito rápido
            streamMsg.subcmd = 'y'; streamMsg.value = y_k; 
            xQueueSend(canTxQueue, &streamMsg, 0);
            
            streamMsg.subcmd = 'u'; streamMsg.value = u_k; 
            xQueueSend(canTxQueue, &streamMsg, 0);
            
            streamMsg.subcmd = 'r'; streamMsg.value = luminaire.reference; 
            xQueueSend(canTxQueue, &streamMsg, 0);
        } else {
            streamMsg.subcmd  = net_stream_var;
            streamMsg.value   = (net_stream_var == 'y') ? y_k : u_k;
            xQueueSend(canTxQueue, &streamMsg, 0);
        }
    }

    // Network buffer drip-feed
    if (net_sending_buffer) {
        int burst_limit = 2;
        while (burst_limit > 0 && net_buffer_index < 6000) {
            ProtocolMsg_t bufMsg;
            bufMsg.dest_id = net_buffer_dest;
            bufMsg.src_id  = node_address;
            bufMsg.cmd     = 'B';
            bufMsg.subcmd  = net_buffer_var;
            bufMsg.value   = metrics.getBufferValue(net_buffer_var, net_buffer_index);
            if (xQueueSend(canTxQueue, &bufMsg, 0) == pdPASS) {
                net_buffer_index++;
                burst_limit--;
            } else {
                break;
            }
        }
        if (net_buffer_index >= 6000) {
            net_sending_buffer = false;
            ProtocolMsg_t endMsg = {node_address, net_buffer_dest, 'b', net_buffer_var, -2.0f};
            xQueueSend(canTxQueue, &endMsg, 0);
        }
    }
}

void runControlLoop_admm() {
    unsigned long current_micros = micros();
    if (last_micros > 0) {
        long period = (long)(current_micros - last_micros);
        long dev = abs(period - 10000);
        if (dev > max_jitter) max_jitter = dev;
    }
    last_micros = current_micros;
    
    float y_k = ldr.readLux();
    float K   = box.get_gain();
    float d_k = box.fixed_background(ldr);
    float u_k;
    
    if (fsm != nullptr && fsm->isCalibrated && admmDecomp != nullptr && distributed_ctrl_active) {
        
        // --- A MAGIA DO PID LIGADO AO ADMM ---
        if (luminaire.feedback_on) {
            // O PID vai seguir a Luz Esperada calculada pelo ADMM para combater perturbações externas rapidamente!
            u_k = pid.compute_control(admm_lux_reference, y_k);
        } else {
            // Sem feedback, confiamos cegamente no duty cycle puro que o ADMM nos diz para aplicar
            u_k = dd_u_setpoint; 
        }
        
        u_k = constrain(u_k, 0.0f, 1.0f);
        
        if (luminaire.feedback_on) {
            // Housekeep para anti-windup com a referência da luz esperada
            pid.housekeep(admm_lux_reference, y_k, u_k); 
        }

    } else {
        // FALLBACK: feedforward or manual
        if (luminaire.feedforward_on && K > 0.001f && luminaire.reference > 0) {
            u_k = (luminaire.reference - d_k) / K;
        } else {
            u_k = luminaire.current_u;
        }
        u_k = constrain(u_k, 0.0f, 1.0f);
    }

    led.setDuty(u_k);
    luminaire.current_u = u_k;

    metrics.update(u_k, y_k, luminaire.reference, 0.01f);

    /*// Local streaming
    if (luminaire.streaming) {
        Serial.print("s ");
        Serial.print(luminaire.stream_var);
        Serial.print(" ");
        Serial.print(luminaire.getId());
        Serial.print(" ");
        if (luminaire.stream_var == 'b') {
            Serial.print(y_k, 2);
            Serial.print(" ");
            Serial.print(u_k, 4); Serial.print(" ");
            Serial.print(luminaire.reference, 2);
        } else {
            float val = (luminaire.stream_var == 'y') ? y_k : u_k;
            Serial.print(val, 4);
        }
        Serial.print(" ");
        Serial.println(millis());
    }

    // Network streaming
    if (net_streaming) {
        ProtocolMsg_t streamMsg;
        streamMsg.dest_id = net_stream_dest;
        streamMsg.src_id  = node_address;
        streamMsg.cmd     = 'D';
        streamMsg.subcmd  = net_stream_var;
        streamMsg.value   = (net_stream_var == 'y') ? y_k : u_k;
        xQueueSend(canTxQueue, &streamMsg, 0);
    }*/

    // =======================================================
    // STREAMING LOCAL (Otimizado para o Serial Plotter)
    // =======================================================
    if (luminaire.streaming) {
        if (luminaire.stream_var == 'a') {
            // Formato perfeito para ver gráficos no Arduino IDE (Ferramentas -> Serial Plotter)
            // Multiplico o u_k por 100 para a escala do gráfico ficar visível com a referência
            Serial.printf("Lux:%.2f Duty:%.2f Ref:%.2f\n", y_k, u_k * 100.0f, luminaire.reference);
        } else {
            Serial.print("s "); Serial.print(luminaire.stream_var); Serial.print(" ");
            Serial.print(luminaire.getId()); Serial.print(" ");
            float val = (luminaire.stream_var == 'y') ? y_k : u_k;
            Serial.print(val, 4); Serial.print(" "); Serial.println(millis());
        }
    }

    // =======================================================
    // STREAMING DE REDE (CAN Bus)
    // =======================================================
    if (net_streaming) {
        ProtocolMsg_t streamMsg;
        streamMsg.dest_id = net_stream_dest;
        streamMsg.src_id  = node_address;
        streamMsg.cmd     = 'D';
        
        if (net_stream_var == 'a') {
            // O CAN Bus só leva 1 valor por pacote, por isso enviamos 3 pacotes seguidos muito rápido
            streamMsg.subcmd = 'y'; streamMsg.value = y_k; 
            xQueueSend(canTxQueue, &streamMsg, 0);
            
            streamMsg.subcmd = 'u'; streamMsg.value = u_k; 
            xQueueSend(canTxQueue, &streamMsg, 0);
            
            streamMsg.subcmd = 'r'; streamMsg.value = luminaire.reference; 
            xQueueSend(canTxQueue, &streamMsg, 0);
        } else {
            streamMsg.subcmd  = net_stream_var;
            streamMsg.value   = (net_stream_var == 'y') ? y_k : u_k;
            xQueueSend(canTxQueue, &streamMsg, 0);
        }
    }

    // Network buffer drip-feed
    if (net_sending_buffer) {
        int burst_limit = 2;
        while (burst_limit > 0 && net_buffer_index < 6000) {
            ProtocolMsg_t bufMsg;
            bufMsg.dest_id = net_buffer_dest;
            bufMsg.src_id  = node_address;
            bufMsg.cmd     = 'B';
            bufMsg.subcmd  = net_buffer_var;
            bufMsg.value   = metrics.getBufferValue(net_buffer_var, net_buffer_index);
            if (xQueueSend(canTxQueue, &bufMsg, 0) == pdPASS) {
                net_buffer_index++;
                burst_limit--;
            } else {
                break;
            }
        }
        if (net_buffer_index >= 6000) {
            net_sending_buffer = false;
            ProtocolMsg_t endMsg = {node_address, net_buffer_dest, 'b', net_buffer_var, -2.0f};
            xQueueSend(canTxQueue, &endMsg, 0);
        }
    }
}