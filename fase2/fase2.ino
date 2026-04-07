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
#include "calibrationMatrix.h"
#include "calibrationFSM.h"
#include "network.h"

// FreeRTOS thread-safe queues for inter-core communication
QueueHandle_t canTxQueue;
QueueHandle_t canRxQueue; 

// Calibration FSM and Matrix
CalibrationMatrix myMatrix(0); // Will set ID later
CalibrationFSM* fsm = nullptr;
TaskHandle_t calibTaskHandle = NULL;

// Network control variables 
bool net_streaming = false;
char net_stream_var = 'y';
uint8_t net_stream_dest = 0;

bool net_sending_buffer = false;
char net_buffer_var = 'y';
int net_buffer_index = 0;
uint8_t net_buffer_dest = 0;

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

String inputBuffer = "";              
unsigned long last_micros = 0; 
long max_jitter = 0;

unsigned long last_seen_time[256] {0};

// FreeRTOS task: Distributed controller (core 0)
void controllerTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Exatamente 10ms (100Hz)
    
    // Initialize variable with current time
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Bloqueia a task até passarem exatamente 10ms desde o último ciclo
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Só executa o controlo se a rede estiver calibrada
        if (fsm != nullptr && fsm->isCalibrated) {
            // Chama a função de controlo (DESCOMENTADO)
            runControlLoop(); 
        }
    }
}

// FreeRTOS Task: Calibration orchestration (core 0)
void calibrationTask(void *pvParameters) {
    calibTaskHandle = xTaskGetCurrentTaskHandle(); // Task address 

  // Trigger initial discovery on startup
  // EVENT_NETWORK_CHANGE - event (flag), that indicates that the system just started 
  // eSetBits - says that the info will be stored and will do an OR operation (overlap of the bits).
    xTaskNotify(calibTaskHandle, EVENT_NETWORK_CHANGE, eSetBits);

    for (;;) {
        // Block and wait for events (or timeout every 100ms to keep FSM ticking)
        uint32_t events = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)); 
        // stop the core action to do other tasks until we receive a notification (1) or the timer ends (0)
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

  // Initialize matrix and State machine (FSM) with the right ID
  myMatrix.setMyId(node_address);
  fsm = new CalibrationFSM(myMatrix, led, ldr, box, node_address);

  // Creation of calibration task (Priority 1)
  xTaskCreate(calibrationTask, "CalibTask", 4096, NULL, 1, NULL);

  // Creation of control task (Priority 2 - highest)
  xTaskCreate(controllerTask, "CtrlTask", 4096, NULL, 2, NULL);
  
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
                  Serial.printf("\n>>> TOPOLOGY: Node %d left the network (Timeout)!\n", id);
                  myMatrix.removeNode(id);
                  
                  // Cancel streams to this node, if any exist
                  if (net_stream_dest == id) net_streaming = false;
                  if (net_buffer_dest == id) net_sending_buffer = false;

                  // Notify Calibration Task that the network topology changed
                  if (calibTaskHandle != NULL) {
                      xTaskNotify(calibTaskHandle, EVENT_NETWORK_CHANGE, eSetBits);
                  }
                  break; // Break the for loop because the array size changed!
              }
          }
      }
  }

  // THE CALL TO runControlLoop() IS NO LONGER HERE BECAUSE 
  // IT RUNS IN controllerTask() WITH MAXIMUM PRIORITY!

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
    } else {
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

  // Network streaming (100Hz)
  if (net_streaming) {
      ProtocolMsg_t streamMsg;
      streamMsg.dest_id = net_stream_dest;
      streamMsg.src_id = node_address;
      streamMsg.cmd = 'D'; 
      streamMsg.subcmd = net_stream_var;
      streamMsg.value = (net_stream_var == 'y') ? y_k : u_k;
      xQueueSend(canTxQueue, &streamMsg, 0);
  }

  // NETWORK BUFFER DRIP-FEED 
  if (net_sending_buffer) {
      int burst_limit = 2;
      while (burst_limit > 0 && net_buffer_index < 6000) { 
          ProtocolMsg_t bufMsg;
          bufMsg.dest_id = net_buffer_dest;
          bufMsg.src_id = node_address;
          bufMsg.cmd = 'B';
          bufMsg.subcmd = net_buffer_var;
          bufMsg.value = metrics.getBufferValue(net_buffer_var, net_buffer_index);
          
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
