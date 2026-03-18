#include "led.h"
#include "ldr.h"
#include "box.h"
#include "command.h"
#include "metrics.h"
#include "pid.h"
#include "can_network.h"
#include "pico/unique_id.h"

// Objects 
LED led;
LDR ldr;
Box box;
Metrics metrics(0.01963); 
Luminaire luminaire;
pid pid(0.01);  
CanNetwork can_net(17);       

//Time variables and buffer
unsigned long last_time = 0;
const unsigned long sample_time = 10; // 10ms = 100Hz 
String inputBuffer = "";              // Buffer for non blocking commands

// Global variables of time and diagnosis
unsigned long last_micros = 0; // Timestamp of the last execution
long max_jitter = 0;           // Maximum deviation in microseconds

uint8_t get_pico_id() {
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    return board_id.id[7]; 
}

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  
  analogReadResolution(12); 
  led.initPWM();            

  // Network Boot: Dynamic ID assignment
  uint8_t my_id = get_pico_id();
  luminaire.setId(my_id);
  can_net.begin();

  Serial.print("System Initializing... Node ID: ");
  Serial.println(my_id);
  box.identify_static_gain(led, ldr);
  
  Serial.println("System Ready");
  last_time = millis();
}

void loop() {
  // Serial processing 
  checkSerial();

  // CAN Bus processing
  checkCanBus();

  // Ensure sincronization to 100 Hz to the control loop
  unsigned long current_time = millis();
  if (current_time - last_time >= sample_time) {
    last_time += sample_time; 
    runControlLoop(); 
  }
}


void checkSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      // Pass the CAN network to the command processor
      processCommand(inputBuffer, led, ldr, box, metrics, luminaire, pid, can_net);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
}

// Process incoming CAN messages
void checkCanBus() {
    CanMessage rxMsg;
    if (can_net.receiveMessage(rxMsg)) {
        
        // Command targeted to this node or broadcast (255)
        if (rxMsg.type == 'C' && (rxMsg.target_id == luminaire.getId() || rxMsg.target_id == 255)) {
            if (rxMsg.variable == 'r') {
                luminaire.reference = rxMsg.value;
            } else if (rxMsg.variable == 'u') {
                luminaire.current_u = rxMsg.value;
                led.setDuty(rxMsg.value);
            } else if (rxMsg.variable == 'f') {
                luminaire.feedback_on = (rxMsg.value > 0.5f);
            }
        }
        
        // Hub Function: route telemetry from the network to the Serial Monitor
        else if (rxMsg.type == 'T') {
            Serial.print("s ");
            Serial.print(rxMsg.variable);
            Serial.print(" ");
            Serial.print(rxMsg.sender_id);
            Serial.print(" ");
            Serial.print(rxMsg.value, 4);
            Serial.print(" ");
            Serial.println(micros());
        }
    }
}


void runControlLoop() {
  // Jitter monitorization
  unsigned long current_micros = micros();
  if (last_micros > 0) {
    long period = (long) (current_micros - last_micros);
    long dev = abs(period - 10000); // Diference to the ideal interval of 10ms
    if (dev > max_jitter) max_jitter = dev;
  }
  last_micros = current_micros;

  // Data acquisition and models
  float y_k = ldr.readLux();
  float K = box.get_gain(); // Identified gain during calibration
  float d_k = box.fixed_background(ldr);
  float u_k;
  float u_ff = 0;
  if (luminaire.feedback_on) {
    // Feedforward component
    if (luminaire.feedforward_on && K > 0.001f) { // to avoid division by 0
      u_ff = (luminaire.reference - d_k) / K;
    } else{
      u_ff = 0.0f;
    }

    // Feedback component
    float u_pid = pid.compute_control(luminaire.reference, y_k);
    
    u_k = u_pid + u_ff;
    u_k = constrain(u_k, 0.0f, 1.0f); // to guarantee that u_k is between 0.0 and 1.0
    float u_feedback = u_k - u_ff;
    pid.housekeep(luminaire.reference, y_k, u_feedback);
    // Combined final signal

  } else {
    u_k = luminaire.current_u;
    pid.housekeep(luminaire.reference, y_k, u_k);
  }
  
  // Actuation and Update
  led.setDuty(u_k);

  // Metrics (Energy (J); Visibility error (Lux) and Flicker (s^-1))
  metrics.update(u_k, y_k, luminaire.reference, 0.01f);
  // Streaming in real time ('s' command, defined in command.cpp)
// Streaming em tempo real (Comando 's' ou 'sb')
if (luminaire.streaming) {
    Serial.print("s ");
    Serial.print(luminaire.stream_var);
    Serial.print(" ");
    Serial.print(luminaire.getId()); // ID 0
    Serial.print(" "); // Espaço vital

    float val = 0.0f; // Variable to hold the value for CAN transmission

    if (luminaire.stream_var == 'b') {
        Serial.print(y_k, 2);
        Serial.print(" "); // Espaço vital
        Serial.print(u_k, 4);
        Serial.print(" "); // Espaço vital
        Serial.print(luminaire.reference, 2); // Current target (r) 
    } else {
        val = (luminaire.stream_var == 'y') ? y_k : u_k;
        Serial.print(val, 4);
    }

    Serial.print(" ");
    //Serial.println(millis());
    Serial.println(micros());

    // Broadcast telemetry to the CAN network (except for 'b' which is dual-variable)
    if (luminaire.stream_var != 'b') {
        CanMessage txMsg;
        txMsg.type = 'T'; // Telemetry
        txMsg.sender_id = luminaire.getId();
        txMsg.target_id = 255; // Broadcast
        txMsg.variable = luminaire.stream_var;
        txMsg.value = val;
        can_net.sendMessage(txMsg);
    }
  }
  
  }