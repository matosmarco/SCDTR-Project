#include "led.h"
#include "ldr.h"
#include "box.h"
#include "command.h"
#include "metrics.h"
#include "pid.h"

// Objects 
LED led;
LDR ldr;
Box box;
Metrics metrics(0.01963); 
Luminaire luminaire;
pid pid(0.01);         

//Time variables and buffer
unsigned long last_time = 0;
const unsigned long sample_time = 10; // 10ms = 100Hz 
String inputBuffer = "";              // Buffer for non blocking commands

// Global variables of time and diagnosis
unsigned long last_micros = 0; // Timestamp of the last execution
long max_jitter = 0;           // Maximum deviation in microseconds



void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  
  analogReadResolution(12); 
  led.initPWM();            

  luminaire.setId(0);
  //luminaire.setId(1);

  Serial.println("System Initializing...");
  box.identify_static_gain(led, ldr);
  
  Serial.println("System Ready");
  last_time = millis();
}

void loop() {
  // Serial processing 
  checkSerial();

  // Ensure sincronization to 100 Hz to the control loop
  unsigned long current_time = millis();
  if (current_time - last_time >= sample_time) {
    last_time += sample_time; 
    runControlLoop(); 
  }
}

/**
 * Lê caracteres da porta Serial sem bloquear o loop principal.
 */
void checkSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      // Quando recebe o fim de linha, processa o comando acumulado
      processCommand(inputBuffer, led, ldr, box, metrics, luminaire, pid);
      inputBuffer = ""; // Limpa o buffer para o próximo comando
    } else if (c != '\r') {
      inputBuffer += c; // Acumula caracteres
    }
  }
}

/**
 * Lógica em tempo real executada a cada 10ms.
 */
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
  // delta_t = 0.01s (10ms)

  //float lux_estimated = (K*u_k) + d_k; // Estimated illuminance 
  //metrics.update(u_k, lux_estimated, luminaire.reference, 0.01f);
  metrics.update(u_k, y_k, luminaire.reference, 0.01f);
  // Streaming in real time ('s' command, defined in command.cpp)
// Streaming em tempo real (Comando 's' ou 'sb')
if (luminaire.streaming) {
    Serial.print("s ");
    Serial.print(luminaire.stream_var);
    Serial.print(" ");
    Serial.print(luminaire.getId()); // ID 0
    Serial.print(" "); // Espaço vital

    if (luminaire.stream_var == 'b') {
        Serial.print(y_k, 2);
        Serial.print(" "); // Espaço vital
        Serial.print(u_k, 4);
        Serial.print(" "); // Espaço vital
        Serial.print(luminaire.reference, 2); // Current target (r) 
    } else {
        float val = (luminaire.stream_var == 'y') ? y_k : u_k;
        Serial.print(val, 4);
    }

    Serial.print(" "); // Espaço vital antes do tempo
    Serial.println(millis()); // println garante o \n para o Python
  }
  
  }
