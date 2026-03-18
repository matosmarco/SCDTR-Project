#include "led.h"
#include "ldr.h"
#include "box.h"
#include "command.h"
#include "metrics.h"
#include "pid.h"
#include "can_network.h"
#include "hub_router.h"
#include "pico/unique_id.h"

// Objects 
LED led;
LDR ldr;
Box box;
Metrics metrics(0.01963); 
Luminaire luminaire;
pid pid(0.01);  
CanNetwork can_net(17);       

unsigned long last_time = 0;
const unsigned long sample_time = 10; 
String inputBuffer = "";              

unsigned long last_micros = 0; 
long max_jitter = 0;           

uint8_t get_pico_id() {
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    return board_id.id[7]; 
}

void setup() {
  Serial.begin(115200);
  
  // Espera 3 segundos para garantir que o PC agarra a porta USB
  delay(3000);
  
  analogReadResolution(12); 
  led.initPWM();            

  uint8_t my_id = get_pico_id();
  luminaire.setId(my_id);
  
  Serial.println("A ignorar o CAN Bus para teste de seguranca...");
  
  // DESATIVADO PARA TESTE: Se o problema for do hardware, isto salva o Pico!
  // can_net.begin(); 

  Serial.print("System Initializing... Node ID: ");
  Serial.println(my_id);
  box.identify_static_gain(led, ldr);
  
  Serial.println("System Ready");
  last_time = millis();
}

void loop() {
  checkSerial();
  
  // Também comentei a leitura do CAN para não causar erros
  // checkCanBus();

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
      processCommand(inputBuffer, led, ldr, box, metrics, luminaire, pid, can_net);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
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
  metrics.update(u_k, y_k, luminaire.reference, 0.01f);

  if (luminaire.streaming) {
    Serial.print("s ");
    Serial.print(luminaire.stream_var);
    Serial.print(" ");
    Serial.print(luminaire.getId()); 
    Serial.print(" "); 

    float val = 0.0f; 

    if (luminaire.stream_var == 'b') {
        Serial.print(y_k, 2);
        Serial.print(" "); 
        Serial.print(u_k, 4);
        Serial.print(" "); 
        Serial.print(luminaire.reference, 2); 
    } else {
        val = (luminaire.stream_var == 'y') ? y_k : u_k;
        Serial.print(val, 4);
    }
    Serial.print(" ");
    Serial.println(micros());
  }
}