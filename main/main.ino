#include "led.h"
#include "ldr.h"
#include "box.h"
#include "command.h"
#include "metrics.h"
#include "pid.h"

// --- Objetos ---
LED led;
LDR ldr;
Box box;
Metrics metrics(0.014); 
Luminaire luminaire;
pid pid(0.01);         

// --- Variáveis de Tempo e Buffer ---
unsigned long last_time = 0;
const unsigned long sample_time = 10; // 10ms = 100Hz [cite: 251, 583]
String inputBuffer = "";              // Buffer para comandos não-bloqueantes

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  
  analogReadResolution(12); 
  led.initPWM();            

  Serial.println("System Initializing...");
  box.identify_static_gain(led, ldr);
  
  Serial.println("System Ready");
  last_time = millis();
}

void loop() {
  // 1. Processamento de Comandos Serial (Não-Bloqueante) 
  checkSerial();

  // 2. Temporização do Ciclo de Controlo (Síncrono a 100Hz) [cite: 517, 527]
  unsigned long current_time = millis();
  if (current_time - last_time >= sample_time) {
    last_time += sample_time; // Mantém a fase do relógio estável
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
 * Lógica em tempo real executada a cada 10ms[cite: 251, 295].
 */
void runControlLoop() {
  float y_k = ldr.readLux();

  float u_k;
  if (luminaire.feedback_on) {
    u_k = pid.compute_control(luminaire.reference, y_k); 
  } else {
    u_k = luminaire.current_u;
  }

  led.setDuty(u_k);

  // Atualização de métricas e PID [cite: 295, 525]
  pid.housekeep(luminaire.reference, y_k, u_k);
  metrics.update(u_k, y_k, luminaire.reference, 0.01f);

  // Streaming em Tempo Real [cite: 501]
  if (luminaire.streaming) {
    float val = (luminaire.stream_var == 'y') ? y_k : u_k;
    Serial.print("s ");
    Serial.print(luminaire.stream_var);
    Serial.print(" ");
    Serial.print(luminaire.getId());
    Serial.print(" ");
    Serial.print(val, 2);
    Serial.print(" ");
    Serial.println(millis()); // Timestamp em ms [cite: 501]
  }
}