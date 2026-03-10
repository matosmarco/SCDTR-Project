#include "led.h"
#include "ldr.h"
#include "box.h"
#include "command.h"

// Objects
LED led;
LDR ldr;
Box box;
Metrics metrics (0.014); // P_max = 0.014 W
Luminaire luminaire;
pid pid(0.01);

// Variáveis de tempo para o ciclo de 100Hz 
unsigned long last_time = 0;
const unsigned long sample_time = 10; // 10ms = 100Hz

void setup(){
  Serial.begin(115200);
  while(!Serial){
    delay(500);
  }
  analogReadResolution(12);

  led.initPWM();
  Serial.println("System Ready");
  Serial.println("Commands: calibb | bg | id | lux | u <0-1> | debug");
  box.identify_static_gain(led, ldr);
}

void loop(){

    if(Serial.available()){

        String cmd = Serial.readStringUntil('\n');
        processCommand(cmd, led, ldr, box, metrics, luminaire, pid);
    }
}
