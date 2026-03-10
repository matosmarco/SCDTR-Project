#include "box.h"
Box::Box(){
	backgroundLux = 0.0;
	gain = 0.0;
}

void Box::calibrate_background(LED& led, LDR& ldr){
  led.off();
  delay(2000);

  // Clear readings due the delay 
  for(int i = 0; i < MEDIAN_BUFFER_SIZE; i++) ldr.readVoltage();
  
  backgroundLux = ldr.readLux();
  Serial.print("Background Lux: ");
  Serial.println(backgroundLux);
}


void Box::identify_static_gain(LED& led, LDR& ldr){
  led.setDuty(0.0);
  delay(2000);
  
  // FLUSH BUFFER: Deletes previous light readings
  for(int i = 0; i < MEDIAN_BUFFER_SIZE; i++) ldr.readVoltage();

  float l0 = ldr.readLux();

  Serial.print("L0: ");
  Serial.println(l0);

  

  led.setDuty(1.0);
  delay(2000);
  // FLUSH BUFFER: Deletes previous light readings
  for(int i = 0; i < MEDIAN_BUFFER_SIZE; i++) ldr.readVoltage();


  float l1 = ldr.readLux();
  
  Serial.print("L1: ");
  Serial.println(l1);

  gain = (l1 - l0);

  Serial.print("Estimated Static Gain K: ");
  Serial.println(gain);

  led.off();
}


// Performs a step response experiment to identify system dynamics (time constants and non-linearities)
void Box::do_step_response(LED& led, LDR& ldr, float u_init, float u_final) {
    Serial.println("Time(s),Real_Lux");

    // Set the initial LED state and wait for the system to fully stabilize
    led.setDuty(u_init);
    delay(2000); 

    // Flush the LDR median filter buffer to clear old readings
    for(int i = 0; i < MEDIAN_BUFFER_SIZE; i++) {
        ldr.readVoltage();
    }

    // Apply the step input to the target duty cycle
    unsigned long startTime = millis();
    led.setDuty(u_final);

    // Sample and stream the data at 100Hz for 2 seconds
    while(millis() - startTime < 2000) {
        unsigned long current_time = millis() - startTime;
        
        // Read the actual illuminance
        float real_lux = ldr.readLux();

        // Print data in CSV format: Time(seconds), Lux
        Serial.print(current_time / 1000.0, 3); 
        Serial.print(",");
        Serial.print(real_lux, 2);
        Serial.println(";");            

        // 10ms delay to enforce the 100Hz sampling rate required for the project
        delay(10); 
    }
    
    // Note: We leave the LED at u_final state after the test completes
}

void Box::do_staircase_response(LED& led, LDR& ldr) {
    Serial.println("Time(s),ADC_Voltage,Real_Lux");

    // Start on dark environment
    led.setDuty(0.0);
    delay(2000); 
    for(int i = 0; i < 11; i++) ldr.readVoltage(); // Limpar buffer

    unsigned long startTime = millis();

    // Higher values of PWM
    for (float u = 0.1; u <= 1.01; u += 0.1) {
        led.setDuty(u);
        unsigned long stepStart = millis();
        
        // Save data for 1 second
        while(millis() - stepStart < 1000) {
            float v = ldr.readVoltage();
            float lux = ldr.readLux();
            
            Serial.print((millis() - startTime) / 1000.0, 3);
            Serial.print(",");
            Serial.print(v, 4);
            Serial.print(",");
            Serial.print(lux, 2);
            Serial.println(";");
            delay(10); // 100Hz
        }
    }

    // Lower the values
    for (float u = 0.9; u >= -0.01; u -= 0.1) {
        led.setDuty(u);
        unsigned long stepStart = millis();
        
        while(millis() - stepStart < 1000) {
            float v = ldr.readVoltage();
            float lux = ldr.readLux();
            
            Serial.print((millis() - startTime) / 1000.0, 3);
            Serial.print(",");
            Serial.print(v, 4);
            Serial.print(",");
            Serial.print(lux, 2);
            Serial.println(";");
            delay(10); // 100Hz
        }
    }
    led.off();
    Serial.println("Experiment finished.");
}



float Box::get_gain() {
    return gain;
}

float Box::get_background(LDR& ldr) {
    backgroundLux = ldr.readLux();
    return backgroundLux;
}
