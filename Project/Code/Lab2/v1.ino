// Raspberry Pi Pico Pins (on Pinout document)
const int LED_PIN = 13; // GP13 (physical pin: PIN17)
const int LDR_PIN = 26; // GP26 (also ADC0 and physical pin: PIN31 )

// Electrical variables
const float vcc = 3.3;
const float r_read_circuit = 10e3;
const int ADC_MAX = 4096;

// LDR  model (calibration variables)
float gamma_LDR = -0.8; //TODO: CHECK CALIBRATION OF THIS VALUE (datasheet pages 6-7)
float b = 0;
float rLDR = 0.0;
float target_lux = 500.0;
float vout = 0;
float lux_value = 0.0;

// System states
float backgroundLux = 0.0;
float gain = 0.0;
float filtered_adc = 0.0;

/* Auxiliary functions*/

// ADC Reading (with averaging)
float readVoltage(int samples = 100){
  int adc = 0;
  for(int i= 0; i < samples; i++){
    adc += analogRead(LDR_PIN);
  }
  float avg_adc = float(adc)/samples;
  return (avg_adc * vcc) / (ADC_MAX-1); // Level starts at 0
}

// R_LDR value (voltage to resistance)
float r_LDR_compute(float v){
  if(v <= 0.001){
    return 1e6;   // avoid division by zero
  }
  return (r_read_circuit * v)/ (vcc-v);
}

// Computation of the Lux values from R_LDR (resistante to Lux)
float lux_compute(float r_ldr){
  float logLux = (log10(r_ldr) - b) / gamma_LDR;
  return pow(10, logLux);
}

// Full reading of the illuminance
float readLux(){
  float v = readVoltage();
  float r_ldr = r_LDR_compute(v);
  return lux_compute(r_ldr);
}

// LED PWM
void initPWM(int freq= 2000, int range = 255){
  analogWriteFreq(freq);
  analogWriteRange(range);
  pinMode(LED_PIN, OUTPUT);
}

void setDuty(float duty, int range = 255){
  duty = constrain(duty, 0.0, 1.0); // just to guarantee that the program is robust, just values between 0 and 1 are used
  analogWrite(LED_PIN, duty*range);
}

/*Calibration functions*/
void calibrate_b(float knownLux= 500.0){
  float v = readVoltage();
  float r_ldr = r_LDR_compute(v);
  b = log10(r_ldr) - gamma_LDR * log10(knownLux);
  Serial.print("Calibrated b: ");
  Serial.println(b);
}

void calibrate_m(){
  // Set Duty cycle of 50 %
  setDuty(0.5);
  delay(2000); // some time to stabilize the LDR
  // Measure for a duty cycle of 50%
  float v_1 = readVoltage();
  float r_1 = r_LDR_compute(v_1);
  
  // Set Duty cycle of 100%
  setDuty(1.0);
  delay(2000); // some time to stabilize the LDR
  float v_2 = readVoltage();
  float r_2 = r_LDR_compute(v_2);

  // Compute new gamma_LDR (m)
  gamma_LDR = (log10(r_1) - log10(r_2)) / (log10(0.5) - log10(1.0));
  Serial.print("New computed gamma_LDR (m): ");
  Serial.println(gamma_LDR);
  setDuty(0);
}

void calibrate_background(){
  setDuty(0);
  delay(2000);

  backgroundLux = readLux();
  Serial.print("Background Lux: ");
  Serial.println(backgroundLux);
}

// Static Gain Identification
void identify_static_gain(){
  Serial.println("duty, lux");
  float L0 = 0;
  float L1 = 0;

  for(float d = 0.0; d <= 1.0; d += 0.1) {

    setDuty(d);
    delay(2000);

    float lux = readLux();

    Serial.print(d);
    Serial.print(",");
    Serial.println(lux);

    if(d == 0.0) L0 = lux;
    if(d >= 0.99) L1 = lux;
  }

  gain = L1 - L0;

  Serial.print("Estimated Static Gain K: ");
  Serial.println(gain);

  setDuty(0);

}

// Command processing
void processCommand(String cmd) {

  cmd.trim();

  if(cmd == "calibb") {
    calibrate_b(500.0);

  } else if (cmd == "calibm"){
    calibrate_m();

  } else if(cmd == "bg") {
    calibrate_background();

  } else if(cmd == "id") {
    identify_static_gain();

  } else if(cmd == "lux") {
    Serial.print("Lux: ");
    Serial.println(readLux());

  } else if(cmd.startsWith("u ")) {
    float duty = cmd.substring(2).toFloat();
    setDuty(duty);

  } else if(cmd == "debug") {
    // Voltage reading
    float v = readVoltage();
    
    // R_LDR value
    float r_ldr = r_LDR_compute(v);
    
    // Lux value
    float lux = lux_compute(r_ldr);
    
    Serial.println("Debug variables");
    Serial.print("Voltage (V): ");
    Serial.println(v, 4);    
    Serial.print("Resistance LDR (Ohms): ");
    Serial.println(r_ldr, 2);
    
    Serial.print("Illuminance (LUX): ");
    Serial.println(lux, 2);
  }

  else {
    Serial.println("Unknown command");
  }
}
void setup() {// the setup function runs once
  Serial.begin(115200);
  while(!Serial){
    delay(500);
  }
  analogReadResolution(12); //default is 10, 12, becaues 2^12 = 4096

  Serial.println("System Ready");
  Serial.println("Commands: calibb | calibm | bg | id | lux | u <0-1> | debug");
  initPWM();
}

void loop() {// the loop function runs cyclically
   if(Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}

