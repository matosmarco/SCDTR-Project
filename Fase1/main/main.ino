#include "led.h"
#include "ldr.h"
#include "box.h"
#include "command.h"
#include "metrics.h"
#include "pid.h"
#include "mcp2515.h"

// Objects 
LED led;
LDR ldr;
Box box;
Metrics metrics(0.01963); 
Luminaire luminaire;
pid pid(0.01);         
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};

// Global variables for CAN-BUS communication
pico_unique_board_id_t pico_board_id; // full id
uint8_t node_address; // short id
struct can_frame canMsgTx, canMsgRx;
unsigned long counter{0};
MCP2515::ERROR err;
uint8_t unsigned long time_to_write;
unsigned long write_delay {1000};
const int BUFSZ = 100;
char printbuf[BUFSZ];

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

  //luminaire.setId(0);
  //luminaire.setId(1);

  Serial.println("System Initializing...");
  box.identify_static_gain(led, ldr);
  
  Serial.println("System Ready");
  last_time = millis();
}

void setup1() {
  pico_get_unique_board_id(&pico_board_id);
  //node_address = pico_board_id.id[7]; //check
  node_address = 0;
  for (int i = 0; i < 8; i++) {
      node_address += pico_board_id.id[i];
  }
  luminaire.setId(node_address); // CHECK IF IT WORKS
  Serial.begin();
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode(); //setLoopbackMode()debug
  unsigned long current_time = millis();
  time_to_write = current_time + write_delay;
}



void loop() {
  // Serial processing 
  //checkSerial();

  // Ensure sincronization to 100 Hz to the control loop
  unsigned long current_time = millis();
  if (current_time - last_time >= sample_time) {
    last_time += sample_time; 
    runControlLoop(); 
  }
}

/*
void loop1() {
  unsigned long current_time = millis();
  if(current_time >= time_to_write ) {
    canMsgTx.can_id = node_address;
    canMsgTx.can_dlc = 8;
  unsigned long div = counter;
  for(int i = 0; i < canMsgTx.can_dlc; i++ ) {
    canMsgTx.data[i] = '0'+(int)(div%10);
    div = div/10;
  }
  err = can0.sendMessage(&canMsgTx);
  snprintf(printbuf, BUFSZ,
  "Sending message %ld from node %x\n",
  counter++, node_address);
  Serial.print(printbuf);
  time_to_write = current_time+write_delay;
  }
  while ((err = can0.readMessage(&canMsgRx)) == MCP2515::ERROR_OK ) {
    unsigned long rx_msg = 0;
    unsigned long mult = 1;
    for (int i=0 ; i < canMsgRx.can_dlc ; i++) {
      rx_msg += mult*(canMsgRx.data[i]-'0');
      mult = mult*10;
    }
  snprintf(printbuf, BUFSZ,"\t\t\t\tReceived message %ld from node %x\n", rx_msg, (int)canMsgRx.can_id);
  Serial.print(printbuf);
  }
  if(err == MCP2515::ERROR_FAIL)
    Serial.println("Error");
}
// CODIGO DO PROFESSOR
*/


void loop1(){
  
  send_message();
  read_message();
}


void checkSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(inputBuffer, led, ldr, box, metrics, luminaire, pid);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
}

// Check received messages
void send_message(){
    if (Serial.available() > 0) {
    String text_to_send = Serial.readStringUntil('\n');
    text_to_send.trim(); 
    
    // Checks if the message size is not null
    if (text_to_send.length() > 0) {
    
      canMsgTx.can_id = node_address;
      
      // Limit the size of the message to 8 bytes (CAN Limit)
      int messageSize = text_to_send.length();
      if (messageSize > 8) {
        messageSize = 8; // Limits to the first 8 
      }
      canMsgTx.can_dlc = messageSize;
      
      // Copy message to CAN buffer-variable
      for (int i = 0; i < messageSize; i++) {
        canMsgTx.data[i] = text_to_send[i];
      }
      
      // Sends the message
      err = can0.sendMessage(&canMsgTx);
      
      // Print of the received message
      char sent_text[9] = {0};
      for(int i = 0; i < messageSize; i++) {
        sent_text[i] = canMsgTx.data[i];
      }
      
      // Text sent 
      snprintf(printbuf, BUFSZ, "Sending message '%s' from node %x\n", sent_text, node_address);
      Serial.print(printbuf);
    }

  }
}

void read_message(){
  while((err = can0.readMessage(&canMsgRx)) == MCP2515::ERROR_OK){
    char receivedText[9] = {0}; // 8 bytes + terminator (\0)
    for(int i = 0; i<canMsgRx.can_dlc; i++){
      receivedText[i] =(char)canMsgRx.data[i];
    }
    // Print of the received message
    snprintf(printbuf, BUFSZ,"\t\t\t\tReceived message %s from node %x\n", receivedText, (int)canMsgRx.can_id);
    Serial.print(printbuf);
  }
  if(err == MCP2515::ERROR_FAIL){
    Serial.print("Error reading CAN message from node: ");
    Serial.println((int)canMsgRx.can_id);
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

    Serial.print(" ");
    Serial.println(millis());
    //Serial.println(micros());
  }
  
  }
