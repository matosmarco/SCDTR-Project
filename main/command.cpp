#include "command.h"

// Processes incoming serial commands to interact with the system
void processCommand(String cmd, LED& led, LDR& ldr, Box& box) {
    // Remove leading and trailing whitespace to prevent string matching errors
    cmd.trim();

    // CALIBRATION & SYSTEM IDENTIFICATION COMMANDS
    
    // Command: "calibb" 
    // Action: Calibrates the 'b' parameter of the LDR assuming a known 500 Lux reference
    if (cmd == "calibb") {
        ldr.calibrate_b(500);	
    } 
    // Command: "calibm" 
    // Action: Calibrates the 'm' (slope) parameter of the LDR by sweeping the LED duty cycle
    else if (cmd == "calibm") {
        ldr.calibrate_m(led);
    } 
    // Command: "bg" 
    // Action: Measures and stores the background illumination (with LED off)
    else if (cmd == "bg") {
        box.calibrate_background(led, ldr);
    } 
    // Command: "id" 
    // Action: Identifies the static gain (K) of the system (relationship between duty cycle and Lux)
    else if (cmd == "id") {
        box.identify_static_gain(led, ldr);
    } 

    // SYSTEM DYNAMICS (STEP RESPONSE)
    
    // Command: "step <u_init> <u_final>"
    // Action: Performs a step response from u_init to u_final to evaluate time constants (tau)
    // Usage example: "step 0 1" (step-up) or "step 1 0" (step-down)
    else if (cmd.startsWith("step ")) {
        float u_init, u_final;
        
        // Parse the initial and final duty cycle values from the command string
        if (sscanf(cmd.c_str(), "step %f %f", &u_init, &u_final) == 2) {
            box.do_step_response(led, ldr, u_init, u_final);
        } else {
            Serial.println("Error: Use format 'step <u_init> <u_final>'");
        }
    }

    // Command: "stairs"
    // Action: Performs a staircase sequence up and down
    else if (cmd == "stairs") {
        box.do_staircase_response(led, ldr);
    }
    // SENSOR READING & ACTUATOR COMMANDS
    
    // Command: "lux"
    // Action: Reads and prints the current illuminance in Lux
    else if (cmd == "lux") {
        Serial.print("Lux: ");
        Serial.println(ldr.readLux());
    } 
    // Command: "u <duty>"
    // Action: Manually sets the LED duty cycle (0.0 to 1.0)
    else if (cmd.startsWith("u ")) {
        float duty = cmd.substring(2).toFloat();
        led.setDuty(duty);
    } 

    // DEBUGGING & PARAMETER INFO COMMANDS
    
    // Command: "debug"
    // Action: Prints detailed raw sensor data (Voltage, Resistance, and Lux)
    else if (cmd == "debug") {
        float v = ldr.readVoltage();
        float r = ldr.R_LDR_compute(v);
        float lux = ldr.luxCompute(r);

        Serial.println("Debug Variables");
        Serial.print("Voltage (V): ");
        Serial.println(v, 4);
        Serial.print("Resistance LDR (Ohms): ");
        Serial.println(r, 2);
        Serial.print("Illuminance (Lux): ");
        Serial.println(lux, 2);
    } 
    // Command: "param"
    // Action: Prints the currently stored calibration parameters
    else if (cmd == "param") {
        Serial.println("Current Parameters");
        Serial.print("b value: ");
        Serial.println(ldr.get_b());
        Serial.print("m value: ");
        Serial.println(ldr.get_m());
        Serial.print("Gain (K): ");
        Serial.println(box.get_gain());
    } 
    
    // UNKNOWN COMMAND HANDLING
    else {
        Serial.println("Error: Unknown command");
	//TODO: write usage
    }
}
