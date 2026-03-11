#include "command.h"

// Processes incoming serial commands to interact with the system
void processCommand(String cmd, LED& led, LDR& ldr, Box& box, Metrics& metrics, Luminaire& luminaire, pid& pid) {
    // Remove leading and trailing whitespace to prevent string matching errors
    cmd.trim();
    
    // Auxiliary variables to protocol commands
    int target_id;
    float val;
    char char_val;
    char type;

    // RESTART, CALIBRATION & SYSTEM IDENTIFICATION COMMANDS
    
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

    // Command: "R" -> Reset all and recalibrate
    else if (cmd == "R") {
        metrics.reset();
        box.calibrate_background(led, ldr);
        box.identify_static_gain(led, ldr);
        Serial.println("ack"); 
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

    // MANDATORY COMMANDS
    // PERFORMANCE METRICS COMMANDS (Table 2)
    
    // Get average energy consumption 
    else if (cmd.startsWith("g E")) {
        Serial.print("E 1 "); 
        Serial.println(metrics.energy, 4);
    }
    // Get average visibility error 
    else if (cmd.startsWith("g V")) {
        Serial.print("V 1 "); 
        Serial.println(metrics.getAverageVisibility(), 4);
    }
    // Get average flicker error 
    else if (cmd.startsWith("g F")) {
        Serial.print("F 1 "); 
        Serial.println(metrics.flicker, 4);
    }

// SENSOR & ACTUATOR BASIC COMMANDS (Table 1)
    // Comand 1
    else if (sscanf(cmd.c_str(), "u %d %f", &target_id, &val) == 2) {
        if (target_id == luminaire.getId()) {
            if (val >= 0.0f && val <= 1.0f) {
                luminaire.current_u = val;
                led.setDuty(val);
                Serial.println("ack");
            } else {
                Serial.println("err"); // Valor fora dos limites
            }
        } else {
            Serial.println("err"); // Roteamento futuro para rede CAN
        }
    }
    // "g u <i>" -> Get current duty cycle 
    // Command 2
    else if (sscanf(cmd.c_str(), "g u %d", &target_id) == 1) {
    if (target_id == luminaire.getId()) {
        // Response format: "u <i> <val>" 
        Serial.print("u "); 
        Serial.print(target_id); 
        Serial.print(" "); 
        Serial.println(luminaire.current_u, 4); // Show 4 decimal places
    } else {
        Serial.println("err");
    }
}

// Command: "r <i> <val>" -> Set the illuminance reference 
// Command 3
else if (sscanf(cmd.c_str(), "r %d %f", &target_id, &val) == 2) {
    // Check if the command is intended for this specific node 
    if (target_id == luminaire.getId()) {
        // Validation: Reference must be non-negative 
        if (val >= 0.0f) {
            luminaire.reference = val; // Update the reference in the Luminaire object
            Serial.println("ack");      // Protocol success response 
        } else {
            Serial.println("err");      // Invalid value 
        }
    } else {
        // In Phase 2, this would be routed via CAN-BUS to the correct node
        Serial.println("err");          // ID mismatch for this node 
    }
}

// Command: "g r <i>" -> Get current illuminance reference
// Command 4
else if (sscanf(cmd.c_str(), "g r %d", &target_id) == 1) {
    // Verification of the luminaire ID
    if (target_id == luminaire.getId()) {
        // Resposta no formato: "r <i> <val>"
        Serial.print("r "); 
        Serial.print(target_id); 
        Serial.print(" "); 
        Serial.println(luminaire.reference, 2); // Returns reference in lux
    } else {
        // Na Fase 1, se o ID não for o nosso, devolvemos erro
        Serial.println("err"); 
    }
}

// Command: "g y <i>" -> Measure the actual illuminance (Lux)
// Command 5
else if (sscanf(cmd.c_str(), "g y %d", &target_id) == 1 ) {
    // Check if the command is intended for this specific luminaire
    if (target_id == luminaire.getId()) {
        // Response format: "y <i> <val>" 
        Serial.print("y "); 
        Serial.print(target_id); 
        Serial.print(" "); 
        // Read the actual illuminance from the LDR sensor 
        Serial.println(ldr.readLux(), 2); 
    } else {
        // ID mismatch for this node
        Serial.println("err"); 
    }
}

// Command: "g v <i>" -> Measure the voltage level at the LDR 
// Command 6
else if (sscanf(cmd.c_str(), "g v %d", &target_id) == 1) {
    // Verifies if the luminaire ID is the correct ID
    if (target_id == luminaire.getId()) {
        // Response in the format: "v <i> <val>" 
        Serial.print("v "); 
        Serial.print(target_id); 
        Serial.print(" "); 
        
        // Voltage reading
        Serial.println(ldr.readVoltage(), 4); 
    } else {
        // Erro se o ID não for o deste nó (relevante para a Fase 2) 
        Serial.println("err"); 
    }
}
// Command: "o <i> <val>" -> Set current occupancy state
// Command 7
else if (sscanf(cmd.c_str(), "o %d %c", &target_id, &char_val) == 2) {
    // 1. Validate the Luminaire ID
    if (target_id == luminaire.getId()) {
        bool valid_state = true;

        // 2. Map character input to the LuminaireState enum 
        if (char_val == 'o') {
            luminaire.state = LuminaireState::OFF;
        } 
        else if (char_val == 'l') {
            luminaire.state = LuminaireState::LOW;
        } 
        else if (char_val == 'h') {
            luminaire.state = LuminaireState::HIGH;
        } 
        else {
            valid_state = false;
        }

        if (valid_state) {
            // 3. Update the reference inside the Luminaire object 
            // This sets reference to 0.0, low_bound, or high_bound 
            luminaire.updateReference();

            // 4. Synchronize the PID reference variable
            // As you decided, the PID also holds a copy of the reference
            //pid.r = luminaire.reference;

            Serial.println("ack"); // Success response [cite: 657]
        } else {
            Serial.println("err"); // Invalid character value
        }
    } else {
        Serial.println("err"); // ID mismatch
    }
}

// Command: "g o <i>" -> Get current occupancy state
// Command 8
else if (sscanf(cmd.c_str(), "g o %d", &target_id) == 1) {
    if (target_id == luminaire.getId()) {
        char s = 'o';
        if (luminaire.state == LuminaireState::LOW) s = 'l';
        else if (luminaire.state == LuminaireState::HIGH) s = 'h';
        
        // Response format: "o <i> <val>" [cite: 657]
        Serial.print("o "); Serial.print(target_id); Serial.print(" "); Serial.println(s);
    } else {
        Serial.println("err");
    }
}

// Command: "a <i> <val>" -> Set Anti-windup state
// Command 9
else if (sscanf(cmd.c_str(), "a %d %f", &target_id, &val) == 2) {
    if (target_id == luminaire.getId()) {
        if (val == 0.0f || val == 1.0f) {
            pid.anti_windup = (val > 0.5f);
            Serial.println("ack");
        } else {
            Serial.println("err");
        }
    } else {
        Serial.println("err");
    }
}

// Command: "g a <i>" -> Get Anti-windup state
// Command 10
else if (sscanf(cmd.c_str(), "g a %d", &target_id) == 1) {
    if (target_id == luminaire.getId()) {
        Serial.print("a "); 
        Serial.print(target_id); 
        Serial.print(" "); 
        Serial.println(pid.anti_windup ? 1 : 0); // converts boolean to 0 or 1
    } else {
        Serial.println("err");
    }
}

// Command: "f <i> <val>" -> Set feedback control on/off
// Command 11
else if (sscanf(cmd.c_str(), "f %d %f", &target_id, &val) == 2) {
    // 1. Verifica se o ID é o desta luminária
    if (target_id == luminaire.getId()) {
        // 2. Valida se o valor é 0 ou 1
        if (val == 0.0f || val == 1.0f) {
            luminaire.feedback_on = (val > 0.5f); // Assume que adicionaste esta flag na classe Luminaire
            Serial.println("ack"); 
        } else {
            Serial.println("err"); 
        }
    } else {
        Serial.println("err"); 
    }
}

// Command: "g f <i>" -> Get feedback control state
// Command 12
else if (sscanf(cmd.c_str(), "g f %d", &target_id) == 1) {
    if (target_id == luminaire.getId()) {
        // Resposta no formato: "f <i> <val>"
        Serial.print("f "); 
        Serial.print(target_id); 
        Serial.print(" "); 
        Serial.println(luminaire.feedback_on ? 1 : 0);
    } else {
        Serial.println("err");
    }
}

// Command: "g d <i>" -> Get current external illuminance (background)
// Command 13
else if (sscanf(cmd.c_str(), "g d %d", &target_id) == 1) {
    // 1. Verifica se o ID solicitado é o desta luminária
    if (target_id == luminaire.getId()) {
        // 2. Resposta no formato: "d <i> <val>"
        Serial.print("d "); 
        Serial.print(target_id); 
        Serial.print(" "); 
        // 3. Obtém o valor guardado no objeto box
        Serial.println(box.get_background(ldr), 2); 
    } else {
        // ID não corresponde a este nó
        Serial.println("err"); 
    }
}

// Command: "g p %d" -> Get instantaneous power consumption
// Command 14
else if (sscanf(cmd.c_str(), "g p %d", &target_id) == 1) {
    if (target_id == luminaire.getId()) {
        // Obtém a potência diretamente da classe Metrics
        float instant_power = metrics.getInstantaneousPower(luminaire.current_u);

        Serial.print("p "); 
        Serial.print(target_id); 
        Serial.print(" "); 
        Serial.println(instant_power, 4); 
    } else {
        Serial.println("err"); 
    }
}

// Command: "g t <i>" -> Get elapsed time since last restart
else if (sscanf(cmd.c_str(), "g t %d", &target_id) == 1) {
    // 1. Verifica se o ID solicitado corresponde a esta luminária
    if (target_id == luminaire.getId()) {
        // 2. Calcula o tempo decorrido em segundos
        float seconds = millis() / 1000.0f;

        // 3. Resposta no formato: "t <i> <val>"
        Serial.print("t "); 
        Serial.print(target_id); 
        Serial.print(" "); 
        Serial.println(seconds, 3); // Três casas decimais para precisão de milissegundos
    } else {
        Serial.println("err"); 
    }
}

// Command: "s <x> <i>" -> Start real-time stream
// <x> can be 'y' (lux) or 'u' (duty cycle)
// Command 16
else if (sscanf(cmd.c_str(), "s %c %d", &type, &target_id) == 2) {
    if (target_id == luminaire.getId()) {
        if (type == 'y' || type == 'u') {
            luminaire.stream_var = type;
            luminaire.streaming = true;
        } else {
            Serial.println("err");
        }
    } else {
        Serial.println("err");
    }
}

// Command: "S <x> <i>" -> Stop real-time stream
// Command 17
else if (sscanf(cmd.c_str(), "S %c %d", &type, &target_id) == 2) {
    if (target_id == luminaire.getId()) {
        if (type == 'y' || type == 'u') {
            luminaire.streaming = false; // Para o envio no loop 
            Serial.println("ack");
        } else Serial.println("err");
    } else Serial.println("err");
}

// Command: "g b <x> <i>" -> Get last minute buffer
// Command 18
else if (sscanf(cmd.c_str(), "g b %c %d", &type, &target_id) == 2) {
    if (target_id == luminaire.getId() && (type == 'y' || type == 'u')) {
        // Resposta formatada: b <x> <i> <val1>, <val2>, ... 
        Serial.print("b "); Serial.print(type); Serial.print(" ");
        Serial.print(target_id); Serial.print(" ");
        
        for (int i = 0; i < BUFFER_SIZE; i++) {
            Serial.print(metrics.getBufferValue(type, i), 2);
            if (i < BUFFER_SIZE - 1) Serial.print(",");
        }
        Serial.println(); 
    } else Serial.println("err");
}
    // UNKNOWN COMMAND HANDLING
    else {
        Serial.println("Error: Unknown command");
	//TODO: write usage
    }
}
