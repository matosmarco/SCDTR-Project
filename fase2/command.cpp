#include "command.h"
#include <FreeRTOS.h>
#include <queue.h>
#include "config.h"

extern long max_jitter;
extern QueueHandle_t canTxQueue;     // Access to the TX queue managed in main.ino
extern float global_energy_cost;     // Access to the global energy cost variable
extern bool net_streaming;

// Processes incoming serial commands to interact with the system
void processCommand(String cmd, LED& led, LDR& ldr, Box& box, Metrics& metrics, Luminaire& luminaire, pid& pid) {
    // Remove leading and trailing whitespace
    cmd.trim();
    if (cmd.length() == 0) return;

    // CAN Network router (Intercepts commands from the tables)
    char c0 = cmd[0];
    char subcmd = ' ';
    char subsubcmd = ' ';
    int dest_id = luminaire.getId();
    float val_f = 0.0f;
    char val_c = ' ';
    bool is_network = false;
    ProtocolMsg_t msg;

    // Restart (Broadcast) - 'R'
    if (c0 == 'R' && cmd.length() == 1) {
        msg = {(uint8_t)luminaire.getId(), 255, 'R', ' ', 0.0f};
        xQueueSend(canTxQueue, &msg, 0);
        // No return here! We let it fall through to restart locally as well.
    }
    // Get Buffer ('g b <x> <i>')
    else if (c0 == 'g' && cmd.length() > 2 && cmd[2] == 'b') {
        if (sscanf(cmd.c_str(), "%c %c %c %d", &c0, &subcmd, &subsubcmd, &dest_id) >= 4) {
            if (dest_id != luminaire.getId()) {
                msg = {(uint8_t)luminaire.getId(), (uint8_t)dest_id, 'b', subsubcmd, 0.0f};
                is_network = true;
            }
        }
    }
    // Gets and Streams ('g', 's', 'S')
    else if (c0 == 'g' || c0 == 's' || c0 == 'S') {
        if (sscanf(cmd.c_str(), "%c %c %d", &c0, &subcmd, &dest_id) >= 3) {
            if (dest_id != luminaire.getId()) {
                msg = {(uint8_t)luminaire.getId(), (uint8_t)dest_id, c0, subcmd, 0.0f};
                is_network = true;
            }
        }
    }
    // Set occupancy ('o')
    else if (c0 == 'o') {
        if (sscanf(cmd.c_str(), "%c %d %c", &c0, &dest_id, &val_c) >= 3) {
            if (dest_id != luminaire.getId()) {
                msg = {(uint8_t)luminaire.getId(), (uint8_t)dest_id, c0, ' ', (float)val_c};
                is_network = true;
            }
        }
    }
    // 1.5 OTHER SETs (Tables 1 and 3: 'u', 'r', 'a', 'f', 'U', 'O', 'C')
    else if (c0 == 'u' || c0 == 'r' || c0 == 'a' || c0 == 'f' || c0 == 'U' || c0 == 'O' || c0 == 'C') {
        if (sscanf(cmd.c_str(), "%c %d %f", &c0, &dest_id, &val_f) >= 3) {
            if (dest_id != luminaire.getId()) {
                msg = {(uint8_t)luminaire.getId(), (uint8_t)dest_id, c0, ' ', val_f};
                is_network = true;
            }
        }
    }

    // If the message is for another node on the network, enqueue it and stop here.
    if (is_network) {
        xQueueSend(canTxQueue, &msg, 0);
        return; 
    }

    // =========================================================
    // 2. LOCAL EXECUTION (The command is for us or is an "Extra")
    // =========================================================
    int target_id;
    float val;
    char char_val;
    char type;

    // --- RESTART, CALIBRATION & SYSTEM IDENTIFICATION COMMANDS ---
    if (cmd == "calibb") {
        ldr.calibrate_b(500);	
    } 
    else if (cmd == "calibm") {
        ldr.calibrate_m(led);
    } 
    else if (cmd == "bg") {
        box.calibrate_background(led, ldr);
    } 
    else if (cmd == "id") {
        box.identify_static_gain(led, ldr);
    }
    else if (cmd == "R") {
        metrics.reset();
        box.calibrate_background(led, ldr);
        box.identify_static_gain(led, ldr);
        Serial.println("ack"); 
    }

    // --- SYSTEM DYNAMICS (STEP RESPONSE) ---
    else if (cmd.startsWith("step ")) {
        float u_init, u_final;
        if (sscanf(cmd.c_str(), "step %f %f", &u_init, &u_final) == 2) {
            box.do_step_response(led, ldr, u_init, u_final);
        } else {
            Serial.println("Error: Use format 'step <u_init> <u_final>'");
        }
    }
    else if (cmd == "stairs") {
        box.do_staircase_response(led, ldr);
    }
    
    // Sensor reading and debugging extras
    else if (cmd == "lux") {
        Serial.print("Lux: "); Serial.println(ldr.readLux());
    } 
    else if (cmd == "debug") {
        float v = ldr.readVoltage();
        float r = ldr.R_LDR_compute(v);
        float lux = ldr.luxCompute(r);

        Serial.println("Debug Variables");
        Serial.printf("Voltage (V): %.4f\n", v);
        Serial.printf("Resistance LDR (Ohms): %.2f\n", r);
        Serial.printf("Illuminance (Lux): %.2f\n", lux);
        Serial.printf("Max Jitter (us): %ld\n", max_jitter);
    } 
    else if (cmd == "param") {
        Serial.println("Current Parameters");
        Serial.printf("b value: %.4f\n", ldr.get_b());
        Serial.printf("m value: %.4f\n", ldr.get_m());
        Serial.printf("Gain (K): %.4f\n", box.get_gain());
    } 
    else if (cmd == "j r") {
        max_jitter = 0;
        Serial.println("ack");
    }

    // Control extras (Percentages and Raw)
    else if (sscanf(cmd.c_str(), "p %d %f", &target_id, &val) == 2) {
        if (target_id == luminaire.getId()) {
            if (val >= 0.0f && val <= 100.0f) {
                luminaire.current_u = val/100.0;
                led.setPercentage(val);
                Serial.println("ack");
            } else Serial.println("err");
        } else Serial.println("err"); 
    }
    else if (sscanf(cmd.c_str(), "w %d %f", &target_id, &val) == 2) {
        if (target_id == luminaire.getId()) {
            if (val >= 0.0f && val <= 4095.0f) {
                luminaire.current_u = val/(ADC_MAX-1);
                led.setRaw(val);
                Serial.println("ack");
            } else Serial.println("err");
        } else Serial.println("err"); 
    }
    else if (sscanf(cmd.c_str(), "g p %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.printf("p %d %.2f\n", target_id, luminaire.current_u * 100.0f);
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "g w %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.printf("w %d %d\n", target_id, (int)(luminaire.current_u * 4095));
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "ff %d %f", &target_id, &val) == 2) {
        if (target_id == luminaire.getId()) {
            luminaire.feedforward_on = (val > 0.5f);
            Serial.println("ack");
        } else Serial.println("err");
    }

    // Mandatory commands (Tables 1, 2, and 3)
    
    // Table 2 - Metrics (Fixed with dynamic target_id)
    else if (sscanf(cmd.c_str(), "g E %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.print("E "); Serial.print(target_id); Serial.print(" "); Serial.println(metrics.energy, 4);
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "g V %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.print("V "); Serial.print(target_id); Serial.print(" "); Serial.println(metrics.getAverageVisibility(), 4);
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "g F %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.print("F "); Serial.print(target_id); Serial.print(" "); Serial.println(metrics.flicker, 4);
        } else Serial.println("err");
    }

    // Table 1 - Basic Commands and Streams
    else if (sscanf(cmd.c_str(), "u %d %f", &target_id, &val) == 2) {
        if (target_id == luminaire.getId()) {
            if (val >= 0.0f && val <= 1.0f) {
                luminaire.current_u = val;
                led.setDuty(val);
                Serial.println("ack");
            } else Serial.println("err"); 
        } else Serial.println("err"); 
    }
    else if (sscanf(cmd.c_str(), "g u %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.print("u "); Serial.print(target_id); Serial.print(" "); Serial.println(luminaire.current_u, 4); 
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "r %d %f", &target_id, &val) == 2) {
        if (target_id == luminaire.getId()) {
            if (val >= 0.0f) {
                luminaire.reference = val; 
                Serial.println("ack");      
            } else Serial.println("err");      
        } else Serial.println("err");          
    }
    else if (sscanf(cmd.c_str(), "g r %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.print("r "); Serial.print(target_id); Serial.print(" "); Serial.println(luminaire.reference, 2); 
        } else Serial.println("err"); 
    }
    else if (sscanf(cmd.c_str(), "g y %d", &target_id) == 1 ) {
        if (target_id == luminaire.getId()) {
            Serial.print("y "); Serial.print(target_id); Serial.print(" "); Serial.println(ldr.readLux(), 2); 
        } else Serial.println("err"); 
    }
    else if (sscanf(cmd.c_str(), "g v %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.print("v "); Serial.print(target_id); Serial.print(" "); Serial.println(ldr.readVoltage(), 4); 
        } else Serial.println("err"); 
    }
    else if (sscanf(cmd.c_str(), "o %d %c", &target_id, &char_val) == 2) {
        if (target_id == luminaire.getId()) {
            bool valid_state = true;
            if (char_val == 'o') { luminaire.state = LuminaireState::OFF; luminaire.reference = 0; } 
            else if (char_val == 'l') { luminaire.state = LuminaireState::LOW; luminaire.reference = 15; } 
            else if (char_val == 'h') { luminaire.state = LuminaireState::HIGH; luminaire.reference = 30; } 
            else { valid_state = false; }

            if (valid_state) Serial.println("ack"); 
            else Serial.println("err"); 
        } else Serial.println("err"); 
    }
    else if (sscanf(cmd.c_str(), "g o %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            char s = 'o';
            if (luminaire.state == LuminaireState::LOW) s = 'l';
            else if (luminaire.state == LuminaireState::HIGH) s = 'h';
            Serial.print("o "); Serial.print(target_id); Serial.print(" "); Serial.println(s);
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "a %d %f", &target_id, &val) == 2) {
        if (target_id == luminaire.getId()) {
            if (val == 0.0f || val == 1.0f) {
                pid.anti_windup = (val > 0.5f);
                Serial.println("ack");
            } else Serial.println("err");
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "g a %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.print("a "); Serial.print(target_id); Serial.print(" "); Serial.println(pid.anti_windup ? 1 : 0); 
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "f %d %f", &target_id, &val) == 2) {
        if (target_id == luminaire.getId()) {
            if (val == 0.0f || val == 1.0f) {
                luminaire.feedback_on = (val > 0.5f);
                Serial.println("ack"); 
            } else Serial.println("err"); 
        } else Serial.println("err"); 
    }
    else if (sscanf(cmd.c_str(), "g f %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.print("f "); Serial.print(target_id); Serial.print(" "); Serial.println(luminaire.feedback_on ? 1 : 0);
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "g d %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.print("d "); Serial.print(target_id); Serial.print(" "); Serial.println(box.get_background(ldr), 2); 
        } else Serial.println("err"); 
    }
    else if (sscanf(cmd.c_str(), "g p %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            float instant_power = metrics.getInstantaneousPower(luminaire.current_u);
            Serial.print("p "); Serial.print(target_id); Serial.print(" "); Serial.println(instant_power, 4); 
        } else Serial.println("err"); 
    }
    else if (sscanf(cmd.c_str(), "g t %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            float seconds = millis() / 1000.0f;
            Serial.print("t "); Serial.print(target_id); Serial.print(" "); Serial.println(seconds, 3);
        } else Serial.println("err"); 
    }
    else if (sscanf(cmd.c_str(), "s %c %d", &type, &target_id) == 2) {
        if (target_id == luminaire.getId()) {
            if (type == 'y' || type == 'u') {
                luminaire.stream_var = type;
                luminaire.streaming = true;
            } else Serial.println("err");
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "sb %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            luminaire.stream_var = 'b'; 
            luminaire.streaming = true;
            Serial.println("ack");
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "S %c %d", &type, &target_id) == 2) {
        if (target_id == luminaire.getId()) {
            if (type == 'y' || type == 'u' || type == 'b') {
                luminaire.streaming = false;
                net_streaming = false;
                Serial.println("ack");
            } else Serial.println("err");
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "g b %c %d", &type, &target_id) == 2) {
        if (target_id == luminaire.getId() && (type == 'y' || type == 'u')) {
            Serial.print("b "); Serial.print(type); Serial.print(" ");
            Serial.print(target_id); Serial.print(" ");
            for (int i = 0; i < BUFFER_SIZE; i++) {
                Serial.print(metrics.getBufferValue(type, i), 2);
                if (i < BUFFER_SIZE - 1) Serial.print(",");
            }
            Serial.println(); 
        } else Serial.println("err");
    }

    // Table 3 - Limits and Costs Configuration
    else if (sscanf(cmd.c_str(), "U %d %f", &target_id, &val) == 2) {
        if (target_id == luminaire.getId()) {
            luminaire.low_bound = val;
            Serial.println("ack");
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "O %d %f", &target_id, &val) == 2) {
        if (target_id == luminaire.getId()) {
            luminaire.high_bound = val;
            Serial.println("ack");
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "C %d %f", &target_id, &val) == 2) {
        if (target_id == luminaire.getId()) {
            global_energy_cost = val;
            Serial.println("ack");
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "g U %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.print("U "); Serial.print(target_id); Serial.print(" "); Serial.println(luminaire.low_bound, 2);
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "g O %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.print("O "); Serial.print(target_id); Serial.print(" "); Serial.println(luminaire.high_bound, 2);
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "g C %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            Serial.print("C "); Serial.print(target_id); Serial.print(" "); Serial.println(global_energy_cost, 4);
        } else Serial.println("err");
    }
    else if (sscanf(cmd.c_str(), "g L %d", &target_id) == 1) {
        if (target_id == luminaire.getId()) {
            float bound = (luminaire.state == LuminaireState::HIGH) ? luminaire.high_bound : luminaire.low_bound;
            Serial.print("L "); Serial.print(target_id); Serial.print(" "); Serial.println(bound, 2);
        } else Serial.println("err");
    }

    // Unknown command handling
    else {
        Serial.println("Error: Unknown command");
    }
}