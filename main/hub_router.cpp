#include "hub_router.h"

void routeCommandToCAN(String cmd, int sender_id, CanNetwork& can_net) {
    cmd.trim();
    int target_id;
    float val;
    char char_val;
    char type;

    CanMessage msg;
    msg.type = 'C'; // It's a Command
    msg.sender_id = sender_id;
    bool valid_route = false;

    // Parse the command to extract target_id, variable type, and value
    if (sscanf(cmd.c_str(), "p %d %f", &target_id, &val) == 2) {
        msg.target_id = target_id; msg.variable = 'p'; msg.value = val; valid_route = true;
    } 
    else if (sscanf(cmd.c_str(), "w %d %f", &target_id, &val) == 2) {
        msg.target_id = target_id; msg.variable = 'w'; msg.value = val; valid_route = true;
    }
    else if (sscanf(cmd.c_str(), "ff %d %f", &target_id, &val) == 2) {
        msg.target_id = target_id; msg.variable = 'F'; msg.value = val; valid_route = true;
    }
    else if (sscanf(cmd.c_str(), "sb %d", &target_id) == 1) {
        msg.target_id = target_id; msg.variable = 's'; msg.value = (float)'b'; valid_route = true;
    }
    else if (sscanf(cmd.c_str(), "u %d %f", &target_id, &val) == 2) {
        msg.target_id = target_id; msg.variable = 'u'; msg.value = val; valid_route = true;
    }
    else if (sscanf(cmd.c_str(), "r %d %f", &target_id, &val) == 2) {
        msg.target_id = target_id; msg.variable = 'r'; msg.value = val; valid_route = true;
    }
    else if (sscanf(cmd.c_str(), "o %d %c", &target_id, &char_val) == 2) {
        msg.target_id = target_id; msg.variable = 'o'; msg.value = (float)char_val; valid_route = true;
    }
    else if (sscanf(cmd.c_str(), "a %d %f", &target_id, &val) == 2) {
        msg.target_id = target_id; msg.variable = 'a'; msg.value = val; valid_route = true;
    }
    else if (sscanf(cmd.c_str(), "f %d %f", &target_id, &val) == 2) {
        msg.target_id = target_id; msg.variable = 'f'; msg.value = val; valid_route = true;
    }
    else if (sscanf(cmd.c_str(), "s %c %d", &type, &target_id) == 2) {
        msg.target_id = target_id; msg.variable = 's'; msg.value = (float)type; valid_route = true;
    }
    else if (sscanf(cmd.c_str(), "S %c %d", &type, &target_id) == 2) {
        msg.target_id = target_id; msg.variable = 'S'; msg.value = (float)type; valid_route = true;
    }

    // If we successfully parsed a routable command, send it
    if (valid_route) {
        can_net.sendMessage(msg);
        Serial.println("ack (routed)");
    } else {
        // If it was a 'g' (get) command or something we don't route yet
        Serial.println("err"); 
    }
}