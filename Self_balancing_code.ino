#include "classes/FSM.cpp"

FSM fsm;
Scheduler mqttLimit(10);

void mqttCallback(char* topic, byte* payload, unsigned int length);
MqttClient mqttClient("Broker IP", 1883, "user", "pass", mqttCallback);

/**
 * Method that is called in the beginning of excecution for setup
*/
void setup() {
  Serial.begin(250000);

  fsm.setup();

  mqttClient.setupWifi();
  mqttClient.connect();
  mqttClient.subscribe("params");
  mqttClient.subscribe("send");
}

/**
 * Loop method that is used to run code in an infinite loop
*/
void loop() {
    if(mqttLimit.hasPassed()){
        mqttClient.loop();
    }
    fsm.loop();
}

/**
 * Method called whenever a new mqtt message is received
 * @param topic The topic on which the message is received
 * @param payload The payload that is received
 * @param length The length of the payload
*/
void mqttCallback(char* topic, byte* payload, unsigned int length){
    if(strcmp(topic, "params") == 0) {
        double doubles[8];
        mqttClient.parseJSONtoDouble(payload, length, 8, doubles);
        
        fsm.setSetpointRight(doubles[3]);
        fsm.setTuningsRight(doubles[0], doubles[1], doubles[2]);

        fsm.setSetpointLeft(doubles[7]);
        fsm.setTuningsLeft(doubles[4], doubles[5], doubles[6]);

        mqttClient.publish("feedback", "Updated the tunings");
        
    } else if(strcmp(topic, "send") == 0){
        // Create a string from the vector array
        std::string result;

        result += "[" ;

        for (const auto& num : fsm.getParams()) {
            result += std::to_string(num) + ",";

        }
        result.pop_back();
        result += "]" ;

        mqttClient.publish("currParams", result.c_str());
    }
    else {
        String message = "";
        for (int i = 0; i < length; i++) {message += (char)payload[i];}   
        Serial.print("Topic: "); Serial.println(topic);
        Serial.print("Message: "); Serial.println(message);
    }
}