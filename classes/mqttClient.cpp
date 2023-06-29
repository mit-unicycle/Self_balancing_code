#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/**
 * Class that is responsible for MQTT connection and messages
 * 
 * Libraries used: 
 *  - PubSubClient by Nick O'Leary
 *  - ArduinoJson by Benoit Blanchon
 * 
 * Written by David Farah
 */
class MqttClient {
  private:
    WiFiClient wifiClient;
    PubSubClient mqttClient;
    const char* mqttServer;
    const char* mqttUser;
    const char* mqttPassword;
    int mqttPort;
    void (*callback)(char*, byte*, unsigned int);

    char Outbuffer[10];
    char Inbuffer[10];

  public:
    /**
     * Constructor for the MqttClient class.
     * 
     * @param server The MQTT server hostname or IP address.
     * @param port The MQTT server port.
     * @param user The MQTT username.
     * @param password The MQTT password.
     * @param mqttCallback The callback function to handle received MQTT messages.
     */
    MqttClient(const char* server, int port, const char* user, const char* password, void (*mqttCallback)(char* topic, byte* payload, unsigned int length))
      : mqttClient(wifiClient) {
      mqttServer = server;
      mqttPort = port;
      mqttUser = user;
      mqttPassword = password;
      callback = mqttCallback;
    }

    /**
     * Connects to the MQTT broker.
     * This method should be called in the setup() function.
     */
    void connect() {
      mqttClient.setServer(mqttServer, mqttPort);
      mqttClient.setCallback(callback);
      
      while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT broker...");
        if (mqttClient.connect("ESP8266Client", mqttUser, mqttPassword)) {
          Serial.println("connected");
        } else {
          Serial.print("failed with state ");
          Serial.print(mqttClient.state());
          delay(2000);
        }
      }
    }

    /**
     * Sets up the WiFi connection.
     * This method should be called in the setup() function before connecting to MQTT.
     */
    void setupWifi() {
      WiFi.begin("Hotspot name", "pass");

      while (WiFi.status() != WL_CONNECTED) {
        delay(2000);
        Serial.println("Connecting to WiFi...");
      }
      Serial.println("Connected to WiFi");
    }

    /**
     * Main loop function.
     * This method should be called in the loop() function.
     * It handles MQTT connection and message handling.
     */
    void loop() {
      if (!mqttClient.connected()) {
        connect();
      }
      mqttClient.loop();
    }

    /**
     * Publishes a message to the specified MQTT topic.
     * 
     * @param topic The MQTT topic to publish to.
     * @param message The message to publish.
     */
    void publish(const char* topic, const char* message) {
      mqttClient.publish(topic, message);
    }

    /**
     * Subscribes to the specified MQTT topic.
     * 
     * @param topic The MQTT topic to subscribe to.
     */
    void subscribe(const char* topic) {
      mqttClient.subscribe(topic);
      Serial.println("Subscribed");
    }

    /**
     * Parses JSON payload containing an array of doubles.
     * 
     * @param payload The JSON payload to parse.
     * @param length The length of the payload.
     * @param maxDoubles The maximum number of doubles to parse.
     * @param doubles An array to store the parsed doubles.
     */
    void parseJSONtoDouble(byte* payload, unsigned int length, int maxDoubles, double (&doubles)[]){
      DynamicJsonDocument doc(1024);
      
      // Deserialize the JSON payload into the document
      deserializeJson(doc, payload, length);
      
      // Loop through the array and read each double
      int i = 0;
      for (JsonVariant value : doc.as<JsonArray>()) {
        // Check if we've reached the maximum number of doubles
        if (i >= maxDoubles) {
          break;
        }
        
        // Check if the value is a number
        if (value.is<double>()) {
          // Store the double in the array
          doubles[i] = value.as<double>();
          i++;
        }
      }
    }

    /**
     * Publishes input and output angles to MQTT topics.
     * 
     * @param inputAngle The input angle to publish.
     * @param outputAngle The output angle to publish.
     */
    void publishAngles(double inputAngle, double outputAngle){
      snprintf(Inbuffer, 10, "%lf", inputAngle); // format double as string with %lf
      mqttClient.publish("input", Inbuffer);

      snprintf(Outbuffer, 10, "%lf", outputAngle); // format double as string with %lf
      mqttClient.publish("output", Outbuffer);
    }
};
