#include "config.h"
#include SENSOR_LIST_HEADER

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <ArduinoOTA.h>

#define ONE_WIRE_BUS 3
#define MAX_SENSORS 10

#ifndef MQTT_CLIENTID 
    #define MQTT_CLIENTID "DefaultClient"
#endif

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Wi-Fi credentials
const char* ssid = WIFISSID;
const char* password = WIFIPASSWORD;

// MQTT server credentials
const char* mqttServer = MQTT_SERVER;
const int mqttPort = MQTT_PORT;
const char* mqttUser = MQTT_USER;
const char* mqttPassword = MQTT_PASSWORD;

// MQTT topics
const char* mqttClientID = MQTT_CLIENTID;
char mqttDataTopic[50];
char mqttInfoTopic[50];

// Wi-Fi and MQTT clients
WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);

//Sensors
DeviceAddress sensorAddresses[MAX_SENSORS]; // Array to hold sensor addresses
int detectedSensorsCount = 0;                   // Counter for detected sensors
char* detectecSensors[MAX_SENSORS] = {};

float lastSentTemperatures[MAX_SENSORS] = {-127}; // Initialize to 0 or a known invalid temperature
const float temperatureChangeThreshold = 0.1;

// NTP server and timezone settings
const char* ntpServer = "pool.ntp.org";
// Timezone for Europe/Budapest
const char* tzInfo = "CET-1CEST,M3.5.0/2,M10.5.0/3";

//Function declarations
void setupOTA();
void connectToWIFI();
void connectToMQTT();
void sendMQTTMessage(const char* topic, const char* message);
char* convertDallasDeviceAddress(DeviceAddress address);
void findDallasDevices();
void checkFoundSensors();
String getCurrentTime();

void setup() {
    Serial.begin(115200);
    connectToWIFI();

    setupOTA();
    ArduinoOTA.setPassword(OTAPWD);
    
    pubSubClient.setServer(mqttServer, mqttPort);
    connectToMQTT();

    // Construct the MQTT topics
    snprintf(mqttDataTopic, sizeof(mqttDataTopic), "%s/Data", mqttClientID);
    snprintf(mqttInfoTopic, sizeof(mqttInfoTopic), "%s/Info", mqttClientID);

    // Start the DallasTemperature library
    sensors.begin();
    // Set all sensors to 12-bit resolution
    sensors.setResolution(12); // Default for all sensors
    for (int i = 0; i < sensors.getDeviceCount(); i++) {
        sensors.setResolution(sensorAddresses[i], 12);
    }
    // Discover and store sensor addresses
    findDallasDevices();
    // Check found sensors
    checkFoundSensors();

    // Configure NTP
    configTime(0, 0, ntpServer);
    setenv("TZ", tzInfo, 1);
    tzset();
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        connectToWIFI();
    }

    ArduinoOTA.handle();  // Listen for OTA updates
    
    sensors.requestTemperatures(); // Send the command to get temperature

    // Get the current time as a C-style string
    String currentTime = getCurrentTime();
    const char* currentTimeCStr = currentTime.c_str();

    for (int i = 0; i < detectedSensorsCount; i++) {
        if (detectecSensors[i] != nullptr) { // Ensure the sensor ID is valid
            float tempC = sensors.getTempC(sensorAddresses[i]); // Get temperature for the specific sensor
            if (tempC != DEVICE_DISCONNECTED_C) {
                float lastTemp = lastSentTemperatures[i];
                if (abs(tempC - lastTemp) >= temperatureChangeThreshold) {
                    Serial.print("Temperature for sensor ");
                    Serial.print(detectecSensors[i]);
                    Serial.print(": ");
                    Serial.print(tempC);
                    Serial.println(" °C (change detected)");

                    // Create a JSON-like message
                    char message[150];
                    snprintf(message, sizeof(message), 
                             "{\"reading_time\": \"%s\", \"sensor_id\": \"%s\", \"temperature\": %.1f}", 
                             currentTimeCStr, detectecSensors[i], tempC);

                    // Publish the message
                    sendMQTTMessage(mqttDataTopic, message);

                    // Update the last sent temperature
                    lastSentTemperatures[i] = tempC;
                } else {
                    Serial.print("Temperature for sensor ");
                    Serial.print(detectecSensors[i]);
                    Serial.print(": ");
                    Serial.print(tempC);
                    Serial.println(" °C (no significant change)");
                }
            } else {
                Serial.print("Error: Could not read temperature for sensor ");
                Serial.println(detectecSensors[i]);

                // Optionally, send an error message for this sensor
                char errorMessage[200];
                snprintf(errorMessage, sizeof(errorMessage), 
                         "{\"reading_time\": \"%s\", \"sensor_id\": \"%s\", \"error\": \"Disconnected\"}", 
                         currentTimeCStr, detectecSensors[i]);
                sendMQTTMessage(mqttInfoTopic, errorMessage);
            }
        }
    }

    delay(5000); // Adjust reading interval
}

void connectToWIFI() {
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);  // Connect to the Wi-Fi network
    const int maxConnectionAttempts = 10;
    int connectionAttempts = 0;

    while (WiFi.status() != WL_CONNECTED && connectionAttempts < maxConnectionAttempts) {
        delay(1000);
        Serial.print(".");
        connectionAttempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWi-Fi connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to Wi-Fi.");
        ESP.restart();
    }
}

void connectToMQTT() {
    const unsigned long timeout = 30000; // 30 seconds timeout
    unsigned long startAttemptTime = millis();

    while (!pubSubClient.connected()) {
        Serial.println("Connecting to MQTT...");

        if (pubSubClient.connect(mqttClientID, mqttUser, mqttPassword)) {
            Serial.println("Connected to MQTT");
            return;
        } else {
            Serial.print("Failed to connect to MQTT. Error code: ");
            Serial.println(pubSubClient.state());

            Serial.println("Retrying in 5 seconds...");
            delay(5000);
        }

        if (millis() - startAttemptTime > timeout) {
            Serial.println("MQTT connection timed out. Restarting...");
            ESP.restart();
        }
    }
}

void setupOTA() {
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        // NOTE: if updating SPIFFS, this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\nUpdate Complete!");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress * 100) / total);
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
        }
    });

    ArduinoOTA.begin();  // Start the OTA service
    Serial.println("OTA Ready. IP address: " + WiFi.localIP().toString());
}

void sendMQTTMessage(const char* topic, const char* message) {
    if (!pubSubClient.connected()) {
        Serial.print("Not connected to MQTT");
        connectToMQTT();
    }
    if (pubSubClient.connected()) {
        if (pubSubClient.publish(topic, message)) {
            Serial.print("MQTT message sent to topic ");
            Serial.print(topic);
            Serial.print(": ");
            Serial.println(message);
        } else {
            Serial.print("Failed to send MQTT message to topic ");
            Serial.println(topic);
        }
    } else {
        Serial.println("MQTT client not connected. Unable to send message.");
    }
}

char* convertDallasDeviceAddress(DeviceAddress address) {
    // Allocate memory for the address string (8 bytes x 2 hex chars + 7 colons + 1 null terminator)
    char* deviceID = new char[24]; // 16 hex digits + 7 colons + null terminator
    char* ptr = deviceID;

    for (int i = 0; i < 8; i++) {
        // Write each byte as two hexadecimal characters
        sprintf(ptr, "%02X", address[i]);
        ptr += 2; // Move the pointer forward by 2 characters

        // Add colon after each pair except the last one
        if (i < 7) {
            *ptr = ':';
            ptr++;
        }
    }

    // Null-terminate the string
    *ptr = '\0';

    return deviceID;
}

void findDallasDevices() {
    // Locate devices on the bus
    Serial.println("Locating devices...");
    detectedSensorsCount = sensors.getDeviceCount();
    Serial.print("Number of devices found on the bus: ");
    Serial.println(detectedSensorsCount);

    if (detectedSensorsCount > MAX_SENSORS) {
        Serial.println("Warning: More sensors detected than the array can hold.");
        sendMQTTMessage(mqttInfoTopic,"Warning: More sensors detected than the array can hold.");
        detectedSensorsCount = MAX_SENSORS; // Limit to the array size
    }

    for (int i = 0; i < detectedSensorsCount; i++) {
        // Check if the address can be retrieved
        if (sensors.getAddress(sensorAddresses[i], i)) {
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.print(" Address: ");

            // Convert and print the address
            char* sensorID = convertDallasDeviceAddress(sensorAddresses[i]);
            detectecSensors[i] = sensorID;
            Serial.println(sensorID);
        } else {
            Serial.print("Unable to find address for sensor ");
            Serial.println(i);
        }
    }

    if (detectedSensorsCount == 0) {
        sendMQTTMessage(mqttInfoTopic,"No temperature sensor found.");
        Serial.println("No temperature sensor found.");
    }
}

void checkFoundSensors() {
    Serial.println("Validating sensors...");
    for (int i = 0; i < detectedSensorsCount; i++) {
        if (detectecSensors[i] != nullptr) {
            bool found = false;

            // Compare detected sensor with all expected sensors
            for (int j = 0; j < 10; j++) {
                if (expectedSensors[j] != nullptr && strcmp(detectecSensors[i], expectedSensors[j]) == 0) {
                    std::string message = "Detected sensor matches expected sensor: " + std::string(detectecSensors[i]);
                    sendMQTTMessage(mqttInfoTopic, message.c_str());
                    Serial.print("Detected sensor matches expected sensor: ");
                    Serial.println(detectecSensors[i]);
                    found = true;
                    break;
                }
            }

            if (!found) {
                std::string message = "Unknown sensor detected: " + std::string(detectecSensors[i]);
                sendMQTTMessage(mqttInfoTopic, message.c_str());
                Serial.print("Unknown sensor detected: ");
                Serial.println(detectecSensors[i]);
            }
        }
    }
}

String getCurrentTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "Failed to obtain time";
    }

    // Format time as a ISO 8601 string
    char timeStr[64];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S%z", &timeinfo);
    
    // Adjust timezone offset format (insert a colon in "+hhmm" or "-hhmm")
    String formattedTime = String(timeStr);
    formattedTime = formattedTime.substring(0, 22) + ":" + formattedTime.substring(22);

    return formattedTime;
}