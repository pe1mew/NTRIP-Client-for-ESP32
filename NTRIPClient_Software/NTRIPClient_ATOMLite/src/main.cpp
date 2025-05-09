#define Debug  // Add this line to enable debug information

// --------------------------------------------------------------------
//   This file is part of the PE1MEW NTRIP Client.
//
//   The NTRIP Client is distributed in the hope that 
//   it will be useful, but WITHOUT ANY WARRANTY; without even the 
//   implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
//   PURPOSE.
// --------------------------------------------------------------------*/

/*!
   
 \file main.cpp
 \brief Main file for the NTRIPClient project
 \author Remko Welling (PE1MEW) 

 This file was based upon the following work:
  - [NTRIP-client-for-Arduino](https://github.com/GLAY-AK2/NTRIP-client-for-Arduino)
  - [ESPNTRIPClient](https://github.com/klstronics/ESPNTRIPClient)

 The original work is adapted for ATOM Lite ESP32 board and the following changes were made:
  - Added WiFiManager for easy configuration
  - Added SPIFFS for storing configuration
  - Added NeoPixel LED for status indication
  - Added button for configuration mode
  - Added GGA sentence parsing and sending
  - Added SourceTable request
  - Added debug information

  The NTRIPClient is a simple NTRIP client for the ESP32 platform. It is designed to connect to an NTRIP server and receive RTCM data. 
  The client can be configured using a button to enter configuration mode and set the WiFi and NTRIP server settings. 
  The client will connect to the NTRIP server and request the SourceTable to get the available mountpoints. 
  The client will then request the raw data from the specified mountpoint using the provided credentials. 
  The client will also read GGA sentences from the serial interface and send them to the NTRIP server. 
  The client will indicate the status using a NeoPixel LED. 

  The client will automatically reconnect to the NTRIP server if the connection is lost. 

  The client will store the configuration in SPIFFS and load it on startup. 

  The client will enter configuration mode if the button is pressed for 3 seconds. 

  The client will turn off the LED after 500ms if no data is received. 

  The client will print debug information if the Debug flag is set. 

  The client will restart if the NTRIP server is not reachable. 

  The client will restart if the mountpoint request fails. 

  The client will restart if the mountpoint request is successful but no data is received. 

  The client will restart if the mountpoint request is successful but the connection is lost. 

   
*/

#include <WiFi.h>
#include <WiFiManager.h>
#include "NTRIPClient.h"
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>

#include <stdint.h>
#include <string.h>
#include <Arduino.h>

#include "hardwareConfig.h" // Include hardware configuration file
#include "NTRIPClientCRC16.h" // Include the CRC-16 calculation header file

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800); ///< NeoPixel LED object
NTRIPClient ntrip_c;        ///< NTRIPClient object
WiFiClient espClient;       ///< WiFi client object
PubSubClient mqttClient(espClient); ///< MQTT client object

// Configuration variables
char _wifiSsid[32] = {'\0'};        ///< WiFi SSID
char _wifiPassword[32] = {'\0'};    ///< WiFi password
char _ntripHost[32] = {'\0'};       ///< NTRIP server host
int  _ntripHttpPort = {0};          ///< NTRIP server port
char _ntripMountPoint[32] = {'\0'}; ///< NTRIP mountpoint
char _ntripUser[32] = {'\0'};       ///< NTRIP username
char _ntripPassword[32] = {'\0'};   ///< NTRIP password
char _mqttBroker[32] = {'\0'};      ///< MQTT broker
char _mqttUser[32] = {'\0'};        ///< MQTT username
char _mqttPassword[32] = {'\0'};    ///< MQTT password
char _mqttTopic[64] = {'\0'};       ///< MQTT topic

// For reading and sending GGA sentences
static char nmeaBuffer[256] = {'\0'};   ///< Buffer to store NMEA sentences
static char ggaBuffer[256] = {'\0'};    ///< Buffer to store GGA sentences
static char rmcBuffer[256] = {'\0'};    ///< Buffer to store RMC sentences
static char vtgBuffer[256] = {'\0'};    ///< Buffer to store VTG sentences
static int nmeaBufferIndex = {0};
bool ggaReceived = false;
bool rmcReceived = false;
bool vtgReceived = false;


// Flag to track receiving state
bool isReceiving = false;          ///< Flag to track if data is being received
unsigned long lastReceiveTime = {0}; ///< Time of last data received in ms


// Button press handling
bool buttonPressed = false;
unsigned long buttonPressTime = {0};
bool configMode = false;

unsigned long lastGGASendTime = 0; ///< Variable to store the last GGA send time
unsigned long sequenceNumber = 0;   ///< Variable to store the sequence number for the JSON data

IPAddress mqttBrokerIP; // Variable to store the MQTT broker IP address

/**
 * @brief Print the current configuration to the serial output.
 */
void printConfig() {
    Serial.println("Current Configuration:");
    Serial.print("WiFi SSID: ");
    Serial.println(_wifiSsid);
    Serial.print("WiFi Password: ");
    Serial.println(_wifiPassword);
    Serial.print("NTRIP Host: ");
    Serial.println(_ntripHost);
    Serial.print("NTRIP Port: ");
    Serial.println(_ntripHttpPort);
    Serial.print("NTRIP Mountpoint: ");
    Serial.println(_ntripMountPoint);
    Serial.print("NTRIP Username: ");
    Serial.println(_ntripUser);
    Serial.print("NTRIP Password: ");
    Serial.println(_ntripPassword);
    Serial.print("MQTT Broker: ");
    Serial.println(_mqttBroker);
    Serial.print("MQTT Username: ");
    Serial.println(_mqttUser);
    Serial.print("MQTT Password: ");
    Serial.println(_mqttPassword);
    Serial.print("MQTT Topic: ");
    Serial.println(_mqttTopic);
}

/**
 * @brief Load configuration from SPIFFS.
 */
void loadConfig() {
    if (!SPIFFS.begin(true)) {
        Serial.println("[ERROR] Failed to mount file system");
        return;
    }

    File configFile = SPIFFS.open("/config.json", "r");
    if (!configFile) {
        Serial.println("[ERROR] Failed to open config file");
        return;
    }

    size_t size = configFile.size();
    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, buf.get());
    if (error) {
        Serial.println("[ERROR] Failed to parse config file");
        return;
    }

    strlcpy(_wifiSsid, doc["wifi"]["ssid"], sizeof(_wifiSsid));
    strlcpy(_wifiPassword, doc["wifi"]["password"], sizeof(_wifiPassword));
    strlcpy(_ntripHost, doc["ntrip"]["host"], sizeof(_ntripHost));
    _ntripHttpPort = doc["ntrip"]["port"];
    strlcpy(_ntripMountPoint, doc["ntrip"]["mntpnt"], sizeof(_ntripMountPoint));
    strlcpy(_ntripUser, doc["ntrip"]["user"], sizeof(_ntripUser));
    strlcpy(_ntripPassword, doc["ntrip"]["passwd"], sizeof(_ntripPassword));
    strlcpy(_mqttBroker, doc["mqtt"]["broker"], sizeof(_mqttBroker));
    mqttBrokerIP.fromString(_mqttBroker); // Convert string to IP address
    strlcpy(_mqttUser, doc["mqtt"]["user"], sizeof(_mqttUser));
    strlcpy(_mqttPassword, doc["mqtt"]["passwd"], sizeof(_mqttPassword));
    strlcpy(_mqttTopic, doc["mqtt"]["topic"], sizeof(_mqttTopic));

    #ifdef Debug
    printConfig();
    #endif
}

/**
 * @brief Save configuration to SPIFFS.
 */
void saveConfig() {
    DynamicJsonDocument doc(1024);
    doc["wifi"]["ssid"] = _wifiSsid;
    doc["wifi"]["password"] = _wifiPassword;
    doc["ntrip"]["host"] = _ntripHost;
    doc["ntrip"]["port"] = _ntripHttpPort;
    doc["ntrip"]["mntpnt"] = _ntripMountPoint;
    doc["ntrip"]["user"] = _ntripUser;
    doc["ntrip"]["passwd"] = _ntripPassword;
    doc["mqtt"]["broker"] = mqttBrokerIP.toString(); // Convert IP address to string
    doc["mqtt"]["user"] = _mqttUser;
    doc["mqtt"]["passwd"] = _mqttPassword;
    doc["mqtt"]["topic"] = _mqttTopic;


    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
        Serial.println("[ERROR] Failed to open config file for writing");
        return;
    }

    serializeJson(doc, configFile);
}

/**
 * @brief Callback function to set the configuration mode flag.
 */
void saveConfigCallback() {
    configMode = true;
}

/**
 * @brief Enter configuration mode to initialize WiFi.
 * 
 * @param buttonPressed Indicates if the button was pressed to enter configuration mode.
 */
void configurationMode(const bool buttonPressed = false) {
    pixels.setPixelColor(0, pixels.Color(255, 255, 255)); // Orange for configuration portal
    pixels.show();

    // Try to load existing configuration
    loadConfig();

    WiFiManager wifiManager;

    // Custom parameters
    WiFiManagerParameter custom_host("host", "NTRIP Host", _ntripHost, sizeof(_ntripHost));
    WiFiManagerParameter custom_port("port", "NTRIP Port", String(_ntripHttpPort).c_str(), 6);
    WiFiManagerParameter custom_mntpnt("mntpnt", "NTRIP Mountpoint", _ntripMountPoint, sizeof(_ntripMountPoint));
    WiFiManagerParameter custom_user("user", "NTRIP Username", _ntripUser, sizeof(_ntripUser));
    WiFiManagerParameter custom_passwd("passwd", "NTRIP Password", _ntripPassword, sizeof(_ntripPassword));
    WiFiManagerParameter custom_mqtt_broker("mqtt_broker", "MQTT Broker", _mqttBroker, sizeof(_mqttBroker));
    WiFiManagerParameter custom_mqtt_user("mqtt_user", "MQTT Username", _mqttUser, sizeof(_mqttUser));
    WiFiManagerParameter custom_mqtt_passwd("mqtt_passwd", "MQTT Password", _mqttPassword, sizeof(_mqttPassword));
    WiFiManagerParameter custom_mqtt_topic("mqtt_topic", "MQTT Topic", _mqttTopic, sizeof(_mqttTopic));

    // Add custom parameters to WiFiManager
    wifiManager.addParameter(&custom_host);
    wifiManager.addParameter(&custom_port);
    wifiManager.addParameter(&custom_mntpnt);
    wifiManager.addParameter(&custom_user);
    wifiManager.addParameter(&custom_passwd);
    wifiManager.addParameter(&custom_mqtt_broker);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_passwd);
    wifiManager.addParameter(&custom_mqtt_topic);

    wifiManager.setSaveConfigCallback(saveConfigCallback);
    wifiManager.setBreakAfterConfig(true);

    // If configuration is empty, start configuration portal
    if (strlen(_wifiSsid) == 0 || strlen(_wifiPassword) == 0 || buttonPressed) {
        // wifiManager.startConfigPortal("NTRIPClient_Config");

        wifiManager.startConfigPortal("NTRIPClient_Config");
        wifiManager.stopConfigPortal();
        wifiManager.disconnect();
    } else {
        WiFi.begin(_wifiSsid, _wifiPassword);
        if ((WiFi.waitForConnectResult() != WL_CONNECTED)) {
            // wifiManager.autoConnect();
            wifiManager.startConfigPortal("NTRIPClient_Config");
        }
    }

    if (configMode){
        // Save the new configuration to globals
        Serial.println("[INFO] Storing data to SPIFFS");
        strlcpy(_wifiSsid, WiFi.SSID().c_str(), sizeof(_wifiSsid));
        strlcpy(_wifiPassword, WiFi.psk().c_str(), sizeof(_wifiPassword));
        strlcpy(_ntripHost, custom_host.getValue(), sizeof(_ntripHost));
        _ntripHttpPort = atoi(custom_port.getValue());
        strlcpy(_ntripMountPoint, custom_mntpnt.getValue(), sizeof(_ntripMountPoint));
        strlcpy(_ntripUser, custom_user.getValue(), sizeof(_ntripUser));
        strlcpy(_ntripPassword, custom_passwd.getValue(), sizeof(_ntripPassword));
        strlcpy(_mqttBroker, custom_mqtt_broker.getValue(), sizeof(_mqttBroker));
        strlcpy(_mqttUser, custom_mqtt_user.getValue(), sizeof(_mqttUser));
        strlcpy(_mqttPassword, custom_mqtt_passwd.getValue(), sizeof(_mqttPassword));
        strlcpy(_mqttTopic, custom_mqtt_topic.getValue(), sizeof(_mqttTopic));
        
        // Save globals to SPIFFS
        saveConfig();
        ESP.restart();
    }
    #ifdef Debug
    printConfig();
    #endif
}

/**
 * @brief Initialize or reconnect to the MQTT broker.
 */
void reconnectMQTT() {
    if (mqttClient.connect("NTRIPClient", _mqttUser, _mqttPassword)) {
        Serial.println("connected");
    } else {
        Serial.print("Failed, rc=");
        Serial.print(mqttClient.state());
    }
}

/**
 * \brief Function to compose the message with Control-A, timestamp, CRC-16, and Control-X.
 * \param timestamp The timestamp string to be included in the message.
 * \param outputBuffer The buffer to store the composed message.
 * \param outputLength The length of the composed message.
 * \details The message format is as follows:
 * - Control-A (0x01)
 * - Timestamp (string)
 * - CRC-16 (2 bytes, big-endian)
 * - Control-X (0x18)
 * 
 * 1. Prefix the message with a byte representing "Control-A" (0x01).
 * 2. Append the timestamp to the message.
 * 3. Calculate the CRC-16 over the bytes (excluding the "Control-A" prefix).
 * 4. Append the CRC-16 result to the message.
 * 5. Close the message with a byte representing "Control-X" (0x18).
 */ 
void composeMessage(const char* timestamp, uint8_t* outputBuffer, size_t& outputLength) {
    const uint8_t controlA = 0x01; // Control-A
    const uint8_t controlX = 0x18; // Control-X

    // Start with Control-A
    size_t index = 0;
    outputBuffer[index++] = controlA;

    // Append the timestamp
    size_t timestampLength = strlen(timestamp);
    memcpy(&outputBuffer[index], timestamp, timestampLength);
    index += timestampLength;

    // Calculate CRC-16 over the timestamp
    uint16_t crc = calculateCRC16((const uint8_t*)timestamp, timestampLength);

    // Append the CRC-16 (big-endian)
    outputBuffer[index++] = (crc >> 8) & 0xFF; // High byte
    outputBuffer[index++] = crc & 0xFF;        // Low byte

    // Append Control-X
    outputBuffer[index++] = controlX;

    // Set the output length
    outputLength = index;
}

/**
 * @brief Setup function to initialize the system.
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200); // Initialize USB CDC at 115200 baud
    delay(1000);          // Allow time for the Serial Monitor to connect
    Serial.println("[INFO] USB CDC enabled on boot.");

    // Initialize Serial2 for NTRIPClient
    Serial.println("[INFO] Initializing Serial2 for NTRIPClient...");
    Serial2.begin(460800, SERIAL_8N1, RXD2_PIN, TXD2_PIN); // UART2 on GPIO17/18
    delay(10);

    // Initialize Serial1 for telemetry unit
    Serial.println("[INFO] Initializing Serial1 for telemetry unit...");
    Serial1.begin(115200, SERIAL_8N1, RXD1_PIN, TXD1_PIN); // UART1 on GPIO19/20
    delay(10);

    // Initialize NeoPixel
    Serial.println("[INFO] Initializing NeoPixel LED...");
    pixels.begin();
    pixels.setBrightness(LED_BRIGHTNESS); // Set brightness to 10%
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red for WiFi disconnected
    pixels.show();

    // Setup button
    Serial.println("[INFO] Configuring button input...");
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Setup LEDs and switch off
    pinMode(WIFI_LED, OUTPUT);
    pinMode(NTRIP_LED, OUTPUT);
    pinMode(MQTT_LED, OUTPUT);
    pinMode(FIX_RTKFLOAT_LED, OUTPUT);
    pinMode(FIX_RTK_LED, OUTPUT);
    digitalWrite(WIFI_LED, LOW);
    digitalWrite(NTRIP_LED, LOW);
    digitalWrite(MQTT_LED, LOW);
    digitalWrite(FIX_RTKFLOAT_LED, LOW);
    digitalWrite(FIX_RTK_LED, LOW);

    // Call for configuration mode to initialize WiFi
    Serial.println("[INFO] Checking for button press to enter configuration mode...");
    configurationMode(digitalRead(BUTTON_PIN) == LOW);

    // Initialize WiFi connection
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("[INFO] WiFi connected.");
        Serial.print("[INFO] IP address: ");
        Serial.println(WiFi.localIP());
        digitalWrite(WIFI_LED, HIGH); // Turn on WiFi LED
        pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green for WiFi connected
        pixels.show();
    } else {
        Serial.println("[ERROR] WiFi connection failed.");
        digitalWrite(WIFI_LED, LOW); // Turn off WiFi LED
    }

    // Log connection attempt to NTRIP server
    Serial.print("[INFO] Connecting to NTRIP server: ");
    Serial.print(_ntripHost);
    Serial.print(":");
    Serial.println(_ntripHttpPort);

    if (!espClient.connect(_ntripHost, _ntripHttpPort)) {
        Serial.print("[ERROR] Failed to connect to NTRIP server at ");
        Serial.print(_ntripHost);
        Serial.print(":");
        Serial.print(_ntripHttpPort);
        Serial.println(". Connection timed out after 3000 ms.");
        Serial.println("[DEBUG] Check if the server is reachable, the port is correct, and the network is stable.");
    } else {
        Serial.println("[INFO] Successfully connected to NTRIP server.");
    }

    // Request SourceTable from NTRIP server
    Serial.println("[INFO] Requesting SourceTable from NTRIP server...");
    if (ntrip_c.reqSrcTbl(_ntripHost, _ntripHttpPort)) {
        char buffer[512];
        delay(5);
        while (ntrip_c.available()) {
            ntrip_c.readLine(buffer, sizeof(buffer));
            Serial.print(buffer); 
        }
        Serial.println("[INFO] SourceTable request completed successfully.");
    } else {
        Serial.println("[ERROR] Failed to request SourceTable from NTRIP server.");
    }
    ntrip_c.stop(); // Need to call "stop" function for next request.

    // Request raw data from MountPoint
    Serial.println("[INFO] Requesting MountPoint's Raw data");
    if (!ntrip_c.reqRaw(_ntripHost, _ntripHttpPort, _ntripMountPoint, _ntripUser, _ntripPassword)) {
        delay(15000);
        ESP.restart();
    }
    Serial.println("[INFO] Successfully requested raw data from MountPoint.");
    pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Blue for NTRIP connected and receiving
    pixels.show();

    // Initialize MQTT client
    Serial.println("[INFO] Initializing MQTT client...");
    mqttClient.setServer(mqttBrokerIP, 1883);
    reconnectMQTT();
}

/**
 * @brief Main loop function to handle data reception and button press.
 */
void loop() {
    // Check WiFi connection status and update WiFi LED
    if (WiFi.status() == WL_CONNECTED) {
        digitalWrite(WIFI_LED, HIGH); // Turn on WiFi LED
    } else {
        digitalWrite(WIFI_LED, LOW); // Turn off WiFi LED
    }

    // Check for button press to activate hotspot
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (!buttonPressed) {
            buttonPressed = true;
            buttonPressTime = millis();
        } else if (millis() - buttonPressTime > BUTTON_PRESS_TIME_MS) { // 3 seconds press
            Serial.println("[INFO] Button pressed for 3 seconds. Entering configuration mode...");
            configurationMode(true);
            buttonPressed = false;
        }
    } else {
        buttonPressed = false;
    }

    // Continuously read and print data from NTRIP server
    bool dataReceived = false;
    while (ntrip_c.available()) {
        char ch = ntrip_c.read();        
        Serial2.print(ch);
        dataReceived = true;
    }

    // Update LED based on receiving state
    if (dataReceived) {
        lastReceiveTime = millis();
        pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green for data received
        pixels.show();
    } else if (millis() - lastReceiveTime > LED_OFF_TIME_MS) {
        pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Blue for no data received
        pixels.show();
    }

    // Check if no data received for more than NTRIP_TIMEOUT_MS
    if (millis() - lastReceiveTime > NTRIP_TIMEOUT_MS) {
        Serial.println("[WARNING] No NTRIP data received for more than a minute, restarting...");
        ESP.restart();
    }

    if (!ntrip_c.connected()) {
        digitalWrite(NTRIP_LED, LOW);
        Serial.println("[ERROR] NTRIP connection lost, restarting...");
        ESP.restart();
    }else{
        digitalWrite(NTRIP_LED, HIGH);
    }

    // Read and send GGA sentences
    while (Serial2.available()) {
        char ch = Serial2.read();

        // // copy GNSS data to Serial.
        // Serial1.print(ch);

        // When complete NMEA sentence is received parse sentense and process relevant results
        if (ch == '\n') {
            // Ensure that the buffer is "\0" terminated. 
            nmeaBuffer[nmeaBufferIndex] = '\0';
            // Process relevant NMEA sentences
            if (strstr(nmeaBuffer, "$GNGGA") != NULL) {
                
                // Check if it is time to send GGA sentence to NTRIP server
                // Send GGA sentence to NTRIP server every GGA_SEND_INTERVAL milliseconds
                unsigned long currentTime = millis();
                if (currentTime - lastGGASendTime >= GGA_SEND_INTERVAL) {
                    ntrip_c.sendGGA(nmeaBuffer); // Send the GGA sentence to NTRIP server
                    lastGGASendTime = currentTime; // Update the last GGA send time
                }

                strlcpy(ggaBuffer, nmeaBuffer, sizeof(ggaBuffer)); // Copy GGA sentence to buffer
                ggaReceived = true; // Set flag to indicate that GGA sentence is received

                // mqttClient.publish("ntripclient/gga", ggaBuffer); // Send the GGA sentence to MQTT broker

                #ifdef DEBUG
                // Send GGA data to Serial for debug
                Serial.println(nmeaBuffer); // Send the complete GGA sentence to Serial
                #endif

            } else if (strstr(nmeaBuffer, "$GNRMC") != NULL) { // Recommended Minimum Navigation Information

                strlcpy(rmcBuffer, nmeaBuffer, sizeof(rmcBuffer)); // Copy RMC sentence to buffer
                rmcReceived = true; // Set flag to indicate that RMC sentence is received

                // Send RMC data to Serial for debug
                #ifdef DEBUG
                Serial.println(nmeaBuffer); // Send the complete RMC sentence to Serial
                #endif
            } else if (strstr(nmeaBuffer, "$GNVTG") != NULL) { // Course Over Ground and Ground Speed

                strlcpy(vtgBuffer, nmeaBuffer, sizeof(vtgBuffer)); // Copy VTG sentence to buffer
                vtgReceived = true; // Set flag to indicate that VTG sentence is received

                // Send RMC data to Serial for debug
                #ifdef DEBUG
                Serial.println(nmeaBuffer); // Send the complete VTG sentence to Serial
                #endif

            }

            // reset Index for buffer to restart colecting NMEA sentence
            nmeaBufferIndex = 0;

            // Check if all relevant NMEA sentences are received
            // If all sentences are received, parse the data and send it to the MQTT broker
            if(ggaReceived && rmcBuffer && vtgReceived) {
                // Reset flags
                ggaReceived = false;
                // Because RMC is only used to get the date, it is not necessary to reset the flag
                // rmcReceived = false;
                vtgReceived = false;

                // The NMEA GGA sentence provides latitude and longitude in
                // a specific format, and additional processing is required 
                // to convert it into decimal degrees.
                //
                // NMEA Latitude and Longitude Format:
                // 1. Latitude: ddmm.mmmm (degrees and minutes)
                //    - dd: Degrees (2 digits)
                //    - mm.mmmm: Minutes (up to 4 decimal places)
                // 2. Longitude: dddmm.mmmm (degrees and minutes)
                //    - ddd: Degrees (3 digits)
                //    - mm.mmmm: Minutes (up to 4 decimal places)
                // To convert these values into decimal degrees:
                //  - Decimal Degrees = Degrees + (Minutes / 60)

                // Serial.println(nmeaBuffer); // Send the complete RMC sentence to Serial
                
                // Parse GGA sentence
                char* token = strtok(ggaBuffer, ",");
                int fieldIndex = 0;
                double latitude = 0.0;
                double longitude = 0.0;
                double altitude = 0.0;
                double ageOfDifferentialData = 0.0;
                int fixType = 0;
                int satellites = 0;
                double hdop = 0.0;
                char timeBuffer[11] = {0};
                char latDirection = '\0';
                char lonDirection = '\0';

                while (token != NULL) {
                    switch (fieldIndex) {
                        case 1: // Time
                            strncpy(timeBuffer, token, sizeof(timeBuffer) - 1);
                            break;
                        case 2: // Latitude
                            latitude = atof(token);
                            break;
                        case 3: // Latitude direction (N/S)
                            latDirection = token[0];
                            break;
                        case 4: // Longitude
                            longitude = atof(token);
                            break;
                        case 5: // Longitude direction (E/W)
                            lonDirection = token[0];
                            break;
                        case 9: // Altitude
                            altitude = atof(token);
                            break;
                        case 6: // Fix type
                            fixType = atoi(token);
                            break;
                        case 7: // Satellites
                            satellites = atoi(token);
                            break;
                        case 8: // HDOP
                            hdop = atof(token);
                            break;
                        case 13: // Age of Differential Data
                            ageOfDifferentialData = atof(token);
                            break;
                    }
                    token = strtok(NULL, ",");
                    fieldIndex++;
                }

                // Convert latitude and longitude to decimal degrees
                int latDegrees = (int)(latitude / 100);
                double latMinutes = latitude - (latDegrees * 100);
                latitude = latDegrees + (latMinutes / 60.0);
                if (latDirection == 'S') {
                    latitude = -latitude; // South is negative
                }

                int lonDegrees = (int)(longitude / 100);
                double lonMinutes = longitude - (lonDegrees * 100);
                longitude = lonDegrees + (lonMinutes / 60.0);
                if (lonDirection == 'W') {
                    longitude = -longitude; // West is negative
                }

                // Parse time from GGA sentence
                int hours = (timeBuffer[0] - '0') * 10 + (timeBuffer[1] - '0');
                int minutes = (timeBuffer[2] - '0') * 10 + (timeBuffer[3] - '0');
                int seconds = (timeBuffer[4] - '0') * 10 + (timeBuffer[5] - '0');
                int milliseconds = (timeBuffer[7] - '0') * 100 + (timeBuffer[8] - '0') * 10 + (timeBuffer[9] - '0');

                
                // Get current date
                token = strtok(rmcBuffer, ",");
                fieldIndex = 0;
                char dateBuffer[7] = {0};
                int year = 2025; 
                int month = 3;
                int day = 24;

                while (token != NULL) {
                    switch (fieldIndex) {
                        case 9:
                            strncpy(dateBuffer, token, sizeof(dateBuffer) - 1);
                            break;
                    }
                    token = strtok(NULL, ",");
                    fieldIndex++;
                }
            
                if (strlen(dateBuffer) == 6) {
                    day = (dateBuffer[0] - '0') * 10 + (dateBuffer[1] - '0');
                    month = (dateBuffer[2] - '0') * 10 + (dateBuffer[3] - '0');
                    year = 2000 + (dateBuffer[4] - '0') * 10 + (dateBuffer[5] - '0');
                }

                // Format timestamp
                char timestamp[24];
                snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d.%03d", year, month, day, hours, minutes, seconds, milliseconds);

                // Parse VTG sentence
                // Serial.println(vtgBuffer); // Print the VTG sentence to Serial for debugging

                token = strtok(vtgBuffer, ",");
                fieldIndex = 0;
                double speed = 0.0;
                double direction = 0.0;
                char* previousToken = nullptr; // To store the previous token

                while (token != NULL) {

                    // Serial.print("Token ");
                    // Serial.print(fieldIndex);
                    // Serial.print(": ");
                    // Serial.println(token);

                    // Check if the current token is "K"
                    if (strcmp(token, "K") == 0 && previousToken != nullptr) {
                        speed = atof(previousToken) / 3.6; // Convert the previous token from speed km/h to m/s
                    }

                    switch (fieldIndex) {
                        case 1: // Direction (true north)
                            direction = atof(token);
                            break;
                        case 2: // Validate 'T' field
                            if (strcmp(token, "T") != 0) {
                                direction = 0.0; // Invalid direction
                            }
                            break;
                    }
                    
                    // Update the previous token and move to the next
                    previousToken = token;
                    token = strtok(NULL, ",");
                    fieldIndex++;
                }

                // Collect data and format it into a JSON structure
                DynamicJsonDocument doc(1024);
                doc["id"] = sequenceNumber++;
                doc["daytime"] = timestamp;
                doc["lat"] = latitude;
                doc["lon"] = longitude;
                doc["alt"] = altitude;
                doc["fix_type"] = fixType;
                doc["speed"] = speed;
                doc["dir"] = direction;
                doc["sats"] = satellites;
                doc["hdop"] = hdop;
                doc["age"] = ageOfDifferentialData; // Add Age of Differential Data

                String jsonString;
                serializeJson(doc, jsonString);
                mqttClient.publish(_mqttTopic, jsonString.c_str());
                
                uint8_t message[64]; // Buffer to hold the composed message
                size_t messageLength = 0;

                // Compose the message
                composeMessage(timestamp, message, messageLength);

                // Send the composed message to Serial1
                Serial1.write(message, messageLength);

            }

        } else {
            // Add character to buffer
            if (nmeaBufferIndex < sizeof(nmeaBuffer) - 1) {
                nmeaBuffer[nmeaBufferIndex++] = ch;
            }
        }
    }

    // Handle MQTT client
    if (!mqttClient.connected()) {
        digitalWrite(MQTT_LED, LOW);
        reconnectMQTT();
    }else{
        digitalWrite(MQTT_LED, HIGH);
    }
    mqttClient.loop(); // Ensure the MQTT client is processing incoming messages
}