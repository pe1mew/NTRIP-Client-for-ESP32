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

static bool ntripConnected = false;
static unsigned long ntripConnectAttemptTime = 0;
static const unsigned long ntripConnectRetryInterval = 100; // 100 ms 
static unsigned long ntripLedOffTime = 0;
static bool ntripLedBlinking = false;

static unsigned long mqttLedOffTime = 0;
static bool mqttLedBlinking = false;

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
 * @param[in] buttonPressed Indicates if the button was pressed to enter configuration mode.
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
 * \brief Function to compose the message with Control-A, message, CRC-16, and Control-X, using byte stuffing.
 * \param[in] message The message string to be included in the message.
 * \param[out] outputBuffer The buffer to store the composed message.
 * \param[in] outputLength The length of the composed message.
 * \details The message format is as follows:
 * - Control-A (0x01)
 * - message (string), with byte stuffing
 * - CRC-16 (2 bytes, big-endian), with byte stuffing
 * - Control-X (0x18)
 * 
 * Byte stuffing:
 *   - Escape byte: 0x10 (DLE)
 *   - If a data or CRC byte is 0x01, 0x18, or 0x10, insert 0x10 before it.
 */
void composeMessage(const char* message, uint8_t* outputBuffer, size_t& outputLength) {
    const uint8_t controlA = 0x01; // Control-A
    const uint8_t controlX = 0x18; // Control-X
    const uint8_t escape   = 0x10; // DLE

    size_t index = 0;
    outputBuffer[index++] = controlA;

    // Append the message with byte stuffing
    size_t messageLength = strlen(message);
    for (size_t i = 0; i < messageLength; ++i) {
        uint8_t b = (uint8_t)message[i];
        if (b == controlA || b == controlX || b == escape) {
            outputBuffer[index++] = escape;
        }
        outputBuffer[index++] = b;
    }

    // Calculate CRC-16 over the message (not including Control-A)
    uint16_t crc = calculateCRC16((const uint8_t*)message, messageLength);

    // Append CRC-16 (big-endian) with byte stuffing
    uint8_t crcHigh = (crc >> 8) & 0xFF;
    uint8_t crcLow  = crc & 0xFF;
    if (crcHigh == controlA || crcHigh == controlX || crcHigh == escape) {
        outputBuffer[index++] = escape;
    }
    outputBuffer[index++] = crcHigh;
    if (crcLow == controlA || crcLow == controlX || crcLow == escape) {
        outputBuffer[index++] = escape;
    }
    outputBuffer[index++] = crcLow;

    // Append Control-X
    outputBuffer[index++] = controlX;

    // Set the output length
    outputLength = index;
}

/**
 * @brief Setup function to initialize the system.
 */
void setup() {
    configurationMode(false); // Ensure WiFi is initialized and credentials are loaded/applied
    
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

    // Start NTRIP connection attempt, but do not block
    Serial.print("[INFO] Connecting to NTRIP server: ");
    Serial.print(_ntripHost);
    Serial.print(":");
    Serial.println(_ntripHttpPort);

    // Initiate connection (non-blocking)
    ntrip_c.connect(_ntripHost, _ntripHttpPort);

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

    // NTRIP LED blink logic
    if (ntripConnected) {
        if (dataReceived && !ntripLedBlinking) {
            digitalWrite(NTRIP_LED, LOW); // Turn off LED briefly
            ntripLedOffTime = millis();
            ntripLedBlinking = true;
        }
        // Turn LED back on after a short moment (e.g., 10 ms)
        if (ntripLedBlinking && millis() - ntripLedOffTime > 100) {
            digitalWrite(NTRIP_LED, HIGH);
            ntripLedBlinking = false;
        }
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

        if (millis() - ntripConnectAttemptTime > ntripConnectRetryInterval) {
            Serial.println("[INFO] Attempting to connect to NTRIP server...");
            ntripConnectAttemptTime = millis();
            ntrip_c.connect(_ntripHost, _ntripHttpPort);
        }
        ntripConnected = false;
    } else if (!ntripConnected) {
        Serial.println("[INFO] NTRIP connection established.");
        digitalWrite(NTRIP_LED, HIGH);
        pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Blue for NTRIP connected and receiving
        pixels.show();

        ntrip_c.reqRaw(_ntripHost, _ntripHttpPort, _ntripMountPoint, _ntripUser, _ntripPassword);

        ntripConnected = true;
    }
    
    
    // --- Always process and send serial data regardless of WiFi state ---
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
                // Serial.println(ggaBuffer); // Print the GGA sentence to Serial for debugging
                
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

                // Check if milliseconds part has 3 or 2 positions
                int milliseconds = 0;
                if (strlen(timeBuffer) > 7) {
                    if (strlen(&timeBuffer[7]) == 3) { // 3 positions for milliseconds
                        milliseconds = (timeBuffer[7] - '0') * 100 + (timeBuffer[8] - '0') * 10 + (timeBuffer[9] - '0');
                    } else if (strlen(&timeBuffer[7]) == 2) { // 2 positions for milliseconds
                        milliseconds = (timeBuffer[7] - '0') * 100 + (timeBuffer[8] - '0') * 10;
                    }
                }

                // Set LED status based on fix type
                if(fixType < 4) {
                    digitalWrite(FIX_RTKFLOAT_LED, LOW);
                    digitalWrite(FIX_RTK_LED, LOW);
                } else if (fixType == 4) {
                    digitalWrite(FIX_RTKFLOAT_LED, HIGH);
                    digitalWrite(FIX_RTK_LED, LOW);
                } else if (fixType == 5) {
                    digitalWrite(FIX_RTKFLOAT_LED, LOW);
                    digitalWrite(FIX_RTK_LED, HIGH);
                }
                
                // Get current date
                // Serial.println(rmcBuffer); // Print the VTG sentence to Serial for debugging

                token = strtok(rmcBuffer, ",");
                fieldIndex = 0;
                char dateBuffer[7] = {0};
                int year = 2025; 
                int month = 3;
                int day = 24;
                bool rmcValid = false;

                while (token != NULL) {
                    switch (fieldIndex) {
                        case 2: // Date
                            if (strcmp(token, "A") == 0) {
                                rmcValid = true; // Valid date format
                            }
                            break;  
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
                    // Extract speed from VTG sentence.
                    // Because VTG sentense may differ in format, we need to check token
                    // to determine if the previous token is speed and in which format it is, km/h or m/s.
                    // Check if the current token is "K" and the previous token is not null, 
                    // then convert the previous token from speed km/h to m/s.
                    if (strcmp(token, "K") == 0 && previousToken != nullptr) {
                        speed = atof(previousToken) / 3.6; // Convert the previous token from speed km/h to m/s
                    }

                    // retrieve direction only if direction it "T" (true north), else 
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
                    
                    // Update the previous token as part of speed retrievement and move to the next token.
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

                // ==== Ready parsing NMEA data, now we can send it to the telemetry unit: ===

                if (rmcValid) {
                    // Compose a time message for acquisition module and send it to Serial1:
                    uint8_t timeMessage[64] = {0};
                    size_t timeMessageLength = 0;
                    composeMessage(timestamp, timeMessage, timeMessageLength);
                    Serial1.write(timeMessage, timeMessageLength);
                }

                // ==== When MQTT is connected, we can send the data to the MQTT broker: ====

                // Collect data and format it into a JSON structure and send it to the MQTT broker
                if (mqttClient.connected()) {
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
                    doc["age"] = ageOfDifferentialData;

                    String jsonString;
                    serializeJson(doc, jsonString);
                    mqttClient.publish(_mqttTopic, jsonString.c_str());

                    // --- MQTT LED blink logic: turn off LED briefly after sending ---
                    digitalWrite(MQTT_LED, LOW); // Turn off LED briefly
                    mqttLedOffTime = millis();
                    mqttLedBlinking = true;
                }

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

    if (WiFi.status() != WL_CONNECTED) {
        static unsigned long lastAttempt = 0;
        if (millis() - lastAttempt > 10000) { // Try every 10 seconds
            Serial.println("[INFO] WiFi disconnected, trying to reconnect...");
            WiFi.begin(_wifiSsid, _wifiPassword);
            lastAttempt = millis();
        }
    }

    if (mqttLedBlinking && millis() - mqttLedOffTime > 100) { // 100 ms blink
        digitalWrite(MQTT_LED, HIGH);
        mqttLedBlinking = false;
    }
}