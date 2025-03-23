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

// NeoPixel LED configuration
#define LED_PIN 27
#define NUMPIXELS 1

// Button configuration
#define BUTTON_PIN 39

// Second serial interface for NTRIPClient
#define TXD_PIN 21
#define RXD_PIN 22

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800); ///< NeoPixel LED object
NTRIPClient ntrip_c; ///< NTRIPClient object

WiFiClient espClient;
PubSubClient mqttClient(espClient);

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
static char nmeaBuffer[256] = {'\0'};
static int nmeaBufferIndex = {0};

#define LED_OFF_TIME_MS 200 ///< Time in milliseconds to turn off LED after last data received
#define NTRIP_TIMEOUT_MS 60000 ///< Time in milliseconds to reset system if no NTRIP data is received

// Flag to track receiving state
bool isReceiving = false;          ///< Flag to track if data is being received
unsigned long lastReceiveTime = {0}; ///< Time of last data received in ms

#define BUTTON_PRESS_TIME_MS 3000 ///< Time in milliseconds to hold button to enter configuration mode

// Button press handling
bool buttonPressed = false;
unsigned long buttonPressTime = {0};
bool configMode = false;

unsigned long lastGGASendTime = 0; ///< Variable to store the last GGA send time
const unsigned long GGA_SEND_INTERVAL = 300000; ///< 5 minutes in milliseconds

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
        Serial.println("  ERROR: Failed to mount file system");
        return;
    }

    File configFile = SPIFFS.open("/config.json", "r");
    if (!configFile) {
        Serial.println("  ERROR: Failed to open config file");
        return;
    }

    size_t size = configFile.size();
    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, buf.get());
    if (error) {
        Serial.println("  ERROR: Failed to parse config file");
        return;
    }

    strlcpy(_wifiSsid, doc["wifi"]["ssid"], sizeof(_wifiSsid));
    strlcpy(_wifiPassword, doc["wifi"]["password"], sizeof(_wifiPassword));
    strlcpy(_ntripHost, doc["ntrip"]["host"], sizeof(_ntripHost));
    _ntripHttpPort = doc["ntrip"]["port"];
    strlcpy(_ntripMountPoint, doc["ntrip"]["mntpnt"], sizeof(_ntripMountPoint));
    strlcpy(_ntripUser, doc["ntrip"]["user"], sizeof(_ntripUser));
    strlcpy(_ntripPassword, doc["ntrip"]["passwd"], sizeof(_ntripPassword));
    strlcpy(_mqttBroker, doc["mqtt"]["host"], sizeof(_mqttBroker));
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
    doc["mqtt"]["host"] = _mqttBroker;
    doc["mqtt"]["user"] = _mqttUser;
    doc["mqtt"]["passwd"] = _mqttPassword;
    doc["mqtt"]["topic"] = _mqttTopic;


    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
        Serial.println("  ERROR: Failed to open config file for writing");
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
        Serial.println("   INFO: Storing data to SPIFFS");
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

void reconnectMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("   INFO: Attempting MQTT connection...");
        if (mqttClient.connect("NTRIPClient", _mqttUser, _mqttPassword)) {
            Serial.println("connected");
        } else {
            Serial.print("Failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}


/**
 * @brief Setup function to initialize the system.
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200); // Default USB Serial (UART0)
    delay(10);

    // Initialize Serial2 for NTRIPClient
    // Quectel LG290P module uses UART2 for NTRIP communication at 460800 bps
    Serial2.begin(460800, SERIAL_8N1, RXD_PIN, TXD_PIN); // UART2 on GPIO21/22
    delay(10);

    // Initialize MQTT client
    mqttClient.setServer(_mqttBroker, 1883);
    reconnectMQTT();
        
    // Initialize NeoPixel
    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red for WiFi disconnected
    pixels.show();

    // Setup button
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Call for configuration mode to initialize WiFi, 
    // if button is pressed reconfiguration is required, start configuration mode
    configurationMode(digitalRead(BUTTON_PIN) == LOW);

    // Initialize NTRIP client
    Serial.println("   INFO: WiFi connected");
    Serial.print("   INFO: IP address: ");
    Serial.println(WiFi.localIP());
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green for WiFi connected
    pixels.show();
    
    // Request SourceTable from NTRIP server
    Serial.println("   INFO: Requesting SourceTable.");
    if(ntrip_c.reqSrcTbl(_ntripHost, _ntripHttpPort)) {
        char buffer[512];
        delay(5);
        while(ntrip_c.available()) {
            ntrip_c.readLine(buffer, sizeof(buffer));
            Serial.print(buffer); 
        }
    } else {
        Serial.println("  ERROR: SourceTable request error");
    }
    Serial.print("   INFO: Requesting SourceTable is OK\n");
    ntrip_c.stop(); // Need to call "stop" function for next request.
    
    // Request raw data from MountPoint
    Serial.println("   INFO: Requesting MountPoint's Raw data");
    if(!ntrip_c.reqRaw(_ntripHost, _ntripHttpPort, _ntripMountPoint, _ntripUser, _ntripPassword)) {
        delay(15000);
        ESP.restart();
    }
    Serial.println("   INFO: Requesting MountPoint is OK");
    pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Blue for NTRIP connected and receiving
    pixels.show();
}

/**
 * @brief Main loop function to handle data reception and button press.
 */
void loop() {
    // Check for button press to activate hotspot
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (!buttonPressed) {
            buttonPressed = true;
            buttonPressTime = millis();
        } else if (millis() - buttonPressTime > BUTTON_PRESS_TIME_MS) { // 3 seconds press
            configurationMode(true);
            buttonPressed = false;
        }
    } else {
        buttonPressed = false;
    }

    // Continuously read and print data from NTRIP server
    bool dataReceived = false;
    while(ntrip_c.available()) {
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
        Serial.println("WARNING: No NTRIP data received for more than a minute, restarting...");
        ESP.restart();
    }

    if (!ntrip_c.connected()) {
        Serial.println("  ERROR: NTRIP connection lost, restarting...");
        ESP.restart();
    }

    // Read and send GGA sentences
    while (Serial2.available()) {
        char ch = Serial2.read();

        // copy GNSS data to Serial.
        Serial1.print(ch);
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

                #ifdef DEBUG
                // Send GGA data to Serial for debug
                Serial.println(nmeaBuffer); // Send the complete GGA sentence to Serial
                #endif
            } else if (strstr(nmeaBuffer, "$GNGSA") != NULL) { // GNSS DOP and Active Satellites
                // Send GST data to Serial for debug
                #ifdef DEBUG
                Serial.println(nmeaBuffer); // Send the complete GST sentence to Serial
                #endif
            } else if (strstr(nmeaBuffer, "$GNVTG") != NULL) { // Course Over Ground and Ground Speed
                // Send RMC data to Serial for debug
                #ifdef DEBUG
                Serial.println(nmeaBuffer); // Send the complete RMC sentence to Serial
                #endif
            }


            // reset Indexe for buffer to restart colecting NMEA sentence
            nmeaBufferIndex = 0;
        } else {
            // Add character to buffer
            if (nmeaBufferIndex < sizeof(nmeaBuffer) - 1) {
                nmeaBuffer[nmeaBufferIndex++] = ch;
            }
        }
    }
}