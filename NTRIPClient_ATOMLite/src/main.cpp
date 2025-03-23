// #define Debug  // Add this line to enable debug information

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

// Configuration variables
char ssid[32] = {'\0'};      ///< WiFi SSID
char password[32] = {'\0'};  ///< WiFi password
char host[32] = {'\0'};      ///< NTRIP server host
int httpPort = {0};          ///< NTRIP server port
char mntpnt[32] = {'\0'};    ///< NTRIP mountpoint
char user[32] = {'\0'};      ///< NTRIP username
char passwd[32] = {'\0'};    ///< NTRIP password

// For reading and sending GGA sentences
static char ggaBuffer[256] = {'\0'};
static int ggaIndex = {0};

#define LED_OFF_TIME_MS 500 ///< Time in milliseconds to turn off LED after last data received
#define NTRIP_TIMEOUT_MS 60000 ///< Time in milliseconds to reset system if no NTRIP data is received

// Flag to track receiving state
bool isReceiving = false;          ///< Flag to track if data is being received
unsigned long lastReceiveTime = {0}; ///< Time of last data received in ms

#define BUTTON_PRESS_TIME_MS 3000 ///< Time in milliseconds to hold button to enter configuration mode

// Button press handling
bool buttonPressed = false;
unsigned long buttonPressTime = {0};
bool configMode = false;

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

    strlcpy(ssid, doc["wifi"]["ssid"], sizeof(ssid));
    strlcpy(password, doc["wifi"]["password"], sizeof(password));
    strlcpy(host, doc["ntrip"]["host"], sizeof(host));
    httpPort = doc["ntrip"]["port"];
    strlcpy(mntpnt, doc["ntrip"]["mntpnt"], sizeof(mntpnt));
    strlcpy(user, doc["ntrip"]["user"], sizeof(user));
    strlcpy(passwd, doc["ntrip"]["passwd"], sizeof(passwd));
}

/**
 * @brief Save configuration to SPIFFS.
 */
void saveConfig() {
    DynamicJsonDocument doc(1024);
    doc["wifi"]["ssid"] = ssid;
    doc["wifi"]["password"] = password;
    doc["ntrip"]["host"] = host;
    doc["ntrip"]["port"] = httpPort;
    doc["ntrip"]["mntpnt"] = mntpnt;
    doc["ntrip"]["user"] = user;
    doc["ntrip"]["passwd"] = passwd;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
        Serial.println("  ERROR: Failed to open config file for writing");
        return;
    }

    serializeJson(doc, configFile);
}

/**
 * @brief Print the current configuration to the serial output.
 */
void printConfig() {
    Serial.println("Current Configuration:");
    Serial.print("WiFi SSID: ");
    Serial.println(ssid);
    Serial.print("WiFi Password: ");
    Serial.println(password);
    Serial.print("NTRIP Host: ");
    Serial.println(host);
    Serial.print("NTRIP Port: ");
    Serial.println(httpPort);
    Serial.print("NTRIP Mountpoint: ");
    Serial.println(mntpnt);
    Serial.print("NTRIP Username: ");
    Serial.println(user);
    Serial.print("NTRIP Password: ");
    Serial.println(passwd);
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
    WiFiManagerParameter custom_host("host", "NTRIP Host", host, sizeof(host));
    WiFiManagerParameter custom_port("port", "NTRIP Port", String(httpPort).c_str(), 6);
    WiFiManagerParameter custom_mntpnt("mntpnt", "NTRIP Mountpoint", mntpnt, sizeof(mntpnt));
    WiFiManagerParameter custom_user("user", "NTRIP Username", user, sizeof(user));
    WiFiManagerParameter custom_passwd("passwd", "NTRIP Password", passwd, sizeof(passwd));

    // Add custom parameters to WiFiManager
    wifiManager.addParameter(&custom_host);
    wifiManager.addParameter(&custom_port);
    wifiManager.addParameter(&custom_mntpnt);
    wifiManager.addParameter(&custom_user);
    wifiManager.addParameter(&custom_passwd);

    wifiManager.setSaveConfigCallback(saveConfigCallback);
    wifiManager.setBreakAfterConfig(true);

    // If configuration is empty, start configuration portal
    if (strlen(ssid) == 0 || strlen(password) == 0 || buttonPressed) {
        // wifiManager.startConfigPortal("NTRIPClient_Config");

        wifiManager.startConfigPortal("NTRIPClient_Config");
        wifiManager.stopConfigPortal();
        wifiManager.disconnect();
    } else {
        WiFi.begin(ssid, password);
        if ((WiFi.waitForConnectResult() != WL_CONNECTED)) {
            // wifiManager.autoConnect();
            wifiManager.startConfigPortal("NTRIPClient_Config");
        }
    }

    if (configMode){
        // Save the new configuration to globals
        Serial.println("   INFO: Storing data to SPIFFS");
        strlcpy(ssid, WiFi.SSID().c_str(), sizeof(ssid));
        strlcpy(password, WiFi.psk().c_str(), sizeof(password));
        strlcpy(host, custom_host.getValue(), sizeof(host));
        httpPort = atoi(custom_port.getValue());
        strlcpy(mntpnt, custom_mntpnt.getValue(), sizeof(mntpnt));
        strlcpy(user, custom_user.getValue(), sizeof(user));
        strlcpy(passwd, custom_passwd.getValue(), sizeof(passwd));
        // Save globals to SPIFFS
        saveConfig();
        ESP.restart();
    }
    #ifdef Debug
    printConfig();
    #endif
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
    if(ntrip_c.reqSrcTbl(host, httpPort)) {
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
    if(!ntrip_c.reqRaw(host, httpPort, mntpnt, user, passwd)) {
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
        if (ch == '\n') {
            ggaBuffer[ggaIndex] = '\0';
            if (strstr(ggaBuffer, "$GPGGA") != NULL) {
                ntrip_c.sendGGA(ggaBuffer);
            }
            ggaIndex = 0;
        } else {
            if (ggaIndex < sizeof(ggaBuffer) - 1) {
                ggaBuffer[ggaIndex++] = ch;
            }
        }
    }
}