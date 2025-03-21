// #define Debug  // Add this line to enable debug information

/*
 *  NTRIP client for Arduino Ver. 1.0.0 
 *  NTRIPClient Sample
 *  Request Source Table (Source Table is basestation list in NTRIP Caster)
 *  Request Reference Data 
 * 
 * 
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

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

NTRIPClient ntrip_c;

// Configuration variables
char ssid[32];      ///< WiFi SSID
char password[32];  ///< WiFi password
char host[32];      ///< NTRIP server host
int httpPort;       ///< NTRIP server port
char mntpnt[32];    ///< NTRIP mountpoint
char user[32];      ///< NTRIP username
char passwd[32];    ///< NTRIP password

// For reading and sending GGA sentences
static char ggaBuffer[256];
static int ggaIndex = 0;

// Flag to track receiving state
#define LED_OFF_TIME_MS 500

bool isReceiving = false;
unsigned long lastReceiveTime = 0;

// Button press handling
bool buttonPressed = false;
unsigned long buttonPressTime = 0;
bool configMode = false;

/**
 * @brief Load configuration from SPIFFS.
 */
void loadConfig() {
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount file system");
        return;
    }

    File configFile = SPIFFS.open("/config.json", "r");
    if (!configFile) {
        Serial.println("Failed to open config file");
        return;
    }

    size_t size = configFile.size();
    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, buf.get());
    if (error) {
        Serial.println("Failed to parse config file");
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
        Serial.println("Failed to open config file for writing");
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
    if ((strlen(ssid) == 0 || strlen(password) == 0)) {
        wifiManager.startConfigPortal("NTRIPClient_Config");
    } else {
        WiFi.begin(ssid, password);
        if ((WiFi.waitForConnectResult() != WL_CONNECTED) || buttonPressed) {
            wifiManager.startConfigPortal("NTRIPClient_Config");
        }
    }

    if (configMode){
        // Save the new configuration to globals
        Serial.println("Storing data to SPIFFS");
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
    printConfig();
}

/**
 * @brief Setup function to initialize the system.
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200); // Default USB Serial (UART0)
    delay(10);

    // Initialize Serial2 for NTRIPClient
    Serial2.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN); // UART2 on GPIO21/22
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
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green for WiFi connected
    pixels.show();
    
    // Request SourceTable from NTRIP server
    Serial.println("Requesting SourceTable.");
    if(ntrip_c.reqSrcTbl(host, httpPort)) {
        char buffer[512];
        delay(5);
        while(ntrip_c.available()) {
            ntrip_c.readLine(buffer, sizeof(buffer));
            Serial.print(buffer); 
        }
    } else {
        Serial.println("SourceTable request error");
    }
    Serial.print("Requesting SourceTable is OK\n");
    ntrip_c.stop(); // Need to call "stop" function for next request.
    
    // Request raw data from MountPoint
    Serial.println("Requesting MountPoint's Raw data");
    if(!ntrip_c.reqRaw(host, httpPort, mntpnt, user, passwd)) {
        delay(15000);
        ESP.restart();
    }
    Serial.println("Requesting MountPoint is OK");
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
        } else if (millis() - buttonPressTime > 3000) { // 3 seconds press
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