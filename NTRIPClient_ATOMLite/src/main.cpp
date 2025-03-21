// #define Debug  // Add this line to enable debug information

/*
 *  NTRIP client for Arduino Ver. 1.0.0 
 *  NTRIPClient Sample
 *  Request Source Table (Source Table is basestation list in NTRIP Caster)
 *  Request Reference Data 
 * 
 * 
 */
//#include <ESP8266WiFi.h>  //Need for ESP8266
#include <WiFi.h>           //Need for ESP32 
#include "NTRIPClient.h"
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>

// NeoPixel LED configuration
#define LED_PIN 27
#define NUMPIXELS 1

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

void setup() {
    // Initialize serial communication
    Serial.begin(115200); // Default USB Serial (UART0)
    delay(10);

    // Iniotialize Serial2 for NTRIPClient
    Serial2.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN); // UART2 on GPIO21/22
    delay(10);

    // Initialize NeoPixel
    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red for WiFi disconnected
    pixels.show();

    // Load configuration
    loadConfig();

    // Connect to WiFi
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
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

void loop() {
    // Continuously read and print data from NTRIP server
    while(ntrip_c.available()) {
        char ch = ntrip_c.read();        
        Serial2.print(ch);
    }

    // Read and send GGA sentences
    static char ggaBuffer[256];
    static int ggaIndex = 0;

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