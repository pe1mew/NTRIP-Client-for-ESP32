// --------------------------------------------------------------------
//   This file is part of the PE1MEW NTRIP Client.
//
//   The NTRIP Client is distributed in the hope that 
//   it will be useful, but WITHOUT ANY WARRANTY; without even the 
//   implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
//   PURPOSE.
// --------------------------------------------------------------------*/

/*!
   
 \file NTRIPClient.cpp
 \brief NTRIPClient class implementation
 \author Remko Welling (PE1MEW) 

 This file was retrieved from the following work:
  - [NTRIP-client-for-Arduino](https://github.com/GLAY-AK2/NTRIP-client-for-Arduino)

 The file was refactored to be used in the PE1MEW NTRIP Client project.

*/

#include "NTRIPClient.h"

bool NTRIPClient::reqSrcTbl(const char* host, int &port) {
    if (!connect(host, port)) {
        Serial.print("Cannot connect to ");
        Serial.println(host);
        return false;
    }
    print(
        "GET / HTTP/1.0\r\n"
        "User-Agent: NTRIPClient for Arduino v1.0\r\n"
    );
    
    unsigned long timeout = millis();
    
    while (available() == 0) {
        if (millis() - timeout > 5000) {
            Serial.println("Client Timeout !");
            stop();
            return false;
        }
        delay(10);
    }
    
    char buffer[50];
    readLine(buffer, sizeof(buffer));
    
    if (strncmp((char*)buffer, "SOURCETABLE 200 OK", 17)) {
        Serial.print((char*)buffer);
        return false;
    } else {
        return true;
    }
}

bool NTRIPClient::reqRaw(const char* host, int &port, const char* mntpnt, const char* user, const char* psw) {
    if (!connect(host, port)){
        return false;
    }
    
    String p = "GET /";
    String auth = "";
    
    Serial.println("Request NTRIP");

    p = p + mntpnt + String(" HTTP/1.0\r\n"
                            "User-Agent: NTRIPClient for Arduino v1.0\r\n");

    if (strlen(user) == 0) {
        p = p + String(
            "Accept: */*\r\n"
            "Connection: close\r\n"
        );
    } else {
        auth = base64::encode(String(user) + String(":") + psw);
        #ifdef Debug
        Serial.println(String(user) + String(":") + psw);
        #endif

        p = p + String("Authorization: Basic ");
        p = p + auth;
        p = p + String("\r\n");
    }

    p = p + String("\r\n");
    print(p);

    #ifdef Debug
    Serial.println(p);
    #endif
    
    unsigned long timeout = millis();
    
    while (available() == 0) {
        if (millis() - timeout > 20000) {
            Serial.println("Client Timeout !");
            return false;
        }
        delay(10);
    }
    char buffer[50];
    readLine(buffer, sizeof(buffer));
    if (strncmp((char*)buffer, "ICY 200 OK", 10)) {
        Serial.print((char*)buffer);
        return false;
    } else {
        return true;
    }
}

bool NTRIPClient::reqRaw(const char* host, int &port, const char* mntpnt) {
    return reqRaw(host, port, mntpnt, "", "");
}

int NTRIPClient::readLine(char* _buffer, int size) {
    int len = 0;
    
    while (available()) {
        _buffer[len] = read();
        len++;
        if (_buffer[len - 1] == '\n' || len >= size) break;
    }

    _buffer[len] = '\0';

    return len;
}

void NTRIPClient::sendGGA(const char* gga) {
    if (!connected()) {
        Serial.println("Not connected to NTRIP Caster");
        return;
    }

    String ggaString = String(gga) + "\r\n";
    print(ggaString);

    #ifdef Debug
    Serial.println("Sent GGA: " + ggaString);
    #endif
}
