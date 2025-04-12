// --------------------------------------------------------------------
//   This file is part of the PE1MEW NTRIP Client.
//
//   The NTRIP Client is distributed in the hope that 
//   it will be useful, but WITHOUT ANY WARRANTY; without even the 
//   implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
//   PURPOSE.
// --------------------------------------------------------------------*/

/*!
   
 \file NTRIPClient.h
 \brief NTRIPClient class definition
 \author Remko Welling (PE1MEW) 

 This file was retrieved from the following work:
  - [NTRIP-client-for-Arduino](https://github.com/GLAY-AK2/NTRIP-client-for-Arduino)

 The file was refactored to be used in the PE1MEW NTRIP Client project.

*/

#ifndef NTRIP_CLIENT
#define NTRIP_CLIENT

#include <WiFiClient.h>
#include <Arduino.h>
#include <base64.h>

/**
 * @class NTRIPClient
 * @brief A client for NTRIP (Networked Transport of RTCM via Internet Protocol).
 * 
 * This class extends WiFiClient to provide functionality for requesting
 * MountPoints List and RAW data from an NTRIP Caster.
 */
class NTRIPClient : public WiFiClient {
public:
    /**
     * @brief Request the MountPoints List serviced by the NTRIP Caster.
     * 
     * @param host The hostname of the NTRIP Caster.
     * @param port The port number of the NTRIP Caster.
     * @return true if the request was successful, false otherwise.
     */
    bool reqSrcTbl(const char* host, int &port);

    /**
     * @brief Request RAW data from the NTRIP Caster with user authentication.
     * 
     * @param host The hostname of the NTRIP Caster.
     * @param port The port number of the NTRIP Caster.
     * @param mntpnt The MountPoint to request data from.
     * @param user The username for authentication.
     * @param psw The password for authentication.
     * @return true if the request was successful, false otherwise.
     */
    bool reqRaw(const char* host, int &port, const char* mntpnt, const char* user, const char* psw);

    /**
     * @brief Request RAW data from the NTRIP Caster without user authentication.
     * 
     * @param host The hostname of the NTRIP Caster.
     * @param port The port number of the NTRIP Caster.
     * @param mntpnt The MountPoint to request data from.
     * @return true if the request was successful, false otherwise.
     */
    bool reqRaw(const char* host, int &port, const char* mntpnt);

    /**
     * @brief Read a line of data from the NTRIP Caster.
     * 
     * @param buffer The buffer to store the read data.
     * @param size The size of the buffer.
     * @return The number of bytes read.
     */
    int readLine(char* buffer, int size);

    /**
     * @brief Send a GGA sentence to the NTRIP Caster.
     * 
     * @param gga The GGA sentence to send.
     */
    void sendGGA(const char* gga);
};

#endif
