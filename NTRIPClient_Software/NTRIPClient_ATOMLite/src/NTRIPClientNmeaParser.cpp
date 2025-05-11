#include "NTRIPClientNmeaParser.h"
#include "NTRIPClientCRC16.h" // Include the external CRC-16 function

char NTRIPClientNmeaParser::_ggaBuffer[NMEA_BUFFER_SIZE] = {'\0'};
char NTRIPClientNmeaParser::_rmcBuffer[NMEA_BUFFER_SIZE] = {'\0'};
char NTRIPClientNmeaParser::_vtgBuffer[NMEA_BUFFER_SIZE] = {'\0'};
int NTRIPClientNmeaParser::_sequenceNumber = 0;
unsigned long NTRIPClientNmeaParser::_timestamp = 0;

NTRIPClientNmeaParser::NTRIPClientNmeaParser()
    : _ggaReceived(false), _rmcReceived(false), _vtgReceived(false),
      _latitude(0.0), _longitude(0.0), _altitude(0.0), _ageOfDifferentialData(0.0),
      _fixType(0), _satellites(0), _hdop(0.0),
      _year(2025), _month(1), _day(1), _hours(0), _minutes(0), _seconds(0), _milliseconds(0),
      _speed(0.0), _direction(0.0)
{
}

NTRIPClientNmeaParser::~NTRIPClientNmeaParser()
{
}

bool NTRIPClientNmeaParser::parse()
{
    // Check if all sentences have been received
    if (_ggaReceived && _rmcReceived && _vtgReceived) {
        // Parse each sentence
        parseGGA();
        parseRMC();
        parseVTG();

        // Reset the received flags
        _ggaReceived = false;
        // _rmcReceived = false;
        _vtgReceived = false;

        return true; // Parsing successful
    }

    return false; // Parsing not successful
}

bool NTRIPClientNmeaParser::getJsonString(char* jsonBuffer, size_t bufferSize)
{
    if (!jsonBuffer || bufferSize == 0) {
        return false; // Invalid buffer
    }

    DynamicJsonDocument doc(1024);
    doc["id"] = _sequenceNumber++;
    doc["daytime"] = _timestamp;
    doc["lat"] = _latitude;
    doc["lon"] = _longitude;
    doc["alt"] = _altitude;
    doc["fix_type"] = _fixType;
    doc["speed"] = _speed;
    doc["dir"] = _direction;
    doc["sats"] = _satellites;
    doc["hdop"] = _hdop;
    doc["age"] = _ageOfDifferentialData;

    // Serialize JSON to the provided buffer
    size_t jsonLength = serializeJson(doc, jsonBuffer, bufferSize);
    return jsonLength > 0;
}

void NTRIPClientNmeaParser::setGGA(const char* ggaSentence)
{
    strncpy(_ggaBuffer, ggaSentence, sizeof(_ggaBuffer) - 1);
    _ggaBuffer[sizeof(_ggaBuffer) - 1] = '\0'; // Ensure null termination
    _ggaReceived = true; // Set the flag
}

void NTRIPClientNmeaParser::parseGGA()
{
    char* token = strtok(_ggaBuffer, ",");
    int fieldIndex = 0;
    char timeBuffer[11] = {0};
    char latDirection = '\0';
    char lonDirection = '\0';

    while (token != NULL) {
        switch (fieldIndex) {
            case 1: // Time
                strncpy(timeBuffer, token, sizeof(timeBuffer) - 1);
                break;
            case 2: // Latitude
                _latitude = atof(token);
                break;
            case 3: // Latitude direction (N/S)
                latDirection = token[0];
                break;
            case 4: // Longitude
                _longitude = atof(token);
                break;
            case 5: // Longitude direction (E/W)
                lonDirection = token[0];
                break;
            case 6: // Fix type
                _fixType = atoi(token);
                break;
            case 7: // Satellites
                _satellites = atoi(token);
                break;
            case 8: // HDOP
                _hdop = atof(token);
                break;
            case 9: // Altitude
                _altitude = atof(token);
                break;
            case 13: // Age of Differential Data
                _ageOfDifferentialData = atof(token);
                break;
        }
        token = strtok(NULL, ",");
        fieldIndex++;
    }

    // Parse time from GGA sentence
    _hours = (timeBuffer[0] - '0') * 10 + (timeBuffer[1] - '0');
    _minutes = (timeBuffer[2] - '0') * 10 + (timeBuffer[3] - '0');
    _seconds = (timeBuffer[4] - '0') * 10 + (timeBuffer[5] - '0');

    // Check if milliseconds part has 3 or 2 positions
    _milliseconds = 0;
    if (strlen(timeBuffer) > 7) {
        if (strlen(&timeBuffer[7]) == 3) { // 3 positions for milliseconds
            _milliseconds = (timeBuffer[7] - '0') * 100 + (timeBuffer[8] - '0') * 10 + (timeBuffer[9] - '0');
        } else if (strlen(&timeBuffer[7]) == 2) { // 2 positions for milliseconds
            _milliseconds = (timeBuffer[7] - '0') * 100 + (timeBuffer[8] - '0') * 10;
        }
    }

    // Convert latitude and longitude to decimal degrees
    int latDegrees = (int)(_latitude / 100);
    double latMinutes = _latitude - (latDegrees * 100);
    _latitude = latDegrees + (latMinutes / 60.0);
    if (latDirection == 'S') {
        _latitude = -_latitude; // South is negative
    }

    int lonDegrees = (int)(_longitude / 100);
    double lonMinutes = _longitude - (lonDegrees * 100);
    _longitude = lonDegrees + (lonMinutes / 60.0);
    if (lonDirection == 'W') {
        _longitude = -_longitude; // West is negative
    }
}

void NTRIPClientNmeaParser::setRMC(const char* rmcSentence)
{
    strncpy(_rmcBuffer, rmcSentence, sizeof(_rmcBuffer) - 1);
    _rmcBuffer[sizeof(_rmcBuffer) - 1] = '\0'; // Ensure null termination
    _rmcReceived = true; // Set the flag
}

void NTRIPClientNmeaParser::parseRMC()
{
    char* token = strtok(_rmcBuffer, ",");
    int fieldIndex = 0;
    char dateBuffer[7] = {0};

    while (token != NULL) {
        switch (fieldIndex) {
            case 9: // Date
                strncpy(dateBuffer, token, sizeof(dateBuffer) - 1);
                break;
        }
        token = strtok(NULL, ",");
        fieldIndex++;
    }

    if (strlen(dateBuffer) == 6) {
        _day = (dateBuffer[0] - '0') * 10 + (dateBuffer[1] - '0');
        _month = (dateBuffer[2] - '0') * 10 + (dateBuffer[3] - '0');
        _year = 2000 + (dateBuffer[4] - '0') * 10 + (dateBuffer[5] - '0');
    }
}

// Setter for VTG sentence
void NTRIPClientNmeaParser::setVTG(const char* vtgSentence)
{
    strncpy(_vtgBuffer, vtgSentence, sizeof(_vtgBuffer) - 1);
    _vtgBuffer[sizeof(_vtgBuffer) - 1] = '\0'; // Ensure null termination
    _vtgReceived = true; // Set the flag
}

void NTRIPClientNmeaParser::parseVTG()
{
    char* token = strtok(_vtgBuffer, ",");
    int fieldIndex = 0;
    char* previousToken = nullptr; // To store the previous token

    while (token != NULL) {
        // Extract speed from VTG sentence
        if (strcmp(token, "K") == 0 && previousToken != nullptr) {
            _speed = atof(previousToken) / 3.6; // Convert speed from km/h to m/s
        }

        // Retrieve direction only if it is "T" (true north)
        switch (fieldIndex) {
            case 1: // Direction (true north)
                _direction = atof(token);
                break;
            case 2: // Validate 'T' field
                if (strcmp(token, "T") != 0) {
                    _direction = 0.0; // Invalid direction
                }
                break;
        }

        // Update the previous token and move to the next token
        previousToken = token;
        token = strtok(NULL, ",");
        fieldIndex++;
    }
}

void NTRIPClientNmeaParser::composeTimeMessage(uint8_t* outputBuffer, size_t& outputLength)
{
    const uint8_t controlA = 0x01; // Control-A
    const uint8_t controlX = 0x18; // Control-X

    // Format timestamp
    char timestamp[24];
    snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d.%03d",
             _year, _month, _day, 12, 34, 56, 789); // Replace with actual time variables if available

    // Start with Control-A
    size_t index = 0;
    outputBuffer[index++] = controlA;

    // Append the timestamp
    size_t timestampLength = strlen(timestamp);
    memcpy(&outputBuffer[index], timestamp, timestampLength);
    index += timestampLength;

    // Calculate CRC-16 using the external function
    uint16_t crc = calculateCRC16((const uint8_t*)timestamp, timestampLength);

    // Append the CRC-16 (big-endian)
    outputBuffer[index++] = (crc >> 8) & 0xFF; // High byte
    outputBuffer[index++] = crc & 0xFF;        // Low byte

    // Append Control-X
    outputBuffer[index++] = controlX;

    // Set the output length
    outputLength = index;
}
