#ifndef NTRIPCLIENTNMEAPARSER_H
#define NTRIPCLIENTNMEAPARSER_H

#include <cstring> // For strncpy, memcpy
#include <cstdlib> // For atof, atoi
#include <ArduinoJson.h> // For JSON handling

/**
 * @class NTRIPClientNmeaParser
 * @brief A class for parsing and managing NMEA sentences (GGA, RMC, VTG).
 */

// Define the buffer size for NMEA sentences
#define NMEA_BUFFER_SIZE 90

class NTRIPClientNmeaParser
{
public:
    /**
     * @brief Constructor for the NTRIPClientNmeaParser class.
     */
    NTRIPClientNmeaParser();

    /**
     * @brief Destructor for the NTRIPClientNmeaParser class.
     */
    ~NTRIPClientNmeaParser();

    /**
     * @brief Sets the GGA sentence, parses it, and updates the corresponding flag.
     * @param ggaSentence The GGA sentence to set.
     */
    void setGGA(const char* ggaSentence);

    /**
     * @brief Sets the RMC sentence and updates the corresponding flag.
     * @param rmcSentence The RMC sentence to set.
     */
    void setRMC(const char* rmcSentence);

    /**
     * @brief Sets the VTG sentence and updates the corresponding flag.
     * @param vtgSentence The VTG sentence to set.
     */
    void setVTG(const char* vtgSentence);

    /**
     * @brief Parses the GGA, RMC, and VTG messages if their received flags are set.
     * @return True if all sentences are successfully parsed, false otherwise.
     */
    bool parse();

    /**
     * @brief Generates a JSON string based on the parsed data.
     * @param[in, out] jsonBuffer A buffer to store the generated JSON string.
     * @param[in] bufferSize The size of the provided buffer.
     * @return True if the JSON string was successfully generated, false otherwise.
     */
    bool getJsonString(char* jsonBuffer, size_t bufferSize);

    /**
     * @brief Function to compose the message with Control-A, formatted timestamp, CRC-16, and Control-X.
     * @param[out] outputBuffer The buffer to store the composed message.
     * @param[in] outputLength The length of the composed message.
     */
    void composeTimeMessage(uint8_t* outputBuffer, size_t& outputLength);

private:
    /**
     * @brief Parses the GGA sentence and updates the parsed data.
     */
    void parseGGA(); // Function to parse the GGA sentence

    /**
     * @brief Parses the RMC sentence and updates the parsed data.
     */
    void parseRMC(); // Function to parse the RMC sentence

    /**
     * @brief Parses the VTG sentence and updates the parsed data.
     */
    void parseVTG(); // Function to parse the VTG sentence

    // Static buffers for storing NMEA sentences
    static char _ggaBuffer[NMEA_BUFFER_SIZE]; ///< Buffer to store GGA sentences
    static char _rmcBuffer[NMEA_BUFFER_SIZE]; ///< Buffer to store RMC sentences
    static char _vtgBuffer[NMEA_BUFFER_SIZE]; ///< Buffer to store VTG sentences

    // Flags to indicate if sentences are received
    bool _ggaReceived; ///< Flag to indicate if GGA sentence is received
    bool _rmcReceived; ///< Flag to indicate if RMC sentence is received
    bool _vtgReceived; ///< Flag to indicate if VTG sentence is received

    // Parsed NMEA data
    double _latitude;                  ///< Parsed latitude
    double _longitude;                 ///< Parsed longitude
    double _altitude;                  ///< Parsed altitude
    double _ageOfDifferentialData;     ///< Parsed age of differential data
    int _fixType;                      ///< Parsed fix type
    int _satellites;                   ///< Parsed number of satellites
    double _hdop;                      ///< Parsed HDOP
    int _year;                         ///< Parsed year
    int _month;                        ///< Parsed month
    int _day;                          ///< Parsed day
    int _hours;                        ///< Parsed hours
    int _minutes;                      ///< Parsed minutes
    int _seconds;                      ///< Parsed seconds
    int _milliseconds;                 ///< Parsed milliseconds
    double _speed;                     ///< Parsed speed
    double _direction;                 ///< Parsed direction

    // Additional data for JSON messages
    static int _sequenceNumber;        ///< Sequence number for JSON messages
    static unsigned long _timestamp;   ///< Timestamp for JSON messages
};

#endif // NTRIPCLIENTNMEAPARSER_H
