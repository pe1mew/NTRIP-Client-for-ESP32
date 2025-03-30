/*! 
 * @file main.cpp
 * @brief Main file for testing CRC-16 checksum calculation.
 * @details This file contains unit tests for the CRC-16 checksum calculation function.
 * @author Remko Welling (remko.welling@han.nl)
 * @date 30-3-2025
 * @note This implementation is based on the CRC-16-CCITT (XMODEM) algorithm.
 * 
 * ## Test Cases:
 * 
 * ### Assertions: 
 * 
 * 1. **CRC-16 for ASCII '12345'**: This test verifies the correctness of the CRC calculation for a common string.
 *    The data is represented in ASCII format, and the expected CRC is precomputed.
 * 
 * 2. **CRC-16 for empty data**: This test ensures that the function correctly handles an empty input.
 *    The expected CRC value for an empty dataset is the initial value of the CRC-16-CCITT algorithm (0xFFFF).
 * 
 * 3. **CRC-16 for a single byte**: This test validates the function's behavior for minimal input.
 *    A single byte (0x01) is used to confirm that the function properly computes the checksum for very short data.
 * 
 * 4. **CRC-16 for daytime message**: This test checks the CRC calculation for a specific datetime string.
 *    The string "2025-03-30 10:27:06.500" is used, and the expected CRC is computed.
 */

#define CATCH_CONFIG_MAIN // This defines the main() function for Catch2

#include "../catch2/catch.hpp"
#include "NTRIPClientCRC16.h"

/*! 
 * @struct TestCase
 * @brief Represents a single test case for CRC-16 checksum calculation.
 * 
 * This structure is used to define the input data, expected output, and description
 * for each test case in the unit tests for the CRC-16 checksum calculation function.
 * 
 * @var TestCase::data
 * Pointer to the input data for the CRC-16 calculation.
 * 
 * @var TestCase::length
 * The length of the input data in bytes.
 * 
 * @var TestCase::expectedCRC
 * The expected CRC-16 checksum value for the given input data.
 * 
 * @var TestCase::description
 * A brief description of the test case, used for documentation and debugging purposes.
 */
struct TestCase {
    const uint8_t* data;      /*!< Pointer to the input data for the CRC-16 calculation. */
    size_t length;            /*!< Length of the input data in bytes. */
    uint16_t expectedCRC;     /*!< Expected CRC-16 checksum value. */
    const char* description;  /*!< Description of the test case. */
};

TEST_CASE("calculateCRC16 computes correct CRC-16 checksum", "[CRC16]") {
    TestCase testCases[] = {
        {reinterpret_cast<const uint8_t*>("\x31\x32\x33\x34\x35"), 5, 0x4560, "CRC-16 for ASCII '12345'"},
        {reinterpret_cast<const uint8_t*>(""), 0, 0xFFFF, "CRC-16 for empty data"},
        {reinterpret_cast<const uint8_t*>("\x01"), 1, 0xF1D1, "CRC-16 for single byte"}
    };

    for (const auto& testCase : testCases) {
        SECTION(testCase.description) {
            REQUIRE(calculateCRC16(testCase.data, testCase.length) == testCase.expectedCRC);
        }
    }
}

TEST_CASE("CRC-16 for daytime message", "[CRC16]") {
    TestCase testCases[] = {
        {reinterpret_cast<const uint8_t*>("\x32\x30\x32\x35\x2D\x30\x33\x2D\x33\x30\x20\x31\x30\x3A\x32\x37\x3A\x30\x36\x2E\x35\x30\x30"), 
         23, 0x4597, "CRC-16 for ASCII '2025-03-30 10:27:06.500'"}
    };

    for (const auto& testCase : testCases) {
        SECTION(testCase.description) {
            REQUIRE(calculateCRC16(testCase.data, testCase.length) == testCase.expectedCRC);
        }
    }
}
