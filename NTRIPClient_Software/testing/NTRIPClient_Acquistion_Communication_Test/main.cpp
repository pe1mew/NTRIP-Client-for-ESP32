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

TEST_CASE("calculateCRC16 computes correct CRC-16 checksum", "[CRC16]") {
    SECTION("CRC-16 for ASCII '12345'") {
        const uint8_t data[] = {0x31, 0x32, 0x33, 0x34, 0x35}; ///< HEX for ASCII for "12345"
        size_t length = sizeof(data) / sizeof(data[0]);
        uint16_t expectedCRC = 0x4560; ///< Expected CRC value for "12345"
        REQUIRE(calculateCRC16(data, length) == expectedCRC);
    }

    SECTION("CRC-16 for empty data") {
        const uint8_t data[] = {};  ///< Empty data
        size_t length = 0;
        uint16_t expectedCRC = 0xFFFF; //< Expected CRC value for empty data
        REQUIRE(calculateCRC16(data, length) == expectedCRC);
    }

    SECTION("CRC-16 for single byte") {
        const uint8_t data[] = {0x01}; ///< Single byte data
        size_t length = 1;
        uint16_t expectedCRC =  0xF1D1; ///< Expected CRC value for single byte 0x01
        REQUIRE(calculateCRC16(data, length) == expectedCRC);
    }
}

TEST_CASE("CRC-16 for daytime message", "[CRC16]") {
    SECTION("CRC-16 for ASCII '2025-03-30 10:27:06.500'") {
        const uint8_t data[] = {0x32, 0x30, 0x32, 0x35, 0x2D, 0x30, 0x33, 0x2D,
                                0x33, 0x30, 0x20, 0x31, 0x30, 0x3A, 0x32, 0x37,
                                0x3A, 0x30, 0x36, 0x2E, 0x35, 0x30, 0x30}; ///< HEX for ASCII "2025-03-30 10:27:06.500"
        size_t length = sizeof(data) / sizeof(data[0]);
        uint16_t expectedCRC = 0x4597; ///< Expected CRC value for "2025-03-30 10:27:06.500"
        REQUIRE(calculateCRC16(data, length) == expectedCRC);
    }
}
