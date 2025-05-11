/*!
 * \file harwareConfig.h
 * \brief Header file with hardware configuration and settings for ATOM-Lite ESP32 board
 *
 * \author Remko Welling (remko.welling@han.nl)
 * \date 11-5-2025
 *
 * 
 */

#ifndef HARDWARECONFIG_H
#define HARDWARECONFIG_H

// Define brightness level (10% of maximum brightness)
#define LED_BRIGHTNESS 25   ///< Brightness level (0-255, where 255 is 100%)

// NeoPixel LED configuration
#define LED_PIN 27          ///< GPIO 38 for the onboard WS2812 RGB LED
#define NUMPIXELS 1         ///< Number of NeoPixels

// Button configuration
#define BUTTON_PIN 39       ///< GPIO for button input

// Second serial interface for NTRIPClient
#define TXD2_PIN 21         ///< Corrected to GPIO17 for UART2 TX
#define RXD2_PIN 22         ///< Corrected to GPIO18 for UART2 RX
// GPIO Serial1 (Telemetry unit)
#define TXD1_PIN 19         ///< Corrected to GPIO19 for UART1 TX
#define RXD1_PIN 23         ///< Corrected to GPIO20 for UART1 RX

#define WIFI_LED 33         //</ GPIO9 for WiFi LED
#define NTRIP_LED 33        ///< GPIO10 for NTRIP LED
#define MQTT_LED 33         ///< GPIO11 for MQTT LED
#define FIX_RTKFLOAT_LED 33 ///< GPIO12 for FIX RTK-FLOAT LED
#define FIX_RTK_LED 33  

#define BUTTON_PRESS_TIME_MS 3000   ///< Time in milliseconds to hold button to enter configuration mode

#define LED_OFF_TIME_MS 100         ///< Time in milliseconds to turn off LED after last data received
#define NTRIP_TIMEOUT_MS 60000      ///< Time in milliseconds to reset system if no NTRIP data is received

const unsigned long GGA_SEND_INTERVAL = 300000; ///< 5 minutes in milliseconds

#endif // HARDWARECONFIG_H
