# Key Pin Assignments for LOLIN S3:

UART (Serial Communication):
 - UART0 (Default Serial):
   - TX: GPIO1
   - RX: GPIO3
 - UART1:
   - TX: GPIO19
   - RX: GPIO20
 - UART2:
   - TX: GPIO17
   - RX: GPIO18

I2C (Default):
 - SDA: GPIO8
 - SCL: GPIO9

SPI (Default):
 - SCK:  GPIO36
 - MISO: GPIO37
 - MOSI: GPIO35
 - CS:   GPIO34

WS2812 RGB LED:
 - Data Pin: GPIO38 (Onboard WS2812 RGB LED)

Buttons:
 - BOOT Button: GPIO0 (Can be used as an input)

Analog Pins (ADC):
 - ADC1: GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO7
 - ADC2: GPIO10, GPIO11, GPIO12, GPIO13, GPIO14, GPIO15

PWM (LED Control):
 - Any GPIO pin can be used for PWM output.

Power Pins:
 - 5V: 5V pin
 - 3.3V: 3.3V pin
 - GND: Ground pins

Other Pins:
 - GPIO38: Onboard WS2812 RGB LED
 - GPIO39: Input-only pin (can be used for buttons or sensors)