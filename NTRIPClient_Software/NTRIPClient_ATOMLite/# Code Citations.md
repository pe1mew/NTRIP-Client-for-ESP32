# Code Citations

## License: LGPL_3_0
https://github.com/GLAY-AK2/NTRIP-client-for-Arduino/tree/875298f5fa1c4ec84929b0c54364f77d5df7e081/src/NTRIPClient.cpp

```
0\r\n"
        "User-Agent: NTRIPClient for Arduino v1.0\r\n"
    );
    unsigned long timeout = millis();
    while (available() == 0) {
        if (millis() - timeout > 5000) {
            Serial.
```


## License: unknown
https://github.com/heyno/arduino/tree/143c22816d5a8fa4521b3c6ef67941aee86f4876/libraries/NTRIP-client-for-Arduino-master/src/NTRIPClient.cpp

```
Arduino v1.0\r\n"
    );
    unsigned long timeout = millis();
    while (available() == 0) {
        if (millis() - timeout > 5000) {
            Serial.println("Client Timeout !");
            stop();
```

