# Heltec WiFi LoRa 32 V3 firmware

A projekt ESP-IDF-alapú, az aktuális célplatform ESP32-S3. Fordítás az ESP-IDF környezetben: `idf.py build`. Feltöltés és soros monitor: `idf.py -p COMx flash monitor` (a COMx helyére a panel portja kerül).

A panel MASTER módban indul. A felhasználói gomb sorrendben a SLAVE, GFSK scan és LoRa scan módokra, majd vissza MASTER módra vált. A LED SLAVE módban világít. Az OLED a kapcsolat állapotát, TX/RX aktivitást, RSSI-t és csomagvesztést, illetve szkennelés közben az előrehaladást mutatja. A rádiós mérési eredmények a soros naplóban jelennek meg.

- [Kommunikációs specifikáció](kommunikacio.md)
- [Hardver és kezelőszervek](hardver.md)
- [SX1262 driver leírása](../components/SX1262/README.md)
