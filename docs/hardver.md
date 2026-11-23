# Hardver és kezelőszervek

| Egység | Firmware-ben használt bekötés |
| --- | --- |
| SX1262 SPI és vezérlés | NSS GPIO8, SCK GPIO9, MOSI GPIO10, MISO GPIO11, RESET GPIO12, BUSY GPIO13, DIO1 GPIO14 |
| SSD1306 OLED | SDA GPIO17, SCL GPIO18, RESET GPIO21, VEXT GPIO36 |
| Felhasználói gomb | GPIO0, belső felhúzás, aktív alacsony |
| Felhasználói LED | GPIO35 |
| Akkumulátormérés | GPIO37 vezérlés, GPIO1 (ADC_CHANNEL_0), 0 dB csillapítás |

Az OLED 128×64 képpontos. Az I²C cím 0x3C, az órajel 100 kHz. A gomb mintavétele 10 ms, a pergésmentesítés 30 ms. A nyitóképernyő időzítése 5 s, a TX/RX visszajelzés 40 ms, a kijelzőfrissítési kísérletek minimális köze 30 ms. Az akkumulátor mérési periódusa 2 s, a szkennelés alatt a mérés szünetel.

A jelenlegi buildben az USB-only mód ki van kapcsolva, az akkumulátormérés vezérlése aktív alacsony (V3.0/V3.1). V3.2-höz a `CONFIG_HELTEC_BATTERY_ADC_ACTIVE_HIGH` opció kapcsolható be. Akkumulátor nélküli üzemhez a `CONFIG_HELTEC_USB_ONLY` letiltja a mérést.
