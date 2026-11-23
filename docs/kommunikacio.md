# Kommunikációs specifikáció

A táblázatok a firmware jelenlegi értékeit rögzítik. Ha átírsz egy értéket, kérd a Codextől: „Vezesd át a docs/kommunikacio.md módosításait a firmware-be, és ellenőrizd a buildet.” Az [AGENTS.md](../AGENTS.md) rögzíti ezt a munkamenetet.

## MASTER–SLAVE rádiós kapcsolat

| Paraméter | Érték | Implementáció |
| --- | --- | --- |
| Moduláció | GFSK | [radio_link.c](../main/radio_link.c), [sx1262.c](../components/SX1262/sx1262.c) |
| Frekvencia | 869 525 000 Hz | `CONFIG_LINK_FREQUENCY_HZ` |
| Adóteljesítmény | 14 dBm | `CONFIG_LINK_TX_POWER_DBM` |
| Bitsebesség és frekvencialöket | 100 000 bit/s, ±25 000 Hz | `sx1262_default_gfsk_config` |
| Szűrés és RX sávszélesség | Gauss BT 0,5, 187,2 kHz | `configure_gfsk` |
| Preambulum és detektor | 32 bit, 16 bites detektor | `sx1262_default_gfsk_config`, `packet_params` |
| Szinkronszó | 32 bit, `2D D4 B7 E1` | `sx1262_default_gfsk_config` |
| Cím szerinti szűrés | Kikapcsolva | `packet_params` |
| CRC | CRC-16/CCITT-FALSE, kezdőérték 0xFFFF, polinom 0x1021 | `configure_gfsk` |
| Whitening | Bekapcsolva, seed 0x1FF | `sx1262_default_gfsk_config`, `packet_params` |
| Csomaghossz | Változó hosszúság, driverlimit 64 bájt, alkalmazáskeret 51 bájt | `SX1262_GFSK_DEFAULT_MAX_PAYLOAD_LENGTH`, `FRAME_LENGTH` |

A keret első 50 bájtja véletlen adat, az utolsó bájt a 8 bites számláló. A MASTER páros számlálót küld, a SLAVE a kapott értékhez egyet ad, és a páratlan értékkel válaszol. A MASTER a küldött értékhez képest pontosan eggyel nagyobb választ vár, 8 bites körbefordulással.

| Időzítés | Érték | Konstans a radio_link.c fájlban |
| --- | --- | --- |
| MASTER-ciklus | 20 ms | `CYCLE_MS` |
| Vételi ablak | 22 ms, a ciklus 110%-a felfelé kerekítve | `RX_WINDOW_MS` |
| Kapcsolat elévülése | 60 ms, három ciklus | `LINK_EXPIRY_MS` |
| Vételi lekérdezés időkorlátja | 2 ms | `RX_SLICE_MS` |
| Adás időkorlátja | 12 ms | `TX_TIMEOUT_MS` |
| SLAVE válaszidőzítés | 2 000 µs | `SLAVE_TURNAROUND_US` |
| RSSI átlagolási ablak | 1 000 ms | `RSSI_AVERAGE_MS` |

## Csatornaszkennelés

| Mód | Frekvenciák (MHz), szkennelési sorrendben |
| --- | --- |
| GFSK | 868,100 · 868,300 · 868,500 · 868,700 · 868,900 · 869,425 · 869,525 · 869,625 |
| LoRa | 867,100 · 867,300 · 867,500 · 867,700 · 867,900 · 868,100 · 868,300 · 868,500 |

A szkennelés csatornánként 6 s, névleg összesen 48 s. A beállási idő 20 ms, az RSSI-mintavétel 50 ms, az aktivitási küszöb −95 dBm (a küszöbértéket elérő minták is aktívnak számítanak). Forrás: [radio_scan.c](../main/radio_scan.c).

A GFSK scan a driver GFSK alapbeállításait használja. A LoRa scan beállításai 125 kHz, SF7, 4/5, 8 preambulumszimbólum, 0x1424 szinkronszó, bekapcsolt CRC, explicit fejléc és normál IQ.

## Soros napló

| Paraméter | Érték | Implementáció |
| --- | --- | --- |
| Elsődleges konzol | UART0, 115 200 baud | `CONFIG_ESP_CONSOLE_UART_BAUDRATE` |
| Másodlagos konzol | USB Serial/JTAG | `CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG` |
| Teljes rádiókeret hexadecimális naplózása | Kikapcsolva | `CONFIG_LINK_LOG_PAYLOAD` |
| Rádiós naplósor mélysége | 32 bejegyzés | `UART_LOG_QUEUE_DEPTH` |

A frekvencia, TX teljesítmény és keretnaplózás konfigurációs opcióit a [Kconfig.projbuild](../main/Kconfig.projbuild) definiálja. A használt build értékeit a `sdkconfig` tartalmazza. A rádiós beállítások módosításakor ellenőrizd a meglévő buildet és az új buildhez használt alapértékeket is.
