# SX1262 radio driver

Native ESP-IDF component for the Heltec WiFi LoRa 32 V3.1. Add `SX1262` to
the calling component's `PRIV_REQUIRES` and include `sx1262.h`.

Use `sx1262_default_config(frequency_hz)` for LoRa or
`sx1262_default_gfsk_config(frequency_hz)` for GFSK. Adjust the returned
configuration before calling `sx1262_init()`. The MASTER/SLAVE application
uses GFSK at 869.525 MHz and sets the transmit power to +22 dBm by default
under **GFSK radio link** in ESP-IDF configuration.

The GFSK profile uses 50 kbit/s, ±25 kHz deviation, BT 0.5, 117.3 kHz
receive bandwidth, a 64-bit preamble, a 32-bit detector and sync word
`2D D4 B7 E1`. The application configures a fixed 53-byte frame:
50 data bytes, one Alive CNT byte, and a two-byte big-endian
CRC-16/CCITT-FALSE over the preceding 51 bytes. The packet engine also adds
hardware CRC-16 and uses whitening seed `0x1FF`.

MASTER sends a fresh random payload every 100 ms with an even counter.
SLAVE replies with the bitwise inverse and the matching odd counter.
Counters wrap at 256. MASTER validates the CRC, counter, and inverted
payload. SLAVE validates the CRC, parity, and sequence. A valid receive
sets LINK UP. After 300 ms without a valid receive the link drops.
The first valid exchange in each mode clears MISS. SLAVE counts a missed
frame for each 150 ms window without a valid MASTER frame and reconciles
those timeout counts with subsequent Alive CNT gaps.
UART0 at 115200 baud logs accepted frames, gaps, errors, RSSI and MASTER
round-trip time.

## Channel scans

The button cycles MASTER, SLAVE, GFSK scan, LoRa scan, then back to MASTER.
Each scan runs once on entry, with eight six-second windows (48 seconds
planned). Press the button to leave early. Both scans are receive-only.
The GFSK centers are 868.1, 868.3, 868.5, 868.7, 868.9, 869.425, 869.525
and 869.625 MHz at 117.3 kHz RX bandwidth. The LoRa centers are 867.1,
867.3, 867.5, 867.7, 867.9, 868.1, 868.3 and 868.5 MHz at 125 kHz.

The UART reports sample count, minimum, mean and maximum instantaneous
RSSI, share of samples at or above −95 dBm, and read errors. The threshold
statistic measures observed RF energy and does not classify decoded packets.
Results are reported after each channel and repeated at the end.

Call `sx1262_set_config()` and `sx1262_get_config()` to update and read back
the modem and packet parameters. The getter returns the last applied
configuration. Frequency and TX power also have individual accessors.

Call `sx1262_send()` or `sx1262_receive()` with a timeout in milliseconds.
Both calls are synchronous. A successful receive returns a payload length and
can also return packet RSSI and, in LoRa mode, SNR. Both peers need matching
radio settings. The GFSK application uses boosted continuous receive.
`sx1262_receive_continuous()` leaves the receiver running when its software
wait expires. Call `sx1262_start_continuous_rx()` to resume reception after
transmitting.

`sx1262_sleep()` retains the radio configuration. `sx1262_standby()` and
packet operations wake the chip on demand. `sx1262_deinit()` releases SPI2.
Use the API from one task or serialize access.

Board setup is based on the [Heltec V3.1 schematic](https://resource.heltec.cn/download/WiFi_LoRa_32_V3/HTIT-WB32LA%28F%29_V3.1_Schematic_Diagram.pdf)
and the [Heltec SX126x reference](https://github.com/HelTecAutomation/Heltec_ESP32/blob/master/src/driver/sx126x.c).
Command framing and calibration were cross-checked with the
[Semtech SX126x driver](https://github.com/Lora-net/sx126x_driver).
