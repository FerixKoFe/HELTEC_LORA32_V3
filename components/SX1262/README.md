# SX1262 radio driver

Native ESP-IDF component for the Heltec WiFi LoRa 32 V3.1. Add `SX1262` to
the calling component's `PRIV_REQUIRES` and include `sx1262.h`.

Use `sx1262_default_config(frequency_hz)` for LoRa or
`sx1262_default_gfsk_config(frequency_hz)` for GFSK. Adjust the returned
configuration before calling `sx1262_init()`. The MASTER/SLAVE application
uses GFSK at 869.525 MHz and sets the transmit power to +14 dBm by default
under **GFSK radio link** in ESP-IDF configuration.

The GFSK profile uses 100 kbit/s, ±25 kHz deviation, BT 0.5, 187.2 kHz
receive bandwidth, a 32-bit preamble, a 16-bit detector and 32-bit sync word
`2D D4 B7 E1`. The packet engine accepts variable payloads up to 64 bytes,
adds and validates CRC-16/CCITT (polynomial `0x1021`, seed `0xFFFF`), and
uses whitening seed `0x1FF`. The MASTER/SLAVE application sends and expects
51 bytes: 50 data bytes and one Alive CNT byte.

MASTER sends a fresh random payload with an even counter on a nominal
100 ms cycle. Both roles allow 110% of the cycle (110 ms) for a valid
receive. If MASTER is still waiting at 100 ms, it starts the next cycle
after the receive window closes.
SLAVE replies with the bitwise inverse and the matching odd counter.
Counters wrap at 256. MASTER validates the CRC, counter, and inverted
payload. SLAVE validates the CRC, parity, and sequence. A valid receive
sets LINK UP. After 300 ms without a valid receive the link drops.
The first valid exchange in each mode clears MISS. SLAVE shows a provisional
miss per 110 ms without a valid MASTER frame. A later in-sequence counter
clears the provisional miss, while a counter gap records the confirmed loss.
SLAVE LINK reports valid requests; MASTER LINK reports validated replies.
UART0 at 115200 baud logs counters, gaps, errors, RSSI, MASTER RX readiness and
round-trip time, and SLAVE reply completion. Enable `LINK_LOG_PAYLOAD` for
full payload hex dumps.

## Channel scans

The button cycles MASTER, SLAVE, GFSK scan, LoRa scan, then back to MASTER.
Each scan runs once on entry, with eight six-second windows (48 seconds
planned). Press the button to leave early. Both scans are receive-only.
The GFSK centers are 868.1, 868.3, 868.5, 868.7, 868.9, 869.425, 869.525
and 869.625 MHz at 187.2 kHz RX bandwidth. The LoRa centers are 867.1,
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
