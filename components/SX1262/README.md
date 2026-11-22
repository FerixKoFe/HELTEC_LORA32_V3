# SX1262 LoRa driver

Native ESP-IDF component for the Heltec WiFi LoRa 32 V3.1. Add `SX1262` to
the calling component's `PRIV_REQUIRES` and include `sx1262.h`.

Use `sx1262_default_config(frequency_hz)` with the frequency for your board
variant and region. Adjust the returned configuration before calling
`sx1262_init()`. The default modulation is SF7, 125 kHz, coding rate 4/5,
explicit header, eight preamble symbols and CRC enabled.

Call `sx1262_set_config()` and `sx1262_get_config()` to update and read back
the modem and packet parameters. The getter returns the last applied
configuration. Frequency and TX power also have individual accessors.

Call `sx1262_send()` or `sx1262_receive()` with a timeout in milliseconds.
Both calls are synchronous. A successful receive returns a payload length and
can also return packet RSSI and SNR. Both peers need compatible modulation,
sync word, IQ polarity, and packet header settings.

`sx1262_sleep()` retains the radio configuration. `sx1262_standby()` and
packet operations wake the chip on demand. `sx1262_deinit()` releases SPI2.
Use the API from one task or serialize access. This component implements raw
LoRa packet exchange.

Board setup is based on the [Heltec V3.1 schematic](https://resource.heltec.cn/download/WiFi_LoRa_32_V3/HTIT-WB32LA%28F%29_V3.1_Schematic_Diagram.pdf)
and the [Heltec SX126x reference](https://github.com/HelTecAutomation/Heltec_ESP32/blob/master/src/driver/sx126x.c).
Command framing and calibration were cross-checked with the
[Semtech SX126x driver](https://github.com/Lora-net/sx126x_driver).
