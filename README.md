# CanSat flight software

MicroPython firmware for a CanSat built on a Raspberry Pi Pico W. It reads temperature and pressure from a BMP280 sensor, works out altitude, logs every reading to a CSV file on the Pico's flash, and sends it to a ground station over a 433 MHz LoRa radio. It also listens for commands from the ground station, so you can change the sample rate, pause, or reboot the can while it's flying.

## Hardware

| Part | What it does |
|---|---|
| Raspberry Pi Pico W | Runs the firmware. Its on-board LED blinks to show what's happening |
| BMP280 | Measures temperature and pressure (I2C, address `0x76`) |
| SX1276/77/78 LoRa module (e.g. RFM9x, Ra-02) | Sends telemetry and receives commands at 433 MHz |

### Wiring

| Pico pin | Connects to |
|---|---|
| GP0 (I2C0 SDA) | BMP280 SDA |
| GP1 (I2C0 SCL) | BMP280 SCL |
| GP16 (SPI0 MISO) | LoRa MISO |
| GP17 | LoRa NSS / CS |
| GP18 (SPI0 SCK) | LoRa SCK |
| GP19 (SPI0 MOSI) | LoRa MOSI |
| GP20 | LoRa RESET |
| GP21 | LoRa DIO0 |
| 3V3 / GND | Power for both modules |

## Files

| File | What it is |
|---|---|
| `main.py` | Flight program: logs to `measurements.csv` with mission elapsed time (`HH:MM:SS.s` since boot) |
| `main2.py` | Earlier, simpler version: temperature and pressure only, logged to `measurements.txt` |
| `main3.py` | Sets the Pico's real-time clock to a launch date and time, timestamps each row with it, and starts a new numbered CSV (`measurements_1.csv`, `measurements_2.csv`, …) on every boot so earlier flights are never overwritten |
| `main4.py` | Same as `main3.py`, plus it checks whether the Pico is on USB or battery and names the log `measurements_debug_N.csv` or `measurements_battery_N.csv` to match |
| `main5.py` | Same as `main3.py` |
| `bmp280.py` | BMP280 driver |
| `sx127x.py` | SX127x LoRa driver, set up for the Pico's SPI |

The Pico only runs a file called `main.py` when it powers up. To fly one of the other versions, copy it onto the Pico **as** `main.py`.

## Setup

1. Flash MicroPython for the Pico W onto the board ([instructions](https://micropython.org/download/RPI_PICO_W/)).
2. Copy `bmp280.py`, `sx127x.py` and your chosen main file (saved as `main.py`) onto the Pico, using Thonny or `mpremote`:

   ```sh
   mpremote cp bmp280.py sx127x.py :
   mpremote cp main4.py :main.py
   ```

3. Before launch, edit two values at the top of the main file:
   - `SEA_LEVEL_PRESSURE` – the day's sea-level pressure in hPa (from a local weather report). The default of 1013.25 hPa can put the altitude out by tens of metres.
   - `LAUNCH_DATETIME` (in `main3.py`, `main4.py` and `main5.py`) – the date and time the RTC is set to at boot, as `(year, month, day, weekday, hour, minute, second, subsecond)`, with weekday 0 = Monday. The Pico has no battery-backed clock, so this is the time the log starts from on every power-up.
4. Power the Pico. The LED gives one long blink when telemetry starts.

## What it does on boot

1. Sets up the BMP280 and the LoRa radio. If the radio isn't found, it carries on logging to flash without radio.
2. Starts a background thread that listens for ground station commands.
3. Creates the CSV file and sends a `BOOT` packet.
4. Every `SAMPLE_INTERVAL` seconds (1 s by default) it reads the sensor, works out altitude, prints the reading over USB serial, transmits it and appends it to the CSV.

Press Ctrl-C in the serial console to stop cleanly before copying the CSV files off the Pico.

## Radio settings

| Setting | Value |
|---|---|
| Frequency | 433.0 MHz |
| Spreading factor | 7 |
| Bandwidth | 125 kHz |
| Coding rate | 4/5 |
| Preamble | 8 symbols |
| Sync word | `0x34` |
| CRC | On |
| TX power | 17 dBm (PA_BOOST) |

The ground station radio has to use the same settings.

## Packet format

Every packet, in both directions, looks like this:

```
$TYPE,SEQ,DATA*CK#
```

- `TYPE` – one letter (see below)
- `SEQ` – a 4-digit packet counter, e.g. `0042`
- `DATA` – comma-separated fields
- `CK` – XOR of every character between `$` and `*`, as two hex digits

| Type | Direction | Meaning | Example `DATA` |
|---|---|---|---|
| `H` | CanSat → ground | Boot beacon | `BOOT,RATE=1` |
| `R` | CanSat → ground | Reading: number, °C, hPa, metres | `17,21.43,1008.62,38.9` |
| `A` | CanSat → ground | Reply to a command | `SETRATE,ACK,0.5` |
| `C` | Ground → CanSat | Command | `SETRATE,2` |

### Commands

A command packet has `C` as its type, so `DATA` starts with the command name, for example `$C,0001,STATUS*CK#`. Packets with a bad checksum are ignored.

| Command | What it does | Reply |
|---|---|---|
| `SETRATE,n` | Sets the sample interval to `n` seconds (0.1–60) | `SETRATE,ACK,n`, or `SETRATE,NAK,OUT_OF_RANGE` / `SETRATE,NAK,INVALID` |
| `STATUS` | Reports the current settings | `STATUS,RATE=1,ACTIVE=True` |
| `PAUSE` | Stops taking readings | `PAUSE,ACK` |
| `RESUME` | Starts taking readings again | `RESUME,ACK` |
| `REBOOT` | Restarts the Pico after 2 seconds | `REBOOT,ACK` |
| anything else | – | `UNKNOWN,<command>` |

The LED blinks twice quickly whenever a command arrives.

## Logged data

`main3.py`, `main4.py` and `main5.py` write:

```
datetime,temperature_C,pressure_hPa,altitude_m
2026-03-11 13:00:05,21.43,1008.62,38.92
```

`main.py` writes the same columns with mission elapsed time in place of the date, under the header `datetime,temperature,pressure,altitude`.

Altitude comes from the barometric formula:

```
altitude = 44330 × (1 − (P / SEA_LEVEL_PRESSURE)^(1 / 5.255))
```

CSV files are listed in `.gitignore`, so flight data stays out of the repo.
