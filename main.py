"""
CanSat Bidirectional Transceiver
Continuously sends telemetry (temp + pressure) every second
AND receives commands from ground station

Commands supported:
- SETRATE,n - Set sample interval to n seconds
- STATUS - Request status update
- REBOOT - Restart system
- PAUSE - Pause telemetry
- RESUME - Resume telemetry
"""

from machine import Pin, I2C, SPI, reset
from bmp280 import BMP280
from sx127x import SX127x
import time
import rp2
import _thread

# Hardware Setup - BMP280 Sensor (I2C)
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
bmp = BMP280(i2c, addr=0x76)

# Hardware Setup - LoRa Radio (SPI)
spi = SPI(0, baudrate=2000000, polarity=0, phase=0,
          sck=Pin(18), mosi=Pin(19), miso=Pin(16))
lora_cs = Pin(17, Pin.OUT)
lora_rst = Pin(20, Pin.OUT)
lora_dio0 = Pin(21, Pin.IN)

# Thread lock to prevent SPI bus conflicts between TX and RX
spi_lock = _thread.allocate_lock()

# Initialize LoRa (433 MHz)
try:
    lora = SX127x(spi, lora_cs, lora_rst, lora_dio0, frequency=433.0)
    lora.set_spreading_factor(7)
    lora.set_signal_bandwidth(125)
    lora.set_coding_rate(5)
    lora.set_preamble_length(8)
    lora.set_sync_word(0x34)
    lora.enable_crc()
    print("✓ LoRa transceiver initialized at 433 MHz")
except Exception as e:
    print(f"✗ LoRa initialization failed: {e}")
    lora = None

# Built-in LED
led = Pin("LED", Pin.OUT)

# Configuration (can be changed via commands)
SAMPLE_INTERVAL = 1  # seconds between readings
TRANSMIT_ENABLED = True
DATA_FILE = 'measurements.csv'
SEA_LEVEL_PRESSURE = 1013.25  # hPa — adjust on launch day for accuracy

# State variables
telemetry_active = True  # continuous readings on by default
packet_counter = 0
rx_thread_running = False

def blink_led(times, delay=0.2):
    """Blink LED for feedback"""
    for _ in range(times):
        led.on()
        time.sleep(delay)
        led.off()
        time.sleep(delay)

def calculate_checksum(payload):
    """Calculate XOR checksum"""
    checksum = 0
    for char in payload:
        checksum ^= ord(char)
    return checksum

def transmit_packet(packet_type, data):
    """Transmit data via LoRa (thread-safe)"""
    global packet_counter

    if not lora or not TRANSMIT_ENABLED:
        return False

    acquired = spi_lock.acquire(1)  # 1 second timeout
    if not acquired:
        print("  ✗ TX skipped - could not acquire lock")
        return False

    try:
        packet_counter += 1
        seq = f"{packet_counter:04d}"
        payload = f"{packet_type},{seq},{data}"
        checksum = calculate_checksum(payload)
        packet = f"${payload}*{checksum:02X}#"

        led.on()
        lora.send(packet)
        led.off()

        print(f"  TX: {packet}")
        return True

    except Exception as e:
        print(f"  ✗ TX error: {e}")
        return False
    finally:
        spi_lock.release()

def verify_checksum(payload, checksum_str):
    """Verify packet checksum"""
    try:
        expected = int(checksum_str, 16)
        calculated = calculate_checksum(payload)
        return calculated == expected
    except:
        return False

def parse_command(packet):
    """Parse received command packet"""
    if not packet.startswith('$') or not packet.endswith('#'):
        return None

    packet = packet[1:-1]
    if '*' not in packet:
        return None

    payload, checksum = packet.split('*', 1)

    if not verify_checksum(payload, checksum):
        print(f"  ✗ Bad checksum: {packet}")
        return None

    parts = payload.split(',')
    if len(parts) < 3:
        return None

    return {
        'type': parts[0],
        'seq': parts[1],
        'command': parts[2] if len(parts) > 2 else None,
        'args': parts[3:] if len(parts) > 3 else []
    }

def handle_command(cmd):
    """Execute received command"""
    global SAMPLE_INTERVAL, telemetry_active

    command = cmd['command']
    args = cmd['args']

    print(f"\nCommand received: {command}")

    if command == "SETRATE" and len(args) > 0:
        try:
            new_rate = float(args[0])
            if 0.1 <= new_rate <= 60:
                SAMPLE_INTERVAL = new_rate
                transmit_packet('A', f"SETRATE,ACK,{new_rate}")
                print(f"  Sample rate set to {new_rate}s")
            else:
                transmit_packet('A', f"SETRATE,NAK,OUT_OF_RANGE")
        except:
            transmit_packet('A', f"SETRATE,NAK,INVALID")

    elif command == "STATUS":
        status = f"RATE={SAMPLE_INTERVAL},ACTIVE={telemetry_active}"
        transmit_packet('A', f"STATUS,{status}")
        print(f"  Status sent")

    elif command == "PAUSE":
        telemetry_active = False
        transmit_packet('A', f"PAUSE,ACK")
        print("  Telemetry paused")

    elif command == "RESUME":
        telemetry_active = True
        transmit_packet('A', f"RESUME,ACK")
        print("  Telemetry resumed")

    elif command == "REBOOT":
        transmit_packet('A', f"REBOOT,ACK")
        print("  Rebooting in 2 seconds...")
        time.sleep(2)
        reset()

    else:
        transmit_packet('A', f"UNKNOWN,{command}")
        print(f"  Unknown command: {command}")

def receive_thread():
    """Background thread to listen for commands"""
    global rx_thread_running

    rx_thread_running = True
    print("RX thread started - listening for commands")

    while rx_thread_running:
        try:
            acquired = spi_lock.acquire(1)  # 1 second timeout
            if not acquired:
                time.sleep(0.05)
                continue

            try:
                data = lora.receive(timeout=200)  # short timeout so lock isn't held long
            finally:
                spi_lock.release()

            if data:
                try:
                    packet_str = data.decode('utf-8')
                    print(f"\nRX: {packet_str}")

                    cmd = parse_command(packet_str)
                    if cmd and cmd['type'] == 'C':
                        blink_led(2, 0.05)
                        handle_command(cmd)

                except UnicodeDecodeError:
                    print("  ✗ Failed to decode packet")

        except Exception as e:
            pass

        time.sleep(0.05)  # small gap before next listen cycle

def calculate_altitude(pressure_hpa):
    """Calculate altitude from pressure using barometric formula"""
    if pressure_hpa <= 0:
        return 0.0
    return 44330.0 * (1.0 - (pressure_hpa / SEA_LEVEL_PRESSURE) ** (1.0 / 5.255))

def format_met(elapsed_s):
    """Format mission elapsed time as HH:MM:SS.s"""
    h = int(elapsed_s // 3600)
    m = int((elapsed_s % 3600) // 60)
    s = elapsed_s % 60
    return f"{h:02d}:{m:02d}:{s:04.1f}"

def init_csv():
    """Write CSV header if file doesn't exist or is empty"""
    try:
        f = open(DATA_FILE, 'r')
        has_data = len(f.read(1)) > 0
        f.close()
        if has_data:
            return
    except OSError:
        pass
    with open(DATA_FILE, 'w') as f:
        f.write("datetime,temperature,pressure,altitude\n")

def save_reading(met, temp, pressure, altitude):
    """Append a single reading to CSV"""
    try:
        with open(DATA_FILE, 'a') as f:
            f.write(f"{met},{temp:.2f},{pressure:.2f},{altitude:.2f}\n")
    except Exception as e:
        print(f"  Save error: {e}")

# ── Main program ──────────────────────────────────────────────
print("\n" + "=" * 60)
print("CANSAT CONTINUOUS TRANSCEIVER")
print("=" * 60)
print("\nSensors:")
print("  BMP280 - Temperature & Pressure")
if lora:
    print("  LoRa SX127x - 433 MHz Transceiver")
    print(f"  Spreading Factor: 7, BW: 125 kHz")
else:
    print("  LoRa - NOT CONNECTED")
print("\nCommands accepted:")
print("  SETRATE,n  - Set interval (seconds)")
print("  STATUS     - Request status")
print("  PAUSE      - Pause telemetry")
print("  RESUME     - Resume telemetry")
print("  REBOOT     - Restart system")
print("=" * 60)

# Start receive thread
if lora:
    _thread.start_new_thread(receive_thread, ())
    time.sleep(0.5)

# Initialize CSV file with header
init_csv()

# Record boot time for mission elapsed time
boot_time = time.ticks_ms()

# Send startup beacon
if lora and TRANSMIT_ENABLED:
    transmit_packet('H', f"BOOT,RATE={SAMPLE_INTERVAL}")
    print("✓ Startup beacon transmitted\n")

print(f"Continuous telemetry active — reading every {SAMPLE_INTERVAL}s")
print("Press Ctrl-C to stop\n")
blink_led(1, 0.5)

reading_number = 0

# Main loop — continuous readings
try:
    while True:
        if telemetry_active:
            # Read sensors (I2C — no SPI conflict)
            temp = bmp.temperature
            pressure = bmp.pressure / 25600
            altitude = calculate_altitude(pressure)

            # Mission elapsed time
            elapsed_ms = time.ticks_diff(time.ticks_ms(), boot_time)
            met = format_met(elapsed_ms / 1000.0)

            reading_number += 1
            print(f"  #{reading_number}  T={temp:.2f}°C  P={pressure:.2f} hPa  Alt={altitude:.1f} m  MET={met}")

            # Transmit over LoRa
            transmit_packet('R', f"{reading_number},{temp:.2f},{pressure:.2f},{altitude:.1f}")

            # Save locally
            save_reading(met, temp, pressure, altitude)

        # Wait for the configured interval, but in small steps
        # so that PAUSE/RESUME commands take effect quickly
        waited = 0.0
        while waited < SAMPLE_INTERVAL:
            time.sleep(0.1)
            waited += 0.1

except KeyboardInterrupt:
    print("\nStopping telemetry...")
    rx_thread_running = False
    time.sleep(0.3)
    print("Done.")
