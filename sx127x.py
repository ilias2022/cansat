"""
SX127x LoRa Driver for MicroPython
Simplified version for CanSat telemetry
Based on Semtech SX1276/77/78/79 datasheet
Fixed for Raspberry Pi Pico (uses write_readinto for SPI)
"""

from machine import Pin, SPI
import time

# SX127x Registers
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_LNA = 0x0c
REG_FIFO_ADDR_PTR = 0x0d
REG_FIFO_TX_BASE_ADDR = 0x0e
REG_FIFO_RX_BASE_ADDR = 0x0f
REG_FIFO_RX_CURRENT_ADDR = 0x10
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_PKT_RSSI_VALUE = 0x1a
REG_PKT_SNR_VALUE = 0x1b
REG_MODEM_CONFIG_1 = 0x1d
REG_MODEM_CONFIG_2 = 0x1e
REG_PREAMBLE_MSB = 0x20
REG_PREAMBLE_LSB = 0x21
REG_PAYLOAD_LENGTH = 0x22
REG_MODEM_CONFIG_3 = 0x26
REG_RSSI_WIDEBAND = 0x2c
REG_DETECTION_OPTIMIZE = 0x31
REG_DETECTION_THRESHOLD = 0x37
REG_SYNC_WORD = 0x39
REG_DIO_MAPPING_1 = 0x40
REG_VERSION = 0x42

# Modes
MODE_LONG_RANGE_MODE = 0x80
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_TX = 0x03
MODE_RX_CONTINUOUS = 0x05
MODE_RX_SINGLE = 0x06

# IRQ flags
IRQ_TX_DONE_MASK = 0x08
IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20
IRQ_RX_DONE_MASK = 0x40

class SX127x:
    def __init__(self, spi, cs, rst, dio0=None, frequency=433.0):
        """
        Initialize SX127x LoRa module

        Args:
            spi: SPI object
            cs: Chip Select pin
            rst: Reset pin
            dio0: DIO0 pin (optional, for interrupts)
            frequency: Frequency in MHz (433.0, 868.0, 915.0)
        """
        self.spi = spi
        self.cs = cs
        self.rst = rst
        self.dio0 = dio0

        # Initialize CS high (deselected) and RST high (not in reset)
        self.cs.init(Pin.OUT, value=1)
        self.rst.init(Pin.OUT, value=1)
        if self.dio0:
            self.dio0.init(Pin.IN)

        # Let pins settle before doing anything
        time.sleep_ms(50)

        # Reset the module and verify communication
        # Retry up to 3 times — module may need multiple resets after a dirty shutdown
        version = 0x00
        for attempt in range(3):
            self.reset()
            time.sleep_ms(50)  # extra settling time after reset

            version = self.read_register(REG_VERSION)
            print(f"  LoRa version check (attempt {attempt + 1}): 0x{version:02x}")

            if version == 0x12:
                break

            # If first attempt failed, try a longer reset
            time.sleep_ms(100)

        if version != 0x12:
            raise RuntimeError(
                f"Invalid SX127x version: 0x{version:02x} (expected 0x12)\n"
                f"  Check wiring and restart"
            )

        # Enter sleep mode
        self.sleep()

        # Set LoRa mode
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)
        time.sleep_ms(10)

        # Set frequency
        self.set_frequency(frequency)

        # Set base addresses
        self.write_register(REG_FIFO_TX_BASE_ADDR, 0)
        self.write_register(REG_FIFO_RX_BASE_ADDR, 0)

        # Set LNA boost
        self.write_register(REG_LNA, self.read_register(REG_LNA) | 0x03)

        # Set auto AGC
        self.write_register(REG_MODEM_CONFIG_3, 0x04)

        # Set output power to 17 dBm (for PA_BOOST pin)
        self.set_tx_power(17)

        # Enter standby mode
        self.standby()

    def reset(self):
        """Hardware reset with proper timing"""
        self.cs.value(1)          # deselect before reset
        self.rst.value(0)
        time.sleep_ms(50)         # hold reset low for 50ms
        self.rst.value(1)
        time.sleep_ms(100)        # wait 100ms for oscillator startup

    def read_register(self, address):
        """Read single register (Pico-compatible using write_readinto)"""
        self.cs.value(0)
        buf_out = bytes([address & 0x7f, 0x00])
        buf_in = bytearray(2)
        self.spi.write_readinto(buf_out, buf_in)
        self.cs.value(1)
        return buf_in[1]

    def write_register(self, address, value):
        """Write single register"""
        self.cs.value(0)
        self.spi.write(bytes([address | 0x80, value]))
        self.cs.value(1)

    def sleep(self):
        """Enter sleep mode"""
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)

    def standby(self):
        """Enter standby mode"""
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY)

    def set_frequency(self, frequency):
        """
        Set frequency in MHz
        Formula: Frf = (Frequency * 2^19) / 32
        """
        frf = int((frequency * 1000000.0) / (32000000.0 / 524288))
        self.write_register(REG_FRF_MSB, (frf >> 16) & 0xff)
        self.write_register(REG_FRF_MID, (frf >> 8) & 0xff)
        self.write_register(REG_FRF_LSB, frf & 0xff)

    def set_tx_power(self, level):
        """
        Set TX power level in dBm (2-17)
        Uses PA_BOOST pin
        """
        if level < 2:
            level = 2
        elif level > 17:
            level = 17
        self.write_register(REG_PA_CONFIG, 0x80 | (level - 2))

    def set_spreading_factor(self, sf):
        """
        Set spreading factor (6-12)
        Higher = longer range but slower
        """
        if sf < 6:
            sf = 6
        elif sf > 12:
            sf = 12

        if sf == 6:
            self.write_register(REG_DETECTION_OPTIMIZE, 0xc5)
            self.write_register(REG_DETECTION_THRESHOLD, 0x0c)
        else:
            self.write_register(REG_DETECTION_OPTIMIZE, 0xc3)
            self.write_register(REG_DETECTION_THRESHOLD, 0x0a)

        reg = self.read_register(REG_MODEM_CONFIG_2)
        reg = (reg & 0x0f) | ((sf << 4) & 0xf0)
        self.write_register(REG_MODEM_CONFIG_2, reg)

    def set_signal_bandwidth(self, sbw):
        """
        Set signal bandwidth in kHz
        Options: 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500
        """
        bw_map = {
            7.8: 0, 10.4: 1, 15.6: 2, 20.8: 3,
            31.25: 4, 41.7: 5, 62.5: 6, 125: 7,
            250: 8, 500: 9
        }
        bw = bw_map.get(sbw, 7)  # Default 125 kHz
        reg = self.read_register(REG_MODEM_CONFIG_1)
        reg = (reg & 0x0f) | (bw << 4)
        self.write_register(REG_MODEM_CONFIG_1, reg)

    def set_coding_rate(self, denominator):
        """
        Set coding rate 4/5, 4/6, 4/7, 4/8
        """
        if denominator < 5:
            denominator = 5
        elif denominator > 8:
            denominator = 8
        cr = denominator - 4
        reg = self.read_register(REG_MODEM_CONFIG_1)
        reg = (reg & 0xf1) | (cr << 1)
        self.write_register(REG_MODEM_CONFIG_1, reg)

    def set_preamble_length(self, length):
        """Set preamble length"""
        self.write_register(REG_PREAMBLE_MSB, (length >> 8) & 0xff)
        self.write_register(REG_PREAMBLE_LSB, length & 0xff)

    def set_sync_word(self, sw):
        """Set sync word (0x12 = LoRaWAN public, 0x34 = private)"""
        self.write_register(REG_SYNC_WORD, sw)

    def enable_crc(self):
        """Enable CRC"""
        reg = self.read_register(REG_MODEM_CONFIG_2)
        reg |= 0x04
        self.write_register(REG_MODEM_CONFIG_2, reg)

    def disable_crc(self):
        """Disable CRC"""
        reg = self.read_register(REG_MODEM_CONFIG_2)
        reg &= 0xfb
        self.write_register(REG_MODEM_CONFIG_2, reg)

    def send(self, data):
        """
        Send data packet

        Args:
            data: bytes or string to send (max 255 bytes)
        """
        if isinstance(data, str):
            data = data.encode()

        if len(data) > 255:
            raise ValueError("Packet too large")

        # Enter standby mode
        self.standby()

        # Clear IRQ flags
        self.write_register(REG_IRQ_FLAGS, 0xff)

        # Initialize FIFO for transmission
        self.write_register(REG_FIFO_ADDR_PTR, 0)
        self.write_register(REG_PAYLOAD_LENGTH, 0)

        # Write data to FIFO
        for byte in data:
            self.write_register(REG_FIFO, byte)

        # Set payload length
        self.write_register(REG_PAYLOAD_LENGTH, len(data))

        # Enter TX mode
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX)

        # Wait for TX done
        start = time.ticks_ms()
        while True:
            irq_flags = self.read_register(REG_IRQ_FLAGS)
            if irq_flags & IRQ_TX_DONE_MASK:
                # Clear IRQ
                self.write_register(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK)
                break
            if time.ticks_diff(time.ticks_ms(), start) > 5000:
                raise RuntimeError("TX timeout")
            time.sleep_ms(10)

        # Return to standby
        self.standby()

    def receive(self, timeout=5000):
        """
        Receive data packet

        Args:
            timeout: Timeout in milliseconds

        Returns:
            Received bytes or None if timeout
        """
        # Enter standby mode
        self.standby()

        # Clear IRQ flags
        self.write_register(REG_IRQ_FLAGS, 0xff)

        # Enter RX mode
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)

        # Wait for RX done
        start = time.ticks_ms()
        while True:
            irq_flags = self.read_register(REG_IRQ_FLAGS)

            if irq_flags & IRQ_RX_DONE_MASK:
                # Check CRC error
                if irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK:
                    self.write_register(REG_IRQ_FLAGS, 0xff)
                    return None

                # Read received data
                length = self.read_register(REG_RX_NB_BYTES)
                addr = self.read_register(REG_FIFO_RX_CURRENT_ADDR)
                self.write_register(REG_FIFO_ADDR_PTR, addr)

                data = bytearray()
                for _ in range(length):
                    data.append(self.read_register(REG_FIFO))

                # Get RSSI
                rssi = self.read_register(REG_PKT_RSSI_VALUE) - 157

                # Clear IRQ
                self.write_register(REG_IRQ_FLAGS, 0xff)

                # Return to standby
                self.standby()

                return bytes(data)

            if time.ticks_diff(time.ticks_ms(), start) > timeout:
                # Timeout
                self.standby()
                return None

            time.sleep_ms(10)

    def get_rssi(self):
        """Get current RSSI value"""
        return self.read_register(REG_RSSI_WIDEBAND) - 157

    def packet_rssi(self):
        """Get RSSI of last received packet"""
        return self.read_register(REG_PKT_RSSI_VALUE) - 157

    def packet_snr(self):
        """Get SNR of last received packet"""
        snr = self.read_register(REG_PKT_SNR_VALUE)
        if snr & 0x80:  # Sign bit set
            snr = ((~snr + 1) & 0xff) >> 2
            snr = -snr
        else:
            snr = (snr & 0xff) >> 2
        return snr
