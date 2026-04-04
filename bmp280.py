from micropython import const
from ustruct import unpack
import time

# BMP280 default address
BMP280_I2C_ADDR = 0x76

# Register addresses
_BMP280_REGISTER_DIG_T1 = const(0x88)
_BMP280_REGISTER_CHIPID = const(0xD0)
_BMP280_REGISTER_CONTROL = const(0xF4)
_BMP280_REGISTER_CONFIG = const(0xF5)
_BMP280_REGISTER_PRESSUREDATA = const(0xF7)
_BMP280_REGISTER_TEMPDATA = const(0xFA)

class BMP280:
    def __init__(self, i2c, addr=BMP280_I2C_ADDR):
        self.i2c = i2c
        self.addr = addr
        self.t_fine = 0
        
        # Check chip ID
        chip_id = self.i2c.readfrom_mem(self.addr, _BMP280_REGISTER_CHIPID, 1)[0]
        if chip_id not in (0x58, 0x56, 0x57):
            raise RuntimeError(f"Failed to find BMP280! Chip ID: 0x{chip_id:x}")
        
        # Read calibration data
        self._read_coefficients()
        
        # Configure sensor: normal mode, oversampling x16 for both temp and pressure
        self.i2c.writeto_mem(self.addr, _BMP280_REGISTER_CONTROL, bytes([0xB7]))
        time.sleep_ms(10)
        
    def _read_coefficients(self):
        # Read temperature calibration (6 bytes starting at 0x88)
        coeff_t = self.i2c.readfrom_mem(self.addr, 0x88, 6)
        self._dig_T1 = unpack("<H", coeff_t[0:2])[0]
        self._dig_T2 = unpack("<h", coeff_t[2:4])[0]
        self._dig_T3 = unpack("<h", coeff_t[4:6])[0]
        
        # Read pressure calibration (18 bytes starting at 0x8E)
        coeff_p = self.i2c.readfrom_mem(self.addr, 0x8E, 18)
        self._dig_P1 = unpack("<H", coeff_p[0:2])[0]
        self._dig_P2 = unpack("<h", coeff_p[2:4])[0]
        self._dig_P3 = unpack("<h", coeff_p[4:6])[0]
        self._dig_P4 = unpack("<h", coeff_p[6:8])[0]
        self._dig_P5 = unpack("<h", coeff_p[8:10])[0]
        self._dig_P6 = unpack("<h", coeff_p[10:12])[0]
        self._dig_P7 = unpack("<h", coeff_p[12:14])[0]
        self._dig_P8 = unpack("<h", coeff_p[14:16])[0]
        self._dig_P9 = unpack("<h", coeff_p[16:18])[0]
        
    def _read_raw_temp(self):
        data = self.i2c.readfrom_mem(self.addr, _BMP280_REGISTER_TEMPDATA, 3)
        return (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    
    def _read_raw_pressure(self):
        data = self.i2c.readfrom_mem(self.addr, _BMP280_REGISTER_PRESSUREDATA, 3)
        return (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    
    @property
    def temperature(self):
        adc_T = self._read_raw_temp()
        var1 = ((adc_T >> 3) - (self._dig_T1 << 1)) * (self._dig_T2 >> 11)
        var2 = (((((adc_T >> 4) - self._dig_T1) * ((adc_T >> 4) - self._dig_T1)) >> 12) * self._dig_T3) >> 14
        self.t_fine = var1 + var2
        return ((self.t_fine * 5 + 128) >> 8) / 100.0
    
    @property
    def pressure(self):
        # Must read temperature first to set t_fine
        temp = self.temperature
        
        adc_P = self._read_raw_pressure()
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * self._dig_P6
        var2 = var2 + ((var1 * self._dig_P5) << 17)
        var2 = var2 + (self._dig_P4 << 35)
        var1 = ((var1 * var1 * self._dig_P3) >> 8) + ((var1 * self._dig_P2) << 12)
        var1 = ((1 << 47) + var1) * self._dig_P1 >> 33
        
        if var1 == 0:
            return 0
        
        p = 1048576 - adc_P
        p = (((p << 31) - var2) * 3125) // var1
        var1 = (self._dig_P9 * (p >> 13) * (p >> 13)) >> 25
        var2 = (self._dig_P8 * p) >> 19
        
        return ((p + var1 + var2) >> 8) + (self._dig_P7 << 4)