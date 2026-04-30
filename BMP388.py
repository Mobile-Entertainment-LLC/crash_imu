"""
BMP388.py - Lightweight BMP388 barometer driver using smbus2.
No external library needed beyond smbus2.

Usage:
    from BMP388 import BMP388
    bmp = BMP388()
    print(bmp.temperature)   # degrees C
    print(bmp.pressure)      # hPa
    print(bmp.altitude)      # metres above sea level
"""

import smbus2
import time
import struct

BMP388_ADDR     = 0x77
SEA_LEVEL_HPA   = 1013.25   # standard sea level pressure

# Registers
REG_CHIP_ID     = 0x00
REG_DATA        = 0x04      # 6 bytes: press (3) + temp (3)
REG_PWR_CTRL    = 0x1B
REG_OSR         = 0x1C
REG_ODR         = 0x1D
REG_CALIB       = 0x31      # 21 bytes of calibration


class BMP388:
    def __init__(self, bus_number=1, address=BMP388_ADDR):
        self._bus  = smbus2.SMBus(bus_number)
        self._addr = address

        chip = self._bus.read_byte_data(self._addr, REG_CHIP_ID)
        if chip != 0x50:
            raise RuntimeError(f"BMP388 not found (chip id=0x{chip:02X})")

        self._load_calibration()

        # Enable pressure + temperature, normal mode
        self._bus.write_byte_data(self._addr, REG_PWR_CTRL, 0x33)
        # Oversampling: x8 pressure, x1 temp
        self._bus.write_byte_data(self._addr, REG_OSR, 0x04)
        # ODR: 50Hz
        self._bus.write_byte_data(self._addr, REG_ODR, 0x02)
        time.sleep(0.1)

    def _load_calibration(self):
        raw = self._bus.read_i2c_block_data(self._addr, REG_CALIB, 21)

        # Unpack calibration coefficients (from BMP388 datasheet)
        (T1, T2, T3,
         P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11) = struct.unpack(
            "<HHbhhbbHHbbhbb", bytes(raw[:21]))

        # Convert to floating point (datasheet Table 14)
        self._T1  = T1  / 2**(-8)
        self._T2  = T2  / 2**30
        self._T3  = T3  / 2**48
        self._P1  = (P1  - 2**14) / 2**20
        self._P2  = (P2  - 2**14) / 2**29
        self._P3  = P3  / 2**32
        self._P4  = P4  / 2**37
        self._P5  = P5  / 2**(-3)
        self._P6  = P6  / 2**6
        self._P7  = P7  / 2**8
        self._P8  = P8  / 2**15
        self._P9  = P9  / 2**48
        self._P10 = P10 / 2**48
        self._P11 = P11 / 2**65

    def _read_raw(self):
        data = self._bus.read_i2c_block_data(self._addr, REG_DATA, 6)
        raw_p = (data[2] << 16) | (data[1] << 8) | data[0]
        raw_t = (data[5] << 16) | (data[4] << 8) | data[3]
        return raw_p, raw_t

    def _compensate_temp(self, raw_t):
        pd1 = raw_t - self._T1
        pd2 = pd1 * self._T2
        temp = pd2 + (pd1 * pd1) * self._T3
        return temp

    def _compensate_press(self, raw_p, t_lin):
        pd1  = self._P6 * t_lin
        pd2  = self._P7 * (t_lin ** 2)
        pd3  = self._P8 * (t_lin ** 3)
        po1  = self._P5 + pd1 + pd2 + pd3

        pd1  = self._P2 * t_lin
        pd2  = self._P3 * (t_lin ** 2)
        pd3  = self._P4 * (t_lin ** 3)
        po2  = raw_p * (self._P1 + pd1 + pd2 + pd3)

        pd1  = raw_p ** 2
        pd2  = self._P9 + self._P10 * t_lin
        pd3  = pd1 * pd2
        pd4  = pd3 + (raw_p ** 3) * self._P11

        press = po1 + po2 + pd4
        return press / 100.0  # Pa → hPa

    @property
    def temperature(self):
        _, raw_t = self._read_raw()
        return round(self._compensate_temp(raw_t), 2)

    @property
    def pressure(self):
        raw_p, raw_t = self._read_raw()
        t_lin = self._compensate_temp(raw_t)
        return round(self._compensate_press(raw_p, t_lin), 2)

    @property
    def altitude(self):
        p = self.pressure
        alt = 44330.0 * (1.0 - (p / SEA_LEVEL_HPA) ** (1.0 / 5.255))
        return round(alt, 1)

    def read_all(self):
        """Returns (temperature_c, pressure_hpa, altitude_m) in one I2C read."""
        raw_p, raw_t = self._read_raw()
        t_lin  = self._compensate_temp(raw_t)
        temp   = round(t_lin, 2)
        press  = round(self._compensate_press(raw_p, t_lin), 2)
        alt    = round(44330.0 * (1.0 - (press / SEA_LEVEL_HPA) ** (1.0 / 5.255)), 1)
        return temp, press, alt


if __name__ == "__main__":
    print("Testing BMP388...")
    bmp = BMP388()
    for _ in range(5):
        t, p, a = bmp.read_all()
        print(f"Temp: {t:.1f} C  |  Pressure: {p:.1f} hPa  |  Altitude: {a:.1f} m")
        time.sleep(0.5)