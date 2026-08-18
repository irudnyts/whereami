import spidev
import time
from math import radians

class ISM330DHCX:

    #---------------------------------------------------------------------------
    # Registers

    _WHO_AM_I = 0x0F
    _INTERNAL_FREQ_FINE = 0x63

    _FIFO_CTRL1 = 0x07
    _FIFO_CTRL2 = 0x08
    _FIFO_CTRL3 = 0x09
    _FIFO_CTRL4 = 0x0A
    _CTRL1_XL = 0x10
    _CTRL2_G = 0x11
    _CTRL10_C = 0x19

    _FIFO_STATUS1 = 0x3A
    _FIFO_STATUS2 = 0x3B

    _OUTX_L_A = 0x28
    _OUTX_L_G = 0x22
    _FIFO_DATA_OUT_TAG = 0x78
    
    _CHIP_ID = 0x6B

    _TAG_GYRO = 0x01
    _TAG_ACCEL = 0x02
    _TAG_TS = 0x04
    # Values
    # Output data rates

    ODR_SHUTDOWN = 0
    ODR_12_5_HZ = 1
    ODR_26_HZ = 2
    ODR_52_HZ = 3
    ODR_104_HZ = 4
    ODR_208_HZ = 5
    ODR_416_HZ = 6
    ODR_833_HZ = 7
    ODR_1_66K_HZ = 8
    ODR_3_33K_HZ = 9
    ODR_6_66K_HZ = 10
    ODR_1_6_HZ= 11

    ACCEL_RANGE_2G = 0
    ACCEL_RANGE_16G = 1
    ACCEL_RANGE_4G = 2
    ACCEL_RANGE_8G = 3
    
    GYRO_RANGE_125_DPS = 125
    GYRO_RANGE_250_DPS = 0
    GYRO_RANGE_500_DPS = 1
    GYRO_RANGE_1000_DPS = 2
    GYRO_RANGE_2000_DPS = 3
    GYRO_RANGE_4000_DPS = 4000

    BDR_12_5 = 0b0001
    BDR_26 = 0b0010
    BDR_52 = 0b0011
    BDR_104 = 0b0100
    BDR_208 = 0b0101
    BDR_417 = 0b0110
    BDR_833 = 0b0111
    BDR_1667 = 0b1000
    BDR_3333 = 0b1001
    BDR_6667 = 0b1010
    BDR_6_5 = 0b1011

    FIFO_MODE_B = 0b000
    FIFO_MODE_F = 0b001
    FIFO_MODE_C2F = 0b011
    FIFO_MODE_B2C = 0b100
    FIFO_MODE_C = 0b110
    FIFO_MODE_B2F = 0b111

    TS_DEC_OFF = 0b00
    TS_DEC1 = 0b01
    TS_DEC8 = 0b10
    TS_DEC32 = 0b11

    # Constants
    _MILLI_G_TO_ACCEL = 0.00980665
    _ACC_SCALE_MAP = {
        ACCEL_RANGE_2G: 0.061,
        ACCEL_RANGE_16G: 0.488,
        ACCEL_RANGE_4G: 0.122,
        ACCEL_RANGE_8G: 0.244
    }

    _GYRO_SCALE_MAP = {
        GYRO_RANGE_125_DPS: 4.375,
        GYRO_RANGE_250_DPS: 8.75,
        GYRO_RANGE_500_DPS: 17.50,
        GYRO_RANGE_1000_DPS: 35.0,
        GYRO_RANGE_2000_DPS: 70.0,
        GYRO_RANGE_4000_DPS: 140.0
    }

    def __init__(self, bus=0, device=0, accel_range=ACCEL_RANGE_2G, accel_rate=ODR_104_HZ, gyro_range=GYRO_RANGE_250_DPS, gyro_rate=ODR_104_HZ):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 1_000_000  # 1 MHz
        self.spi.mode = 0b00

        self.chip_id = self._read_register(self._WHO_AM_I)

        if self.chip_id != self._CHIP_ID:
            raise RuntimeError("ISM330DHCX not found, check your wiring!")

        self.ts_tick = self._get_ts_tick()

        self._accel_scale = None
        self._gyro_scale = None
        
        self.configure_accel(accel_range, accel_rate)
        self.configure_gyro(gyro_range, gyro_rate)

    #---------------------------------------------------------------------------
    # Low-level functions

    def _read_register(self, reg):
        # Set read bit (bit 7)
        reg = reg | 0x80
        resp = self.spi.xfer2([reg, 0x00])
        return resp[1]
    
    def _read_registers(self, reg, length):
        reg = reg | 0x80
        resp = self.spi.xfer2([reg] + [0x00]*length)
        return resp[1:]

    def _write_register(self, reg, value):
        # Set write bit (bit 7)
        reg = reg & 0x7F
        self.spi.xfer2([reg, value])

    #---------------------------------------------------------------------------
    # Utilities

    def _to_int16(self, lo, hi):
        val = (hi << 8) | lo
        if val & 0x8000:
            val -= 65536
        return val
    
    def _to_int8(self, val):
        return val - 256 if val & 0x80 else val

    def _to_uint32_le(self, b0, b1, b2, b3):
        val = (b3 << 24) | (b2 << 16) | (b1 << 8) | b0
        return val 
    
    def _scale_ts(self, ts): 
        return ts * self.ts_tick

    def _read_accel_raw(self):
        data = self._read_registers(self._OUTX_L_A, 6)

        x = self._to_int16(data[0], data[1])
        y = self._to_int16(data[2], data[3])
        z = self._to_int16(data[4], data[5])

        return x, y, z
    
    def _scale_accel(self, ax, ay, az): 
        return (
            ax * self._accel_scale * self._MILLI_G_TO_ACCEL,
            ay * self._accel_scale * self._MILLI_G_TO_ACCEL,
            az * self._accel_scale * self._MILLI_G_TO_ACCEL
        )

    def _read_gyro_raw(self):
        data = self._read_registers(self._OUTX_L_G, 6)
        
        x = self._to_int16(data[0], data[1])
        y = self._to_int16(data[2], data[3])
        z = self._to_int16(data[4], data[5])
        
        return x, y, z

    def _scale_gyro(self, gx, gy, gz): 
        return (
            radians(gx * self._gyro_scale / 1000),
            radians(gy * self._gyro_scale / 1000),
            radians(gz * self._gyro_scale / 1000)
        )
    
    def _read_fifo_level(self): 
        status = self._read_registers(self._FIFO_STATUS1, 2)
        status1, status2 = status[0], status[1]

        fifo_level = status1 | ((status2 & 0x03) << 8)
        return fifo_level  # number of words
    
    def _enable_ts(self): 
        mode_val = 0b00100000

        self._write_register(self._CTRL10_C, mode_val)
        time.sleep(0.2)        
    
    def _get_ts_tick(self):
        raw = self._read_register(self._INTERNAL_FREQ_FINE)
        freq_fine = self._to_int8(raw)

        effective_freq = 40000 * (1 + 0.0015 * freq_fine)
        return 1_000_000 / effective_freq

    #---------------------------------------------------------------------------
    # High-level functions

    def configure_fifo(self, mode=FIFO_MODE_C, accel_bdr=BDR_417, gyro_bdr=BDR_417, ts_decimation=TS_DEC1): 

        if ts_decimation != self.TS_DEC_OFF:
            self._enable_ts()

        bdr_val = (gyro_bdr << 4) | accel_bdr
        self._write_register(self._FIFO_CTRL3, bdr_val)
        time.sleep(0.2)

        mode_val = ((ts_decimation & 0x03) << 6) | (mode & 0x3F)
        self._write_register(self._FIFO_CTRL4, mode_val)
        time.sleep(0.2)

    def read_fifo(self): 

        n_words = self._read_fifo_level()
        
        if n_words == 0:
            return []

        n_bytes = 7 * n_words 

        data = self._read_registers(self._FIFO_DATA_OUT_TAG, n_bytes)

        samples = []

        for i in range(n_words):
            offset = i * 7

            tag = data[offset]
            x_l = data[offset + 1]
            x_h = data[offset + 2]
            y_l = data[offset + 3]
            y_h = data[offset + 4]
            z_l = data[offset + 5]
            z_h = data[offset + 6]

            tag = (tag >> 3) & 0b11111

            if tag == self._TAG_TS:
                samples.append({
                    "type": "ts",
                    "value": self._scale_ts(self._to_uint32_le(x_l, x_h, y_l, y_h))
                })
            elif tag == self._TAG_GYRO: 
                x = self._to_int16(x_l, x_h)
                y = self._to_int16(y_l, y_h)
                z = self._to_int16(z_l, z_h)

                samples.append({
                    "type": "gyro",
                    "value": self._scale_gyro(x, y, z)
                })
            elif tag == self._TAG_ACCEL: 
                x = self._to_int16(x_l, x_h)
                y = self._to_int16(y_l, y_h)
                z = self._to_int16(z_l, z_h)

                samples.append({
                    "type": "accel",
                    "value": self._scale_accel(x, y, z)
                })

        return samples

    def configure_accel(self, accel_range, accel_rate):
        self._accel_scale = self._ACC_SCALE_MAP[accel_range]
        acc_val = (accel_rate << 4) | (accel_range << 2)
        self._write_register(self._CTRL1_XL, acc_val)
        time.sleep(0.2)

    def configure_gyro(self, gyro_range, gyro_rate):
        self._gyro_scale = self._GYRO_SCALE_MAP[gyro_range]

        fs_125 = 0
        fs_4000 = 0
        fs_g = 0

        if gyro_range == self.GYRO_RANGE_125_DPS:
            fs_125 = 1
        elif gyro_range == self.GYRO_RANGE_4000_DPS:
            fs_4000 = 1
        else:
            fs_g = gyro_range  # 250/500/1000/2000 mapping already matches your enum

        gyro_val = (
            (gyro_rate << 4) |
            (fs_g << 2) |
            (fs_125 << 1) |
            (fs_4000 << 0)
        )

        self._write_register(self._CTRL2_G, gyro_val)
        time.sleep(0.2)

    def read_accel(self):
        ax, ay, az = self._read_accel_raw()
        ax, ay, az = self._scale_accel(ax, ay, az)
        return ax, ay, az

    def read_gyro(self):
        gx, gy, gz = self._read_gyro_raw()
        gx, gy, gz = self._scale_gyro(gx, gy, gz)
        return gx, gy, gz



def main():

    sensor = ISM330DHCX()

    while True:
        ax, ay, az = sensor.read_accel()
        print(ax, ay, az)
        gx, gy, gz = sensor.read_gyro()
        print(gx, gy, gz)
        time.sleep(0.5)

if __name__ == "__main__":
    main()