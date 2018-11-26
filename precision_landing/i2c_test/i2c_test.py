from smbus2 import SMBus
import struct
import time

# The arduino is arbitrarily set up as address 13
DEVICE_ADDRESS = 13

VAR_COORDINATE_0 = 0
VAR_COORDINATE_1 = 1

# 3 floats amount to a length of 12 bytes
COORDINATE_LEN = 12

bus = SMBus(1)

while True:
    try:
        data = bus.read_i2c_block_data(DEVICE_ADDRESS, VAR_COORDINATE_0, COORDINATE_LEN)
        (x,y,z) = struct.unpack('<fff',bytearray(data))
        print(x,y,z)
    except Exception as e:
        print(e)
    time.sleep(1)


