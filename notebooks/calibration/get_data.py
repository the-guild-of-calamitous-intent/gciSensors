#!/usr/bin/env python3

# from magPlot import MagPlot
from serial import Serial
import numpy as np
# from the_collector import Collector
from datetime import datetime
import time
# from threading import Thread, Event, Lock
# import copy
from struct import Struct
import sys
from pickle_save import pickle_write, pickle_read
from colorama import Fore
from collections import namedtuple

vec_t = namedtuple("vec_t","x y z")
imu_t = namedtuple("imu_t","a g m temperature timestamp")


"""
struct __attribute__((packed)) imu_t {
  sensors::vec_t a,g,m;
  float temperature;
  uint64_t timestamp;
};
"""
imu_struct = Struct("9ffQ")
msg_size = imu_struct.size
print(f"msg size: {msg_size}")

if len(sys.argv) != 3:
    print(f"{Fore.RED}ERROR: proper useage: {Fore.CYAN}{sys.argv[0]} <port> <file.pkl>{Fore.RESET}")
    sys.exit(1)

port = sys.argv[1]
filename = sys.argv[2]

info = {
  "lsm6dsox": {
    "accel_range_g": 4,
    "gyro_range_dps": 2000,
    "rate_hz": 208,
  },
  "lis3mdl": {
    "mag_range_gauss": 4,
    "rate_hz": 300,
  },
  "date": datetime.now().isoformat()
}

msgs = []

ser = Serial()
ser.port = port
ser.baudrate = 1000000
ser.open()

try:
  while True:
    while True:
      header = ser.read(2);
      if header == b"AA":
        # print(header)
        break
    msg = ser.read(msg_size)
    d = imu_struct.unpack(msg)
    imu = imu_t(vec_t(*d[:3]), vec_t(*d[3:6]), vec_t(*d[6:9]),*d[9:])
    msgs.append(imu)
    # print(imu)

except KeyboardInterrupt:
  print(f"\r{len(msgs)} data points captured")

  data = {
    "data": msgs,
    "info": info
  }
  pickle_write("magcal.pkl", data, info)
    
finally:
  ser.close()

