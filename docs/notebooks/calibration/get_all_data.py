#!/usr/bin/env python3

# from magPlot import MagPlot
from serial import Serial
import numpy as np
from the_collector import Collector
from datetime import datetime
import time
from datatypes import *
# from threading import Thread, Event, Lock
# import copy
import sys


info = {
    "lis3mdl": {
        "mag_range_gs": 4,
        "samplerate_hz": 155,
        "date": datetime.now().isoformat()
    }
}

data = {
    "accels": [],
    "gyros": [],
    "mags": [],
    "pt": [],
    "timestamp": []
}

def main():
    global plot_buffer
    coll = Collector()
    coll.timestamp = False

    ser = Serial()
    ser.port = "/dev/tty.usbmodem14401"
    ser.baudrate = 1000000
    ser.open()

    try:
        while (True):
            buffer = []
            c = ser.read().decode("utf8")
            while (c != '\n'):
                buffer.append(c)
                c = ser.read().decode("utf8")

            if buffer[0] == '*':
                continue

            s = ''.join(buffer)
            print(s)

            v = s.split('|') # breakout sensors

            vv = v[1].split(',')
            a = vec_t(float(vv[0]),float(vv[1]),float(vv[2]))
            data["accels"].append(a)

            vv = v[2].split(',')
            g = vec_t(float(vv[0]),float(vv[1]),float(vv[2]))
            data["gyros"].append(g)

            ts = float(v[3])
            data["timestamp"].append(ts)

            vv = v[4].split(',')
            m = vec_t(float(vv[0]),float(vv[1]),float(vv[2]))
            data["mags"].append(m)

            vv = v[5].split(',')
            pt = pt_t(float(vv[0]),float(vv[1]))
            data["pt"].append(pt)

    except KeyboardInterrupt:
        print("Data points captured: ", len(data["accels"]))
        coll.write("allcal.pkl", data, info)
        sys.stdout.flush()
    finally:
        ser.close()

main()
