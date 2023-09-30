#!/usr/bin/env python3

# from magPlot import MagPlot
from serial import Serial
import numpy as np
from the_collector import Collector
from datetime import datetime
import time
# from threading import Thread, Event, Lock
# import copy
import sys


info = {"lis3mdl": {
        "mag_range_gs": 4,
        "samplerate_hz": 155,
        "date": datetime.now().isoformat()
    }
}

data = []

def main():
    global plot_buffer
    coll = Collector()
    coll.timestamp = False

    ser = Serial()
    ser.port = "/dev/tty.usbmodem14601"
    ser.baudrate = 1000000
    ser.open()

    try:
        while (mp.ok):
            buffer = []
            c = ser.read().decode("utf8")
            while (c != '\n'):
                buffer.append(c)
                c = ser.read().decode("utf8")
            s = ''.join(buffer)
            print(s)
            v=s.split(',')
            if len(v) != 3: continue
            d = {
                'x': float(v[0]),
                'y': float(v[1]),
                'z': float(v[2]),
                "timestamp": time.time()
            }
            data.append(d)

            if lock.locked() == False:
                plot_buffer.append(d)

    except KeyboardInterrupt:
        print("Data points captured: ", len(data))
        coll.write("magcal.pkl", data, info)
        sys.stdout.flush()
    finally:
        ser.close()

main()
