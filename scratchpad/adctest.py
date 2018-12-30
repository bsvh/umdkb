#!/usr/bin/env python3
# -*- coding: utf-8 -*

import spidev
import time

BUS = 0
DEVICE = 1
RATE = 1500

spi = spidev.SpiDev()
spi.open(BUS, DEVICE)

spi.max_speed_hz = RATE

while True:
    vals = spi.readbytes(10)
    for val in vals:
        print(val)
    time.sleep(1)
