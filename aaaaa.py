import time
import board
import adafruit_adt7410
import adafruit_tca9548a
import RPi.GPIO as GPIO
import board
import busio
import tkinter as tk
import threading
from simple_pid import PID
i2c = board.I2C() 
import time
import board
import adafruit_adt7410
import adafruit_tca9548a
import RPi.GPIO as GPIO
import board
import busio
import tkinter as tk
import threading


temp_sensor = [-1] * 8

mux0 = adafruit_tca9548a.TCA9548A(i2c, address=0x70)
mux1 = adafruit_tca9548a.TCA9548A(i2c, address=0x71)
mux2 = adafruit_tca9548a.TCA9548A(i2c, address=0x72)

temp_sensor[0] = adafruit_adt7410.ADT7410(mux0[0], address=0x49)
temp_sensor[1] = adafruit_adt7410.ADT7410(mux0[1], address=0x49)
temp_sensor[2] = adafruit_adt7410.ADT7410(mux0[2], address=0x49)
temp_sensor[3] = adafruit_adt7410.ADT7410(mux0[3], address=0x49)

temp_sensor[4] = adafruit_adt7410.ADT7410(mux1[4], address=0x49)
temp_sensor[5] = adafruit_adt7410.ADT7410(mux1[5], address=0x49)
temp_sensor[6] = adafruit_adt7410.ADT7410(mux1[6], address=0x49)
temp_sensor[7] = adafruit_adt7410.ADT7410(mux1[7], address=0x49)

print("\nMUX 0")
for channel in range(8):
    if mux0[channel].try_lock():
        print("Channel {}:".format(channel), end="")
        addresses = mux0[channel].scan()
        print([hex(address) for address in addresses])
        mux0[channel].unlock()


print("\nMUX 1")
for channel in range(8):

    if mux1[channel].try_lock():
        print("Channel {}:".format(channel), end="")
        addresses = mux1[channel].scan()
        print([hex(address) for address in addresses])
        mux1[channel].unlock()
        

print("\nMUX 2")
for channel in range(8):

    if mux2[channel].try_lock():
        print("Channel {}:".format(channel), end="")
        addresses = mux2[channel].scan()
        print([hex(address) for address in addresses])
        mux2[channel].unlock()
