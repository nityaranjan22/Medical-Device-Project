import time
from simple_pid import PID
import tkinter as tk
from tkinter import messagebox
import RPi.GPIO as GPIO
import adafruit_adt7410
import threading
import busio
import board

PWM_PIN = 18
TARGET_TEMPERATURE = 75.0

i2c = busio.I2C(board.SCL, board.SDA)

pid = PID(1.0, 0.1, 0.05, setpoint=TARGET_TEMPERATURE)

GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)

pwm = GPIO.PWM(PWM_PIN, 100)  # 100Hz frequency
pwm.start(0)

temperature_var = tk.StringVar()
stop_heating_event = threading.Event()

def check_temperature():
    current_temperature = adt7410.temperature
    temperature_var.set(f"Current temperature: {current_temperature:.2f} °C")
    control = pid(current_temperature)
    pwm.ChangeDutyCycle(control)

def start_heating():
    stop_heating_event.clear()
    
    while not stop_heating_event.is_set():
        check_temperature()
        time.sleep(0.1)

def stop_heating():
    stop_heating_event.set()
    pwm.stop()

def reset():
    GPIO.cleanup()

# Create the UI
window = tk.Tk()
button_start = tk.Button(window, text="Start", command=lambda: threading.Thread(target=start_heating).start())
button_stop = tk.Button(window, text="Stop", command=stop_heating)
button_reset = tk.Button(window, text="Reset", command=reset)
label_temperature = tk.Label(window, textvariable=temperature_var)

button_start.pack()
button_stop.pack()
button_reset.pack()
label_temperature.pack()

# Start the main event loop
window.mainloop()
