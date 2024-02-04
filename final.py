import time
from simple_pid import PID
import tkinter as tk
import RPi.GPIO as GPIO
import adafruit_adt7410
import threading
import busio
import board

window = tk.Tk()

PWM_PIN = 18

MOSFET_PINS = [4, 17, 27, 22, 6, 13, 19, 26, 23, 24, 16, 20]  # 12 pins from 2 to 13

TARGET_TEMPERATURE = 75.0  # in degrees Celsius
TEMPERATURE_TOLERANCE = 0.5  # in degrees Celsius
TEMPERATURE_STEP = 5.0  # in degrees Celsius


# Create a new I2C bus
i2c = adafruit_adt7410.busio.I2C(board.SCL, board.SDA)

# Create a new PID controller
pid = PID(-1.0, -0.1, 0.05, setpoint=TEMPERATURE_STEP)

# Configure the GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)

# Configure the GPIO pins for the MOSFETs
for pin in MOSFET_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

pwm = GPIO.PWM(PWM_PIN, 100)  # 100Hz frequency

pwm.start(0)

temperature_var = tk.StringVar()

stop_heating_event = threading.Event()

current_mosfet = None
adt7410 = None

print("INIT")

def check_temperature():
    global adt7410
    print("!!!!!!! IN CHECK_TEMPERATURE !!!!!!!!!!!!")
    #adt7410 = adafruit_adt7410._ADT7410_STATUS
    print(f"!!!!!!!!!!!!!!! adt7410 INIT {adt7410.i2c_device.i2c.scan()}!!!!!!!!!!!!!!!!!!")
    #adt7410.i2c_device.i2c.try_lock()
    try:
        current_temperature = adt7410.temperature
        print(f"Current temperature: {current_temperature:.2f} °C")
        temperature_var.set(f"Current temperature: {current_temperature:.2f} °C")

        # If the current temperature is within the tolerance of the setpoint,
        # increase the setpoint by the temperature step, but do not exceed the target temperature
        if abs(current_temperature - pid.setpoint) <= TEMPERATURE_TOLERANCE:
            pid.setpoint = min(pid.setpoint + TEMPERATURE_STEP, TARGET_TEMPERATURE)

        control = pid(current_temperature)
        clipped_control = max(0, min(control, 100))
        pwm.ChangeDutyCycle(clipped_control)
    except:
        pass

    #print(f"Current temperature: {current_temperature}  Setpoint: {pid.setpoint}    Raw PID control value: {control}    Clipped: {clipped_control}")


def start_heating(mosfet_pin):
    global adt7410
    print("!!!!!!!!!!!! IN START_HEATING !!!!!!!!!!!")
    global current_mosfet

    if current_mosfet is not None:
        stop_heating()

    for pin in MOSFET_PINS:
        GPIO.output(pin, GPIO.LOW)
        print("!!!!!!!!!!!!! ALL GPIO LOW !!!!!!!!!!!!!")

    GPIO.output(mosfet_pin, GPIO.HIGH)
    print(f"!!!!!!!!!!!!!!!! PIN {mosfet_pin} HIGH !!!!!!!!!!!!")
    current_mosfet = mosfet_pin

    stop_heating_event.clear()

    pid.setpoint = TEMPERATURE_STEP
    print(f"!!!!!!!!!!!!! TEMP STEP: {TEMPERATURE_STEP} !!!!!!!!!!!!!!!!!!!!!!")
    
#    if adt7410 is not None:
#        adt7410.i2c_device.i2c.unlock() 
    try:
        adt7410 = adafruit_adt7410.ADT7410(i2c, address=0x49)
    except:
        time.sleep(0.2)
        adt7410 = adafruit_adt7410.ADT7410(i2c, address=0x49)

    while not stop_heating_event.is_set():
        check_temperature()
        time.sleep(0.1)  

def stop_heating():
    print("!!!!!!!!!!!!! IN STOP_HEATING !!!!!!!!!!")
    global current_mosfet
    if current_mosfet is None:
        return

    GPIO.output(current_mosfet, GPIO.LOW)
    current_mosfet = None
    stop_heating_event.set()

def reset():
    print("!!!!!!!!!!! RESET !!!!!!!!!!!!!!!")
    stop_heating()
    GPIO.cleanup()

# Create the UI

buttons_start = [
    tk.Button(window, text=f"Start {i+1}", width=12, height=3, bg='blue', fg='white', command=lambda i=i: threading.Thread(target=start_heating, args=(MOSFET_PINS[i],)).start())
    for i in range(len(MOSFET_PINS))
]
button_stop = tk.Button(window, text="Stop", width=15, height=5, bg='red', fg='white', command=stop_heating)
button_reset = tk.Button(window, text="Reset", width=15, height=5, bg='green', fg='white', command=reset)
label_temperature = tk.Label(window, textvariable=temperature_var, font=("Helvetica", 24))

for button in buttons_start:
    button.pack()
button_stop.pack()
button_reset.pack()
label_temperature.pack()

try:
    window.mainloop()
except KeyboardInterrupt:
    print("Ctrl C clicked....Stopping Code...")
    stop_heating()
    reset()
    time.sleep(0.2)
except RuntimeError as e:
    print(f"RUNTIME ERROR: {e}....STOPPING CODE")
    stop_heating()
    reset()
    time.sleep(0.2)
finally:
    print(f'UNKNOWN ERROR......STOPPING CODE')
    stop_heating()
    reset()
    time.sleep(0.2)



