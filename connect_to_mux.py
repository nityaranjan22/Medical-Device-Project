import time
import board
import adafruit_adt7410
import adafruit_tca9548a
import RPi.GPIO as GPIO
import board
import tkinter as tk
import threading
from simple_pid import PID
import sys

window = tk.Tk()
window.title("Temperature Controller")
window.geometry('1050x600')
# # window.attributes('-fullscreen', True)

PWM_PIN = 18
#MOSFET_PINS = [17, 4, 27, 22, 6, 13, 19, 26, 23, 24, 16, 20] 
MOSFET_PINS = [22, 4, 17, 27, 20, 23, 24, 16, 6, 13, 19, 26]

TARGET_TEMPERATURE = 75.0  # in degrees Celsius
TEMPERATURE_TOLERANCE = 0.5  # in degrees Celsius
TEMPERATURE_STEP = 3.0  # in degrees Celsius

i2c = board.I2C() 

GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)
pwm = GPIO.PWM(PWM_PIN, 10)
pwm.start(0)

for pin in MOSFET_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

stop_heating_event = threading.Event()
temperature_var = tk.StringVar()
warm_var = tk.StringVar()
painful_var = tk.StringVar()

current_mosfet = None

temp_sensor = [-1] * 12
mux0 = adafruit_tca9548a.TCA9548A(i2c, address=0x70)
mux1 = adafruit_tca9548a.TCA9548A(i2c, address=0x71)
mux2 = adafruit_tca9548a.TCA9548A(i2c, address=0x72)

temp_sensor[0] = adafruit_adt7410.ADT7410(mux0[0], address=0x49)
temp_sensor[1] = adafruit_adt7410.ADT7410(mux0[1], address=0x49)
temp_sensor[2] = adafruit_adt7410.ADT7410(mux0[2], address=0x49)
temp_sensor[3] = adafruit_adt7410.ADT7410(mux0[3], address=0x49)

temp_sensor[4] = adafruit_adt7410.ADT7410(mux1[2], address=0x49)
temp_sensor[5] = adafruit_adt7410.ADT7410(mux1[3], address=0x49)
temp_sensor[6] = adafruit_adt7410.ADT7410(mux1[4], address=0x49)
temp_sensor[7] = adafruit_adt7410.ADT7410(mux1[5], address=0x49)

temp_sensor[8] = adafruit_adt7410.ADT7410(mux2[2], address=0x49)
temp_sensor[9] = adafruit_adt7410.ADT7410(mux2[3], address=0x49)
temp_sensor[10] = adafruit_adt7410.ADT7410(mux2[4], address=0x49)
temp_sensor[11] = adafruit_adt7410.ADT7410(mux2[5], address=0x49)

def get_temperature(sensor):
    temperature = sensor.temperature
    nice_temp = str(round(temperature, 2))
    return float(nice_temp)

def check_temperature(current_mosfet, pid):
    mosfet_index = MOSFET_PINS.index(current_mosfet)
    current_temperature = get_temperature(temp_sensor[mosfet_index])

    temperature_var.set(f"Current temperature: {current_temperature:.2f} °C")

    if abs(current_temperature - pid.setpoint) <= TEMPERATURE_TOLERANCE:
        pid.setpoint = min(pid.setpoint + TEMPERATURE_STEP, TARGET_TEMPERATURE)

    control = pid(current_temperature)
    clipped_control = max(0, min(control, 100))
    pwm.ChangeDutyCycle(clipped_control)

    print(f"Current temperature: {current_temperature}  Setpoint: {pid.setpoint}    Raw PID control value: {control}    Clipped: {clipped_control}")

def start_heating(mosfet_pin):
    pid = PID(-0.5, -0.1, 0.5, setpoint=TEMPERATURE_STEP)
    global current_mosfet

    if current_mosfet is not None:
        stop_heating()
    
    for pin in MOSFET_PINS:
        GPIO.output(pin, GPIO.LOW)

    GPIO.output(mosfet_pin, GPIO.HIGH)
    current_mosfet = mosfet_pin

    stop_heating_event.clear()

    pid.setpoint = TEMPERATURE_STEP

    while not stop_heating_event.is_set():
        check_temperature(current_mosfet, pid)
        time.sleep(0.3)  

def stop_heating():
    global current_mosfet
    if current_mosfet is None:
        return

    GPIO.output(current_mosfet, GPIO.LOW)
    current_mosfet = None
    stop_heating_event.set()

def end():
    stop_heating()
    GPIO.cleaup()

def exit_program():
    stop_heating()
    GPIO.cleanup()
    window.destroy()
    sys.exit()

def warm():
    if current_mosfet is None:
        warm_var.set("No current temperature")
    else:
        mosfet_index = MOSFET_PINS.index(current_mosfet)
        warm_temp = get_temperature(temp_sensor[mosfet_index])
        warm_var.set(f"WARM : {warm_temp}")

def painful():
    if current_mosfet is None:
        painful_var.set("PAINFUL : 0.00")
    else:
        mosfet_index = MOSFET_PINS.index(current_mosfet)
        painful_temp = get_temperature(temp_sensor[mosfet_index])
        painful_var.set(f"PAINFUL : {painful_temp}")
    stop_heating()

def start(foo):
    pass

def ar():
    pass
   
################## UI ###############################
frame_start1 = tk.Frame(window)
frame_start2 = tk.Frame(window)
frame_display = tk.Frame(window)
frame_exit = tk.Frame(window)

frame_start1.grid(row=0, column=0, pady=(10, 0))
frame_start2.grid(row=1, column=0, pady=(10, 0))
frame_display.grid(row=2, column=0, pady=10)
frame_exit.grid(row=0, column=1, sticky="e")

buttons_start = [
    tk.Button(frame_start1 if i < 6 else frame_start2, text=f"Start {i+1}", width=12, height=3, bg='blue', fg='white', 
              command=lambda i=i: threading.Thread(target=start, args=(MOSFET_PINS[i],)).start())
    for i in range(len(MOSFET_PINS))
]

for i, button in enumerate(buttons_start):
    button.grid(row=0, column=i % 6, padx=5, pady=5) 



button_warm = tk.Button(frame_display, text="Warm", width=10, height=2, bg='yellow', fg='black', command=lambda: warm())
button_warm.grid(row=1, column=0, padx=5, pady=5)


button_painful = tk.Button(frame_display, text="Painful", width=10, height=2, bg='red', fg='white', command=lambda: painful())
button_painful.grid(row=3, column=0, padx=5, pady=5)


button_exit = tk.Button(frame_exit, text="Exit", width=10, height=2, bg='orange', fg='white', command=lambda:exit_program())
button_exit.pack(side='right')

frame_stop = tk.Frame(window)
frame_stop.grid(row=5, column=0, pady=10)
button_stop = tk.Button(frame_stop, text="Stop", width=15, height=5, bg='red', fg='white', command=stop_heating())
button_stop.grid(row=0, column=0, padx=5, pady=5)


window.mainloop()


