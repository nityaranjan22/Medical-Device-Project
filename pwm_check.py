import time
import board
import adafruit_adt7410
from RPi import GPIO

# Constants
PWM_PIN = 18  # Connect the PWM signal to GPIO 18
PWM_FREQ = 100  # PWM frequency in Hz
TIME_STEP = 0.01  # Time step in seconds for updating the duty cycle

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)

# Create a PWM object with a 100 Hz frequency
pwm = GPIO.PWM(PWM_PIN, PWM_FREQ)
pwm.start(0)  # Start PWM with a 0% duty cycle

# try:
#     while True:
#         pwm.ChangeDutyCycle(100)
#         time.sleep(TIME_STEP)


i2c = board.I2C()

temp_sensor = adafruit_adt7410.ADT7410(i2c, address=0x49)

# try:
#     while True:
#         # Gradually increase the duty cycle from 0 to 100%
#         for duty_cycle in range(101):
#             pwm.ChangeDutyCycle(duty_cycle)
#             temp = temp_sensor.temperature
#             print(f'DUTY CYCLE:{duty_cycle}     TEMPERATURE:{temp}')
#             time.sleep(0.5)

#         # # Gradually decrease the duty cycle from 100 to 0%
#         # for duty_cycle in range(100, -1, -1):
#         #     pwm.ChangeDutyCycle(duty_cycle)
#         #     time.sleep(TIME_STEP)

try:
    while True:
            pwm.ChangeDutyCycle(100)
            temp = temp_sensor.temperature
            print(f'DUTY CYCLE:{100}     TEMPERATURE:{temp}')
            time.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopping PWM test.")

finally:
    pwm.stop()
    GPIO.cleanup()
    print("Cleaned up GPIO.")
