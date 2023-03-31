import time
from RPi import GPIO

# Constants
PWM_PIN = 18  # Connect the PWM signal to GPIO 18
TIME_STEP = 0.1  # Time step in seconds for updating the duty cycle

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)

# Create a PWM object with a 100 Hz frequency
pwm = GPIO.PWM(PWM_PIN)
pwm.start(0)  # Start PWM with a 0% duty cycle

try:
    while True:
        pwm.ChangeDutyCycle(100)
        time.sleep(TIME_STEP)

# try:
#     while True:
#         # Gradually increase the duty cycle from 0 to 100%
#         for duty_cycle in range(101):
#             pwm.ChangeDutyCycle(duty_cycle)
#             time.sleep(TIME_STEP)

#         # Gradually decrease the duty cycle from 100 to 0%
#         for duty_cycle in range(100, -1, -1):
#             pwm.ChangeDutyCycle(duty_cycle)
#             time.sleep(TIME_STEP)

except KeyboardInterrupt:
    print("\nStopping PWM test.")

finally:
    pwm.stop()
    GPIO.cleanup()
    print("Cleaned up GPIO.")
