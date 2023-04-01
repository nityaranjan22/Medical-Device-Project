import time
import board
import busio
import adafruit_adt7410
from RPi import GPIO

# Constants
PWM_PIN = 18  # Connect the PWM signal to GPIO 18
MAX_TARGET_TEMPERATURE = 100.0  # Maximum target temperature in Celsius
RAMP_TIME = 60  # Time in seconds for the target temperature to reach the maximum value

# PID controller constants
Kp = 1.0
Ki = 0.01
Kd = 0.1

# Initialize I2C
i2c = board.I2C()

# Initialize ADT7410
temp_sensor = adafruit_adt7410.ADT7410(i2c, address=0x49)

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)
pwm = GPIO.PWM(PWM_PIN, 100)  # Create a PWM object with a 100 Hz frequency
pwm.start(0)  # Start PWM with a 0% duty cycle

# PID controller variables
previous_error = 0
integral = 0
time_step = 0.1

# Function to smoothly increase the target temperature
def ramp_temperature(start_temperature, max_temperature, ramp_time, current_time):
    return start_temperature + (max_temperature - start_temperature) * min(current_time / ramp_time, 1)

start_time = time.time()
start_temperature = temp_sensor.temperature

try:
    while True:
        current_time = time.time() - start_time
        target_temperature = ramp_temperature(start_temperature, MAX_TARGET_TEMPERATURE, RAMP_TIME, current_time)

        # Read temperature from ADT7410
        temp = temp_sensor.temperature

        # Calculate PID controller values
        error = target_temperature - temp
        integral += error * time_step
        derivative = (error - previous_error) / time_step
        previous_error = error

        # Compute control output (duty cycle)
        control_output = Kp * error + Ki * integral + Kd * derivative
        control_output = max(min(control_output, 1 00), 0)  # Clamp the control output between 0 and 100

        # Update PWM duty cycle
        pwm.ChangeDutyCycle(control_output)


        print(f'Current time:{current_time}     Target Temp:{target_temperature}       Current Temperature:{temp}   Control Output:{control_output}')


        # Wait for the next iteration
        time.sleep(time_step)

except KeyboardInterrupt:
    print("\nTerminating the PID controller.")

finally:
    pwm.stop()
    GPIO.cleanup()
    print("Cleaned up GPIO.")
