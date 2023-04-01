import time
import board
import busio
import adafruit_adt7410
from RPi import GPIO

# Constants
PWM_PIN = 18  # Connect the PWM signal to GPIO 18
BUTTON_PIN = 24  # Connect the button to GPIO 24
MAX_TARGET_TEMPERATURE = 100.0  # Maximum target temperature in Celsius
TEMP_STEP = 5.0  # Temperature step in degrees Celsius
TEMP_TOLERANCE = 0.5  # Tolerance in degrees Celsius for considering the target temperature reached

# PID controller constants
Kp = 1.0
Ki = 0.01
Kd = 0.1

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize ADT7410
adt7410 = adafruit_adt7410.ADT7410(i2c)

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
pwm = GPIO.PWM(PWM_PIN, 100)  # Create a PWM object with a 100 Hz frequency
pwm.start(0)  # Start PWM with a 0% duty cycle

# PID controller variables
previous_error = 0
integral = 0
time_step = 0.1

def button_pressed():
    return GPIO.input(BUTTON_PIN) == GPIO.LOW

start_temperature = adt7410.temperature
target_temperature = start_temperature

try:
    while not button_pressed():
        # Read temperature from ADT7410
        temp = adt7410.temperature

        # Check if the current target temperature is reached
        if abs(temp - target_temperature) <= TEMP_TOLERANCE:
            # Move to the next target temperature step, up to the maximum
            target_temperature = min(target_temperature + TEMP_STEP, MAX_TARGET_TEMPERATURE)

        # Calculate PID controller values
        error = target_temperature - temp
        integral += error * time_step
        derivative = (error - previous_error) / time_step
        previous_error = error

        # Compute control output (duty cycle)
        control_output = Kp * error + Ki * integral + Kd * derivative
        control_output = max(min(control_output, 100), 0)  # Clamp the control output between 0 and 100

        # Update PWM duty cycle
        pwm.ChangeDutyCycle(control_output)

        # Wait for the next iteration
        time.sleep(time_step)

    print("\nButton pressed. Terminating the PID controller.")

except KeyboardInterrupt:
    print("\nTerminating the PID controller.")

finally:
    pwm.stop()
    GPIO.cleanup()
    print("Cleaned up GPIO.")
