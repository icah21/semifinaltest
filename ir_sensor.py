import RPi.GPIO as GPIO

class IRSensor:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN)

    def is_object_detected(self):
        return GPIO.input(self.pin) == 0  # adjust to 1 if needed
