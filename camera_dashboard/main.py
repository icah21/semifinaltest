import threading
import time
import RPi.GPIO as GPIO

from ir_sensor import IRSensor
from stepper_motor import StepperMotor
from servo import ServoController
import camera_dashboard  # This now has a callable function

# Adjusted GPIO pin assignments to avoid conflicts
IR_PIN = 17
STEPPER_PINS = [4, 5, 6, 13]  # Avoid conflict with Servo (18)
SERVO_PIN = 18

# Initialize components
ir_sensor = IRSensor(IR_PIN)
stepper = StepperMotor(*STEPPER_PINS)
servo = ServoController(SERVO_PIN)

# Shared state
state = {
    "current_angle": 0,
    "detection_result": None,
    "camera_done": False
}

def monitor_ir_sensor():
    print("[IR] IR Sensor monitoring started.")
    while True:
        if ir_sensor.is_object_detected():
            print("[IR] Object detected.")
            state["current_angle"] = stepper.go_to_angle(state["current_angle"], 90)
            print("[Stepper] Reached 90 degrees. Waiting for camera detection.")
            break
        time.sleep(0.1)

def wait_for_camera_and_decide():
    print("[Camera] Waiting for camera to detect a type...")
    while not state["camera_done"]:
        result = max(camera_dashboard.counts, key=camera_dashboard.counts.get)
        if camera_dashboard.counts[result] > 0:
            print(f"[Camera] Detected type: {result}")
            state["detection_result"] = result
            state["camera_done"] = True
        time.sleep(0.5)

def control_servo_based_on_detection():
    while not state["camera_done"]:
        time.sleep(0.1)

    print("[Stepper] Rotating to 180 degrees for drop zone.")
    state["current_angle"] = stepper.go_to_angle(state["current_angle"], 180)
    print("[Servo] Acting based on detection result.")

    result = state["detection_result"]
    if result == "Criollo":
        servo.move_to_variety("Criollo")
    elif result == "Forastero":
        servo.move_to_variety("Forastero")
    elif result == "Trinitario":
        servo.move_to_variety("Trinitario")
    else:
        servo.move_to_variety("Unknown")

    print("[System] Process complete. Ready for next detection.")
    cleanup()

def cleanup():
    try:
        servo.cleanup()
        GPIO.cleanup()
        print("[System] GPIO cleaned up.")
    except Exception as e:
        print(f"[Cleanup Error] {e}")

if __name__ == "__main__":
    try:
        cam_thread = threading.Thread(target=camera_dashboard.launch_dashboard, daemon=True)
        cam_thread.start()

        ir_thread = threading.Thread(target=monitor_ir_sensor)
        detect_thread = threading.Thread(target=wait_for_camera_and_decide)
        servo_thread = threading.Thread(target=control_servo_based_on_detection)

        ir_thread.start()
        ir_thread.join()

        detect_thread.start()
        servo_thread.start()

        detect_thread.join()
        servo_thread.join()

    except KeyboardInterrupt:
        print("[System] Interrupted by user.")
        cleanup()
