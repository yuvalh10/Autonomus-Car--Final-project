import time
import logging
import threading
from gpiozero import AngularServo

logging.basicConfig(level=logging.INFO)

# Initialize AngularServo
SERVO_PIN = 13
servo = AngularServo(
    SERVO_PIN,
    min_angle=0,          # Minimum angle (0 degrees)
    max_angle=180,        # Maximum angle (180 degrees)
    min_pulse_width=0.5/1000,   # 0.5ms pulse width for 0�
    max_pulse_width=2.5/1000,   # 2.5ms pulse width for 180�
    frame_width=20/1000         # 20ms frame width (50Hz)
)

tracking_lock = threading.Lock()
last_call_time = 0
current_servo_angle = 90  # Center position (90 degrees)
is_centered = True

# Servo angle limits
MIN_SERVO_ANGLE = 0    # 0 degrees (leftmost)
MAX_SERVO_ANGLE = 180  # 180 degrees (rightmost)
CENTER_SERVO_ANGLE = 90  # 90 degrees (center)

def move_servo_smooth(target_angle, steps=10):
    """Move servo smoothly to target angle with interpolation"""
    global current_servo_angle
    
    # Clamp the angle to valid range
    target_angle = max(MIN_SERVO_ANGLE, min(MAX_SERVO_ANGLE, target_angle))
    
    # Calculate step size for smooth movement
    angle_diff = target_angle - current_servo_angle
    step_size = angle_diff / steps
    
    # Move in small steps
    for i in range(steps):
        current_servo_angle += step_size
        servo.angle = current_servo_angle
        time.sleep(0.02)  # Small delay between steps
    
    # Ensure we reach exact target
    current_servo_angle = target_angle
    servo.angle = target_angle

def move_servo_instant(target_angle):
    """Move servo instantly to target angle (for real-time tracking)"""
    global current_servo_angle
    
    # Clamp the angle to valid range
    target_angle = max(MIN_SERVO_ANGLE, min(MAX_SERVO_ANGLE, target_angle))
    
    # Limit movement speed by constraining max change per update
    max_movement = 10  # Maximum degrees change per update
    if abs(target_angle - current_servo_angle) > max_movement:
        if target_angle > current_servo_angle:
            target_angle = current_servo_angle + max_movement
        else:
            target_angle = current_servo_angle - max_movement
    
    current_servo_angle = target_angle
    servo.angle = target_angle
    # No delays to prevent video lag

def tracking_logic(center_x):
    """Fast tracking logic using AngularServo"""
    global last_call_time, is_centered, current_servo_angle
    
    with tracking_lock:
        now = time.time()
        # Fast response time
        if now - last_call_time < 0.1:
            return
        last_call_time = now
        
        # Frame parameters - ADJUST THESE TO MATCH YOUR CAMERA
        frame_center = 320  # For 640px wide frame
        frame_width = 640   # Adjust based on your actual frame width
        tolerance = 60      # Dead zone around center
        
        # Calculate error from center
        error = center_x - frame_center
        
        logging.info(f"Person at x={center_x}, current_angle={current_servo_angle:.1f}�, error={error}")
        
        if abs(error) > tolerance:
            # Map person position to servo angle
            # When person is at left edge (x=0), servo should point left (angle=0)
            # When person is at right edge (x=width), servo should point right (angle=180)
            person_ratio = center_x / frame_width  # 0.0 to 1.0
            target_angle = person_ratio * 180  # Map to 0-180 degrees
            
            if center_x < frame_center - tolerance:
                logging.info(f"Tracking Left - Moving to angle: {target_angle:.1f}�")
            else:
                logging.info(f"Tracking Right - Moving to angle: {target_angle:.1f}�")
            
            # Use instant movement to prevent video delays
            move_servo_instant(target_angle)
            is_centered = False
        else:
            logging.info("Person centered - No movement needed")
            is_centered = True

def return_to_center():
    """Return servo to center position smoothly"""
    global is_centered, current_servo_angle
    
    with tracking_lock:
        if not is_centered and abs(current_servo_angle - CENTER_SERVO_ANGLE) > 5:
            logging.info("Returning to center...")
            # Use smooth movement for returning to center (not time-critical)
            move_servo_smooth(CENTER_SERVO_ANGLE, steps=5)
            is_centered = True
        else:
            logging.info("Already centered")

def cleanup_servo():
    """Clean up servo - return to center and close"""
    try:
        logging.info("Cleaning up servo...")
        servo.angle = CENTER_SERVO_ANGLE
        time.sleep(0.5)
        servo.close()  # Important: close the servo to free resources
        logging.info("Servo cleanup completed")
    except Exception as e:
        logging.error(f"Cleanup error: {e}")

def test_servo():
    """Test servo movement across full range"""
    print("Testing AngularServo movement...")
    
    test_angles = [0, 45, 90, 135, 180, 90]
    
    for angle in test_angles:
        print(f"Moving to {angle}�")
        servo.angle = angle
        time.sleep(1.5)
    
    print("Test complete - servo at center")

def calibrate_servo():
    """Interactive servo calibration"""
    print("Servo Calibration Mode")
    print("Commands: 0-180 (set angle), 'q' (quit), 'c' (center)")
    
    servo.angle = 90  # Start at center
    
    while True:
        cmd = input("Enter angle (0-180) or command: ").strip().lower()
        
        if cmd == 'q':
            break
        elif cmd == 'c':
            servo.angle = 90
            print("Moved to center (90�)")
        else:
            try:
                angle = float(cmd)
                if 0 <= angle <= 180:
                    servo.angle = angle
                    print(f"Moved to {angle}�")
                else:
                    print("Angle must be between 0 and 180")
            except ValueError:
                print("Invalid input. Enter a number between 0-180, 'c' for center, or 'q' to quit")
    
    servo.angle = 90  # Return to center
    print("Calibration complete")

# Uncomment to test or calibrate:
# test_servo()
# calibrate_servo()