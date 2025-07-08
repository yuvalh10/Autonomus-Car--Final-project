import RPi.GPIO as GPIO
import hailo
from hailo_apps_infra.hailo_rpi_common import app_callback_class, get_caps_from_pad, get_numpy_from_buffer
import cv2
from gi.repository import Gst
import logging
import time

# Logging
logging.basicConfig(
    filename='detection_log.txt',
    filemode='a',
    format='%(asctime)s - %(levelname)s - %(message)s',
    level=logging.INFO
)

class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.GPIO_PIN_LED = 18        # LED pin for person detection
        self.GPIO_PIN_SIGNAL = 17   # Additional signal pin        
        self.last_person_time = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.GPIO_PIN_LED, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.GPIO_PIN_SIGNAL, GPIO.OUT, initial=GPIO.LOW)

    def send_person_signal(self):
        """Turn on LED and send signal when person is detected"""
        GPIO.output(self.GPIO_PIN_LED, GPIO.HIGH)
        GPIO.output(self.GPIO_PIN_SIGNAL, GPIO.HIGH)
        logging.info("Person detected - LED and signal activated")

    def clear_person_signal(self):
        """Turn off LED and clear signal when no person is detected"""
        GPIO.output(self.GPIO_PIN_LED, GPIO.LOW)
        GPIO.output(self.GPIO_PIN_SIGNAL, GPIO.LOW)
        logging.info("No person detected - LED and signal deactivated")

    def cleanup(self):
        """Clean up GPIO pins"""
        GPIO.output(self.GPIO_PIN_LED, GPIO.LOW)
        GPIO.output(self.GPIO_PIN_SIGNAL, GPIO.LOW)
        GPIO.cleanup()
        logging.info("GPIO cleanup completed")

def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    user_data.increment()
    format, width, height = get_caps_from_pad(pad)

    frame = None
    if user_data.use_frame and format and width and height:
        frame = get_numpy_from_buffer(buffer, format, width, height)

    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    person_detected = False

    for detection in detections:
        label = detection.get_label()
        confidence = detection.get_confidence()

        if label == "person":
            person_detected = True
            box = detection.get_bbox()
            if box and width:
                # Calculate bounding box coordinates
                xmin = int(box.xmin() * width)
                ymin = int(box.ymin() * height)
                ymax = int(box.ymax() * height)
                xmax = int(box.xmax() * width)
                
                center_x = (xmin + xmax) // 2
                center_y = (ymin + ymax) // 2
                
                # Log person detection details
                logging.info(f"Frame size: {width}x{height}")
                logging.info(f"Person box: ({xmin},{ymin}) to ({xmax},{ymax})")
                logging.info(f"Person detected at center_x: {center_x}, center_y: {center_y}, confidence: {confidence:.2f}")
                
                # Update last person detection time
                user_data.last_person_time = time.time()
                
            break  # Only process the first person detected

    # Control LED and signal based on person detection
    if person_detected:
        user_data.send_person_signal()
    else:
        user_data.clear_person_signal()

    # Display annotated frame if needed
    if user_data.use_frame and frame is not None:
        cv2.putText(frame, f"{user_data.new_function()} {user_data.new_variable}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        user_data.set_frame(frame)

    return Gst.PadProbeReturn.OK