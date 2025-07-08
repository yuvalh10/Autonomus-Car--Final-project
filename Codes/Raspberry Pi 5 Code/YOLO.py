import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import logging

from names_list import names
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp
from user_callback import user_app_callback_class, app_callback

# --- Configure Logging ---
logging.basicConfig(
    filename='detection_log.txt',
    filemode='a',
    format='%(asctime)s - %(levelname)s - %(message)s',
    level=logging.INFO
)

# Print and log available object classes
logging.info("Application started.")
logging.info("Available objects:")
print("Available objects:")
for idx, name in names.items():
    print(f"{idx}: {name}")
    logging.info(f"{idx}: {name}")

# --- Main entry point ---
if __name__ == "__main__":
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)

    try:
        logging.info("Running GStreamer Detection App.")
        app.run()
    except Exception as e:
        logging.exception("Exception occurred while running the detection app.")
    finally:
        user_data.cleanup()       # Clean up GPIO
        logging.info("Cleanup complete. Application exited.")