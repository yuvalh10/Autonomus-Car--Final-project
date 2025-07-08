#ifndef CONFIG_H
#define CONFIG_H

#include <math.h>
#include <Wire.h>
#include <AFMotor.h>
#include <LIDARLite.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <MPU9250_asukiaaa.h>
#include <TimerOne.h>

// Global flag for GPS timer interrupt
extern volatile bool gpsUpdateFlag;

// Timer ISR function
void gpsTimerISR();

// Speed constants
class SpeedConfig {
public:
    static const int FULL_SPEED = 255;
    static const int BASE_SPEED = 255;
    static const int NAV_SPEED = 235;
    static const int BAUD_RATE = 9600;
};

// Timer configuration
class TimerConfig {
public:
    static const unsigned long GPS_UPDATE_INTERVAL_US = 5000000; // 5 seconds in microseconds
};

// Hardware pin configuration
class PinConfig {
public:
    // Communication
    static const int GPS_SERIAL = 2;     // Serial2
    static const int BLUETOOTH_SERIAL = 3; // Serial3

    // Hardware pins
    static const int BUZZER_PIN = 26;
    static const int TRIG_LEFT = 22;
    static const int ECHO_LEFT = 18;
    static const int TRIG_RIGHT = 23;
    static const int ECHO_RIGHT = 19;
    static const int PERSON_DETECT_PIN = 38;
    
    // LED indicator pins
    static const int LED_LEFT = 44;   // Left turn indicator LED
    static const int LED_RIGHT = 45;  // Right turn indicator LED
};

// Navigation configuration
class NavigationConfig {
public:
    static const float ARRIVAL_THRESHOLD = 15.0f;   // meters
    static const float HEADING_TOLERANCE = 15.0f;   // degrees
    static const float MIN_DISTANCE_MANUAL = 0.3f;  // meters
    static const float MIN_DISTANCE_AUTO = 0.5f;    // meters
    static const int SAMPLES = 10;
};

// Destination data structure
struct Destination {
    const float lat;
    const float lng;
    const char* name;
};

// Predefined destinations
class DestinationConfig {
public:
    static const int NUM_DESTINATIONS = 4;

    // Static function that returns the destinations array
    static const Destination* getDestinations() {
        static const Destination destinations[] = {
            {32.82366258f, 35.06808999f, "Yuvals Street"},
            {32.82386562f, 35.06751569f, "Yuvals Backyard"},
            {31.26157f, 34.79954f, "Iras Home (BGU)"},
            {32.91365f, 35.28214f, "ORT Braude College"}
        };
        return destinations;
    }

    // Helper method to get a specific destination safely
    static const Destination& getDestination(int index) {
        const Destination* destinations = getDestinations();
        if (index >= 0 && index < NUM_DESTINATIONS) {
            return destinations[index];
        }
        return destinations[0]; // Default to first destination
    }
};

// Sensor data structure
struct SensorDistances {
    float left;
    float center;
    float right;

    SensorDistances() : left(0), center(0), right(0) {}
    SensorDistances(float l, float c, float r) : left(l), center(c), right(r) {}
};

// Navigation states
enum class NavState {
    NAVIGATING,
    OBSTACLE_DETECTED,
    MANEUVERING
};

// Car operation modes
enum class CarMode {
    MANUAL,
    AUTOMATIC
};

#endif // CONFIG_H