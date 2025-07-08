// DistanceSensors.cpp - Fixed version for manual mode obstacle detection

#include "DistanceSensors.h"
#include "CarMovement.h"  // Forward declaration resolved

// Static instance pointer for ISR access
DistanceSensors* DistanceSensors::instance = nullptr;

// Constructor
DistanceSensors::DistanceSensors() 
    : leftEchoStartTime(0),
      leftEchoDuration(0),
      leftMeasurementReady(false),
      rightEchoStartTime(0),
      rightEchoDuration(0),
      rightMeasurementReady(false),
      proximityAlert(false),
      leftSideAlert(false),
      rightSideAlert(false),
      currentMode(nullptr),
      motorController(nullptr) {
    
    // Set static instance for ISR access
    instance = this;
}

// Destructor
DistanceSensors::~DistanceSensors() {
    enableProximityMonitoring(false);
    if (instance == this) {
        instance = nullptr;
    }
}

// Initialization methods
void DistanceSensors::initRaspberry() {
    pinMode(PinConfig::PERSON_DETECT_PIN, INPUT);
}

void DistanceSensors::initUSSensorPins() {
    // Left ultrasonic sensor
    pinMode(PinConfig::TRIG_LEFT, OUTPUT);
    pinMode(PinConfig::ECHO_LEFT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PinConfig::ECHO_LEFT), leftEchoISRWrapper, CHANGE);
    
    // Right ultrasonic sensor
    pinMode(PinConfig::TRIG_RIGHT, OUTPUT);
    pinMode(PinConfig::ECHO_RIGHT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PinConfig::ECHO_RIGHT), rightEchoISRWrapper, CHANGE);
    
    Serial.println("Ultrasonic sensors initialized:");
    Serial.print("Left US: Trigger=");
    Serial.print(PinConfig::TRIG_LEFT);
    Serial.print(", Echo=");
    Serial.println(PinConfig::ECHO_LEFT);
    Serial.print("Right US: Trigger=");
    Serial.print(PinConfig::TRIG_RIGHT);
    Serial.print(", Echo=");
    Serial.println(PinConfig::ECHO_RIGHT);
    Serial.println("Center: LIDAR (I2C)");
    
    enableProximityMonitoring(true);
}

void DistanceSensors::initLidarSensors() {
    centerLidar.begin(0, true);  // (address = 0x62, fast I2C = true)
    centerLidar.configure(0);    // Default mode: Balanced performance
    Serial.println("Center LIDAR sensor initialized (I2C)");
}

void DistanceSensors::initCompass() {
    compass.setWire(&Wire);
    compass.beginMag();  // Only initialize magnetometer for heading
    Serial.println("MPU9250 compass initialized");
}

void DistanceSensors::initialize() {
    initRaspberry();
    initUSSensorPins();
    initLidarSensors();
    initCompass();
}

// Setup methods
void DistanceSensors::setCarMode(CarMode* mode) {
    currentMode = mode;
}

void DistanceSensors::setMotorController(CarMovement* controller) {
    motorController = controller;
}

// Static ISR wrappers
void DistanceSensors::leftEchoISRWrapper() {
    if (instance) {
        instance->leftEchoISR();
    }
}

void DistanceSensors::rightEchoISRWrapper() {
    if (instance) {
        instance->rightEchoISR();
    }
}

// FIXED: Instance ISR handlers - less aggressive in manual mode
void DistanceSensors::leftEchoISR() {
    if (digitalRead(PinConfig::ECHO_LEFT) == HIGH) {
        leftEchoStartTime = micros();
    } else {
        leftEchoDuration = micros() - leftEchoStartTime;
        leftMeasurementReady = true;
        
        // Calculate distance immediately
        float distance = (leftEchoDuration * 0.034f) / 2.0f / 100.0f; // Convert to meters
        float currentMinDist = getCurrentMinDistance();
        
        if (distance <= currentMinDist && distance > 0.02f) { // Ignore very close readings (noise)
            leftSideAlert = true;  // Set side alert flag
            
            // Only emergency stop in AUTO mode for side sensors
            // In manual mode, just log and set flag for next movement check
            if (currentMode && *currentMode == CarMode::AUTOMATIC) {
                proximityAlert = true;
                Serial.println("Left side obstacle detected (AUTO) - emergency stop");
                emergencyStopISR();
            } else {
                Serial.println("Left side obstacle detected (MANUAL) - will block next forward command");
                proximityAlert = true; // Set flag but don't emergency stop
            }
        }
    }
}

void DistanceSensors::rightEchoISR() {
    if (digitalRead(PinConfig::ECHO_RIGHT) == HIGH) {
        rightEchoStartTime = micros();
    } else {
        rightEchoDuration = micros() - rightEchoStartTime;
        rightMeasurementReady = true;
        
        // Calculate distance immediately
        float distance = (rightEchoDuration * 0.034f) / 2.0f / 100.0f; // Convert to meters
        float currentMinDist = getCurrentMinDistance();
        
        if (distance <= currentMinDist && distance > 0.02f) { // Ignore very close readings (noise)
            rightSideAlert = true;  // Set side alert flag
            
            // Only emergency stop in AUTO mode for side sensors
            // In manual mode, just log and set flag for next movement check
            if (currentMode && *currentMode == CarMode::AUTOMATIC) {
                proximityAlert = true;
                Serial.println("Right side obstacle detected (AUTO) - emergency stop");
                emergencyStopISR();
            } else {
                Serial.println("Right side obstacle detected (MANUAL) - will block next forward command");
                proximityAlert = true; // Set flag but don't emergency stop
            }
        }
    }
}

// Emergency stop function for ISR safety
void DistanceSensors::emergencyStopISR() {
    // This is called from ISR - keep it minimal and fast
    if (motorController) {
        motorController->emergencyStop();
    }
}

// Private helper methods
void DistanceSensors::triggerUltrasonicMeasurement(int trigPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
}

// FIXED: Center LIDAR - emergency stop only in AUTO mode, flag-only in MANUAL
float DistanceSensors::getCenterLidarDistance() {
    float totalDistance = 0;
    int validReadings = 0;
    
    for (int i = 0; i < NavigationConfig::SAMPLES; i++) {
        float reading = centerLidar.distance() / 100.0f; // Convert mm to meters
        
        // Check if reading is within valid range
        if (reading > 0.02f && reading < 40.0f) { // Valid range
            totalDistance += reading;
            validReadings++;
            
            // Check for immediate center obstacle
            float currentMinDist = getCurrentMinDistance();
            if (reading <= currentMinDist) {
                proximityAlert = true;
                
                if (currentMode && *currentMode == CarMode::AUTOMATIC) {
                    Serial.println("CENTER obstacle detected (AUTO) - EMERGENCY STOP!");
                    emergencyStopISR();  // Emergency stop in auto mode
                } else {
                    Serial.println("CENTER obstacle detected (MANUAL) - will block next forward command");
                    // In manual mode, just set flag, don't emergency stop
                }
                break;
            }
        }
        delay(10); // Small delay between readings
    }
    
    if (validReadings == 0) {
        return 4.0f; // Return safe distance if no valid readings
    }
    
    return totalDistance / validReadings;
}

// Public measurement methods
void DistanceSensors::triggerAllUltrasonicMeasurements() {
    leftMeasurementReady = false;
    rightMeasurementReady = false;
    leftSideAlert = false;   // Clear side alerts
    rightSideAlert = false;
    
    // Trigger left sensor
    triggerUltrasonicMeasurement(PinConfig::TRIG_LEFT);
    delayMicroseconds(50); // Small delay between triggers
    
    // Trigger right sensor
    triggerUltrasonicMeasurement(PinConfig::TRIG_RIGHT);
}

float DistanceSensors::getDistanceUS(int trigger, int echo) {
    bool isLeftSensor = (echo == PinConfig::ECHO_LEFT);
    volatile bool* measurementReady = isLeftSensor ? &leftMeasurementReady : &rightMeasurementReady;
    volatile unsigned long* echoDuration = isLeftSensor ? &leftEchoDuration : &rightEchoDuration;
    
    // Trigger measurement
    *measurementReady = false;
    triggerUltrasonicMeasurement(trigger);
    
    // Wait for measurement to complete (with timeout)
    unsigned long startWait = millis();
    while (!*measurementReady && (millis() - startWait) < 30) {
        delayMicroseconds(100);
    }
    
    if (!*measurementReady) {
        return 4.0f; // Timeout - return max range
    }
    
    // Calculate distance in meters
    float distance = (*echoDuration * 0.034f) / 2.0f / 100.0f;
    *measurementReady = false; // Reset flag
    
    // Filter out unrealistic readings
    if (distance < 0.02f || distance > 4.0f) {
        return 4.0f; // Return max range for invalid readings
    }
    
    return distance;
}

SensorDistances DistanceSensors::getAllSensorDistances() {
    SensorDistances distances;
    
    // Get LEFT ultrasonic distance
    distances.left = getDistanceUS(PinConfig::TRIG_LEFT, PinConfig::ECHO_LEFT);
    
    // Get CENTER LIDAR distance
    distances.center = getCenterLidarDistance();
    
    // Get RIGHT ultrasonic distance
    distances.right = getDistanceUS(PinConfig::TRIG_RIGHT, PinConfig::ECHO_RIGHT);
    
    return distances;
}

float DistanceSensors::getAverageHeading() {
    float sinSum = 0;
    float cosSum = 0;
    
    // Take multiple readings for accuracy
    for (int i = 0; i < NavigationConfig::SAMPLES; i++) {
        compass.magUpdate();
        
        // Calculate heading from magnetometer data
        float heading = atan2(compass.magY(), compass.magX()) * 180.0f / PI;
        
        // Convert to radians for circular averaging
        float headingRad = heading * PI / 180.0f;
        
        // Accumulate sine and cosine components
        sinSum += sin(headingRad);
        cosSum += cos(headingRad);
        
        delay(50); // Short delay between readings
    }
    
    // Calculate average using circular averaging
    float avgHeadingRad = atan2(sinSum, cosSum);
    
    // Convert back to degrees (0-360)
    float avgHeading = avgHeadingRad * 180.0f / PI;
    if (avgHeading < 0) avgHeading += 360.0f;
    
    return avgHeading;
}

// Continuous monitoring
void DistanceSensors::continuousProximityCheck() {
    static unsigned long lastCheck = 0;
    unsigned long currentTime = millis();
    
    // Check every 50ms instead of waiting for manual triggers
    if (currentTime - lastCheck >= 50) {
        triggerAllUltrasonicMeasurements();
        
        // FIXED: Check center LIDAR for BOTH modes
        getCenterLidarDistance(); // This will trigger emergency stop if needed in any mode
        
        lastCheck = currentTime;
    }
}

void DistanceSensors::enableProximityMonitoring(bool enable) {
    if (enable) {
        attachInterrupt(digitalPinToInterrupt(PinConfig::ECHO_LEFT), leftEchoISRWrapper, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PinConfig::ECHO_RIGHT), rightEchoISRWrapper, CHANGE);
        Serial.println("Proximity monitoring ENABLED");
    } else {
        detachInterrupt(digitalPinToInterrupt(PinConfig::ECHO_LEFT));
        detachInterrupt(digitalPinToInterrupt(PinConfig::ECHO_RIGHT));
        Serial.println("Proximity monitoring DISABLED");
    }
}

// Alert management
bool DistanceSensors::isProximityAlertActive() const {
    return proximityAlert;
}

bool DistanceSensors::isLeftSideBlocked() const {
    return leftSideAlert;
}

bool DistanceSensors::isRightSideBlocked() const {
    return rightSideAlert;
}

// FIXED: Don't clear the flag when checking - use a separate method for clearing
bool DistanceSensors::checkProximityAlert() {
    return proximityAlert; // Just return the flag status without clearing
}

void DistanceSensors::clearProximityAlert() {
    proximityAlert = false;
    leftSideAlert = false;
    rightSideAlert = false;
}

// Utility functions
float DistanceSensors::getCurrentMinDistance() const {
    if (!currentMode) {
        return NavigationConfig::MIN_DISTANCE_MANUAL; // Default to manual mode
    }
    
    return (*currentMode == CarMode::AUTOMATIC) ? 
           NavigationConfig::MIN_DISTANCE_AUTO : 
           NavigationConfig::MIN_DISTANCE_MANUAL;
}