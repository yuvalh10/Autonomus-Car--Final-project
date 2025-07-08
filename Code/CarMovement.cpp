#include "CarMovement.h"
#include "DistanceSensors.h"

volatile bool gpsUpdateFlag = false;

// Timer interrupt for GPS updates
void gpsTimerISR() {
    gpsUpdateFlag = true;
}

// Constructor
CarMovement::CarMovement() 
    : motorFL(1), motorBL(2), motorBR(3), motorFR(4),
      navState(NavState::NAVIGATING),
      maneuverStartTime(0),
      maneuverLeft(true),
      lastKnownLat(0.0f),
      lastKnownLng(0.0f),
      lastGPSUpdateTime(0),
      hasValidLastPosition(false),
      gps(nullptr),
      coordsReceived(nullptr),
      targetLatitude(nullptr),
      targetLongitude(nullptr),
      sensors(nullptr),
      currentMode(nullptr) {
}

// Destructor
CarMovement::~CarMovement() {
    stopMotors();
    turnOffAllLEDs(); // Make sure LEDs are off when destroyed
}

// Initialization
void CarMovement::initialize() {
    setSpeed(SpeedConfig::FULL_SPEED);
    stopMotors();
    
    // Initialize LED pins
    initializeLEDs();
    
    // Initialize person detection pin as input
    pinMode(PinConfig::PERSON_DETECT_PIN, INPUT);

    // Initialize GPS update timer
    Timer1.initialize(TimerConfig::GPS_UPDATE_INTERVAL_US);  // 3 seconds
    Timer1.attachInterrupt(gpsTimerISR);
}

// NEW: Pin 52 control method
bool CarMovement::isPin52Active() {
    return digitalRead(PinConfig::PERSON_DETECT_PIN) == HIGH;
}

// NEW: Person detection method
bool CarMovement::isPersonDetected() {
    return digitalRead(PinConfig::PERSON_DETECT_PIN) == HIGH;
}

// NEW: LED Control Functions
void CarMovement::initializeLEDs() {
    pinMode(PinConfig::LED_LEFT, OUTPUT);
    pinMode(PinConfig::LED_RIGHT, OUTPUT);
    turnOffAllLEDs();
    Serial.println("Turn indicator LEDs initialized.");
}

void CarMovement::turnOnLeftLED() {
    digitalWrite(PinConfig::LED_LEFT, HIGH);
    digitalWrite(PinConfig::LED_RIGHT, LOW);
}

void CarMovement::turnOnRightLED() {
    digitalWrite(PinConfig::LED_LEFT, LOW);
    digitalWrite(PinConfig::LED_RIGHT, HIGH);
}

void CarMovement::turnOffAllLEDs() {
    digitalWrite(PinConfig::LED_LEFT, LOW);
    digitalWrite(PinConfig::LED_RIGHT, LOW);
}

// Dependency injection
void CarMovement::setGPS(TinyGPSPlus* gpsInstance, bool* coordsReceivedFlag, 
                        float* targetLat, float* targetLng) {
    gps = gpsInstance;
    coordsReceived = coordsReceivedFlag;
    targetLatitude = targetLat;
    targetLongitude = targetLng;
}

void CarMovement::setSensors(DistanceSensors* sensorInstance) {
    sensors = sensorInstance;
}

void CarMovement::setCarMode(CarMode* mode) {
    currentMode = mode;
}

// Motor Control Functions
void CarMovement::setSpeed(int val) {
    motorFL.setSpeed(val);
    motorFR.setSpeed(val);
    motorBL.setSpeed(val);
    motorBR.setSpeed(val);
}

void CarMovement::stopMotors() {
    motorFL.run(RELEASE);
    motorFR.run(RELEASE);
    motorBL.run(RELEASE);
    motorBR.run(RELEASE);
    turnOffAllLEDs(); // Turn off LEDs when stopping
}

void CarMovement::emergencyStop() {
    // For ISR calls - keep minimal and fast
    motorFL.run(RELEASE);
    motorFR.run(RELEASE);
    motorBL.run(RELEASE);
    motorBR.run(RELEASE);
    turnOffAllLEDs(); // Turn off LEDs during emergency stop
}

void CarMovement::honk() {
    digitalWrite(PinConfig::BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(PinConfig::BUZZER_PIN, LOW);
    delay(80);
    digitalWrite(PinConfig::BUZZER_PIN, HIGH);
    delay(300);
    digitalWrite(PinConfig::BUZZER_PIN, LOW);
}

// Unified drive function with safety checks
void CarMovement::executeDrive(int flDir, int frDir, int blDir, int brDir, int speed) {
    // NEW: Check person detection first - highest priority
    if (isPersonDetected()) {
        Serial.println("PERSON DETECTED - stopping car");
        stopMotors();
        return;
    }
    
    if (currentMode && *currentMode == CarMode::MANUAL && sensors && sensors->isProximityAlertActive()) {
        stopMotors();
        return;
    }
    
    setSpeed(speed);
    motorFL.run(flDir);
    motorFR.run(frDir);
    motorBL.run(blDir);
    motorBR.run(brDir);
}

// Basic movement functions
void CarMovement::driveForward(int speed) {
    if (currentMode && (*currentMode == CarMode::AUTOMATIC || !isProximityBlocked())) {
        turnOffAllLEDs(); // No turn indicators for forward movement
        executeDrive(FORWARD, FORWARD, FORWARD, FORWARD, speed);
    }
}

void CarMovement::driveBackward(int speed) { 
    turnOffAllLEDs(); // No turn indicators for backward movement
    executeDrive(BACKWARD, BACKWARD, BACKWARD, BACKWARD, speed); 
}

// UPDATED: Turn functions with LED indicators
void CarMovement::driveRight(int speed) { 
    turnOnRightLED(); // Turn on right LED for right turn
    executeDrive(FORWARD, BACKWARD, FORWARD, BACKWARD, speed); 
}

void CarMovement::driveLeft(int speed) { 
    turnOnLeftLED(); // Turn on left LED for left turn
    executeDrive(BACKWARD, FORWARD, BACKWARD, FORWARD, speed); 
}

// Advanced movement functions for manual mode compatibility
void CarMovement::driveForwardLeft(int speed) {
    if (currentMode && *currentMode == CarMode::MANUAL && isProximityBlocked()) {
        stopMotors();
        return;
    }
    
    // NO LED indicators for forward-diagonal movements (as requested)
    turnOffAllLEDs();
    
    motorFL.setSpeed(speed * 0.7f);
    motorBL.setSpeed(speed * 0.7f);
    motorFR.setSpeed(speed);
    motorBR.setSpeed(speed);
    
    motorFL.run(FORWARD);
    motorFR.run(FORWARD);
    motorBL.run(FORWARD);
    motorBR.run(FORWARD);
}

void CarMovement::driveForwardRight(int speed) {
    if (currentMode && *currentMode == CarMode::MANUAL && isProximityBlocked()) {
        stopMotors();
        return;
    }
    
    // NO LED indicators for forward-diagonal movements (as requested)
    turnOffAllLEDs();
    
    motorFL.setSpeed(speed);
    motorBL.setSpeed(speed);
    motorFR.setSpeed(speed * 0.7f);
    motorBR.setSpeed(speed * 0.7f);
    
    motorFL.run(FORWARD);
    motorFR.run(FORWARD);
    motorBL.run(FORWARD);
    motorBR.run(FORWARD);
}

void CarMovement::driveBackwardLeft(int speed) {
    if (currentMode && *currentMode == CarMode::MANUAL && isProximityBlocked()) {
        stopMotors();
        return;
    }
    
    // NO LED indicators for backward-diagonal movements
    turnOffAllLEDs();
    
    motorFL.setSpeed(speed * 0.7f);
    motorBL.setSpeed(speed * 0.7f);
    motorFR.setSpeed(speed);
    motorBR.setSpeed(speed);
    
    motorFL.run(BACKWARD);
    motorFR.run(BACKWARD);
    motorBL.run(BACKWARD);
    motorBR.run(BACKWARD);
}

void CarMovement::driveBackwardRight(int speed) {
    if (currentMode && *currentMode == CarMode::MANUAL && isProximityBlocked()) {
        stopMotors();
        return;
    }
    
    // NO LED indicators for backward-diagonal movements
    turnOffAllLEDs();
    
    motorFL.setSpeed(speed);
    motorBL.setSpeed(speed);
    motorFR.setSpeed(speed * 0.7f);
    motorBR.setSpeed(speed * 0.7f);
    
    motorFL.run(BACKWARD);
    motorFR.run(BACKWARD);
    motorBL.run(BACKWARD);
    motorBR.run(BACKWARD);
}

// Lightweight GPS data update for timer-based call
void CarMovement::updateGPSData() {
    if (!gps) return;

    static unsigned long lastGPSTime = 0;
    static unsigned long lastDataReceived = 0;
    bool gpsUpdated = false;

    while (Serial2.available() > 0) {
        char c = Serial2.read();
        lastDataReceived = millis();
        if (gps->encode(c)) {
            gpsUpdated = true;
            lastGPSTime = millis();
        }
    }

    if (gpsUpdated && gps->location.isValid() && gps->location.age() < 5000) {
        lastKnownLat = gps->location.lat();
        lastKnownLng = gps->location.lng();
        lastGPSUpdateTime = millis();
        hasValidLastPosition = true;

        Serial.print("GPS Updated (Timer): ");
        Serial.print(lastKnownLat, 8);
        Serial.print(", ");
        Serial.println(lastKnownLng, 8);
    }
}

// Navigation Functions
void CarMovement::handleObstacleDetection() {
    stopMotors();
    delay(200);
    
    if (!sensors) return;
    
    SensorDistances distances = sensors->getAllSensorDistances();
    float minDist = sensors->getCurrentMinDistance();
    
    // Check if path is actually clear now
    if (distances.center > minDist * 1.8f) {
        Serial.println("False alarm - path is clear");
        navState = NavState::NAVIGATING;
        return;
    }
    
    // NEW: Check if ALL sensors are blocked
    bool leftBlocked = distances.left <= minDist;
    bool centerBlocked = distances.center <= minDist;
    bool rightBlocked = distances.right <= minDist;
    
    if (leftBlocked && centerBlocked && rightBlocked) {
        Serial.println("*** ALL SENSORS BLOCKED - STOPPING CAR ***");
        Serial.print("All distances below threshold - L:");
        Serial.print(distances.left);
        Serial.print(" C:");
        Serial.print(distances.center);
        Serial.print(" R:");
        Serial.print(distances.right);
        Serial.print(" (min: ");
        Serial.print(minDist);
        Serial.println(")");
        
        stopMotors();
        honk(); // Alert that car is stopped due to obstacles
        
        // Stay in OBSTACLE_DETECTED state - don't try to maneuver
        // Car will only resume if obstacles are cleared
        return;
    }
    
    // Check if we're in an emergency situation (very close obstacle)
    if (distances.center < minDist * 0.5f) {
        handleEmergencyManeuver();
        return;
    }
    
    // Normal obstacle - choose best maneuvering direction (only if not all blocked)
    Serial.println("Planning maneuver around obstacle");
    Serial.print("Distances - L:");
    Serial.print(distances.left);
    Serial.print(" C:");
    Serial.print(distances.center);
    Serial.print(" R:");
    Serial.println(distances.right);
    
    // Choose direction with more space
    if (distances.left > distances.right + 0.3f) {
        maneuverLeft = true;
        Serial.println("Turning LEFT - more space available");
    } else if (distances.right > distances.left + 0.3f) {
        maneuverLeft = false;
        Serial.println("Turning RIGHT - more space available");
    } else {
        maneuverLeft = distances.left > distances.right;
        Serial.println(maneuverLeft ? "Turning LEFT" : "Turning RIGHT");
    }
    
    navState = NavState::MANEUVERING;
    maneuverStartTime = millis();
}


// Smart obstacle avoidance - handles side obstacles efficiently
bool CarMovement::handleSideObstacles() {
    if (!sensors) return true;
    
    SensorDistances distances = sensors->getAllSensorDistances();
    float minDist = sensors->getCurrentMinDistance();
    
    bool leftBlocked = distances.left <= minDist;
    bool rightBlocked = distances.right <= minDist;
    bool centerBlocked = distances.center <= minDist;
    
    // Skip if center is blocked (let state machine handle)
    if (centerBlocked) return true;
    
    // Handle side obstacles
    if (leftBlocked && rightBlocked) {
        stopMotors();
        delay(500);
        return false;  // Both sides blocked
    }
    
    if (leftBlocked || rightBlocked) {
        // Clear, straightforward logic
        if (leftBlocked) {
            Serial.println("LEFT obstacle detected - turning RIGHT to avoid");
        } else if (rightBlocked) {
            Serial.println("RIGHT obstacle detected - turning LEFT to avoid");
        }
        
        // Quick avoidance maneuver - turn away from the obstacle
        for (int i = 0; i < 5 && (leftBlocked || rightBlocked); i++) {
            if (leftBlocked) driveRight(SpeedConfig::FULL_SPEED);  // LEFT blocked → turn RIGHT (LED will turn on)
            else if (rightBlocked) driveLeft(SpeedConfig::FULL_SPEED);   // RIGHT blocked → turn LEFT (LED will turn on)
            delay(300);
            stopMotors();
            delay(100);
            
            // Recheck sensors
            distances = sensors->getAllSensorDistances();
            leftBlocked = distances.left <= minDist;
            rightBlocked = distances.right <= minDist;
        }
        
        // Move forward while turning to clear obstacle ONLY if sides are clear
        distances = sensors->getAllSensorDistances();  // Re-read sensors after turning
        leftBlocked = distances.left <= minDist;
        rightBlocked = distances.right <= minDist;
        
        if (distances.center > minDist * 1.5f && !leftBlocked && !rightBlocked) {
            // Only move forward if center AND sides are clear
            for (int i = 0; i < 3; i++) {
                // Move forward straight since we've already turned away
                driveForward(SpeedConfig::FULL_SPEED);
                delay(400);
                stopMotors();
                delay(100);
                
                // Check if we're clear after each forward movement
                distances = sensors->getAllSensorDistances();
                leftBlocked = distances.left <= minDist;
                rightBlocked = distances.right <= minDist;
                
                // If sides become blocked again, stop forward movement
                if (leftBlocked || rightBlocked) {
                    break;
                }
            }
        } else {
            Serial.println("Cannot move forward - sides still blocked or center blocked");
        }
        
        delay(500);  // Settle time
        
        // FINAL CHECK: Re-read sensors to verify obstacle is cleared
        distances = sensors->getAllSensorDistances();
        leftBlocked = distances.left <= minDist;
        rightBlocked = distances.right <= minDist;
        
        // Only return true if sides are actually clear
        if (leftBlocked || rightBlocked) {
            Serial.println("Side obstacle still present after avoidance");
            return false;  // Still blocked, don't proceed with normal navigation
        }
        
        Serial.println("Side obstacle cleared successfully");
    }
    
    return true;  // No side obstacles or successfully cleared
}

// Obstacle Detection
bool CarMovement::isObstacleAhead() {
    if (!sensors) return false;
    
    SensorDistances distances = sensors->getAllSensorDistances();
    float minDist = sensors->getCurrentMinDistance();
    
    if (currentMode && *currentMode == CarMode::AUTOMATIC) {
        return distances.center <= minDist;  // Only check center in auto mode
    }
    return (distances.left <= minDist || distances.center <= minDist || distances.right <= minDist);
}

// Main GPS Navigation - simplified state machine
void CarMovement::gpsNavigation() {
    // NEW: Check person detection first - highest priority stop
    if (isPersonDetected()) {
        Serial.println("PERSON DETECTED - car stopped");
        stopMotors();
        return;
    }
    
    if (!coordsReceived || !*coordsReceived) {
        Serial.println("Waiting for GPS data...");
        return;
    }
    
    static bool wasManeuveringLastCycle = false;
    bool currentlyManeuvering = (navState == NavState::MANEUVERING);
    
    if (currentlyManeuvering && !wasManeuveringLastCycle) {
        Serial.println("Starting maneuver - GPS updates paused");
    } else if (!currentlyManeuvering && wasManeuveringLastCycle) {
        Serial.println("Maneuvering complete - resuming GPS updates");
    }
    
    wasManeuveringLastCycle = currentlyManeuvering;
    
    // Handle side obstacles first (but not during active maneuvering)
    if (!currentlyManeuvering && !handleSideObstacles()) {
        navState = NavState::OBSTACLE_DETECTED;
        maneuverStartTime = millis();
        return;
    }
    
    switch (navState) {
        case NavState::NAVIGATING:
            if (isObstacleAhead()) {
                navState = NavState::OBSTACLE_DETECTED;
                maneuverStartTime = millis();
                stopMotors();
                return;
            }
            
            // ONLY update GPS during NAVIGATING state
            performNormalNavigation();
            break;
            
        case NavState::OBSTACLE_DETECTED:
            // Use last known coordinates for destination checking
            checkDestinationWithLastKnownPosition();
            handleObstacleDetection();
            break;
            
        case NavState::MANEUVERING:
            // Use last known coordinates for destination checking
            checkDestinationWithLastKnownPosition();
            
            // Check for timeout
            if (millis() - maneuverStartTime > 10000) {
                Serial.println("Maneuvering timeout - trying different direction");
                maneuverLeft = !maneuverLeft;
                maneuverStartTime = millis();
            }
            
            if (!sensors) break;
            
            SensorDistances currentDistances = sensors->getAllSensorDistances();
            float minDist = sensors->getCurrentMinDistance();
            
            if (currentDistances.center > minDist * 1.8f) {
                Serial.println("Path cleared - resuming navigation");
                navState = NavState::NAVIGATING;
                stopMotors();
                delay(200);
                return;
            }
            
            // Continue maneuvering with LED indicators
            if (maneuverLeft) {
                driveLeft(SpeedConfig::FULL_SPEED);  // LED will turn on automatically
                Serial.println("Smooth left turn...");
            } else {
                driveRight(SpeedConfig::FULL_SPEED); // LED will turn on automatically
                Serial.println("Smooth right turn...");
            }
            delay(50);
            
            // Safety check for getting stuck
            static unsigned long lastProgressCheck = 0;
            static float lastCenterDistance = 0;
            
            if (millis() - lastProgressCheck > 3000) {
                if (abs(currentDistances.center - lastCenterDistance) < 0.1f) {
                    Serial.println("No progress - switching turn direction");
                    maneuverLeft = !maneuverLeft;
                    maneuverStartTime = millis();
                }
                lastCenterDistance = currentDistances.center;
                lastProgressCheck = millis();
            }
            break;
    }    
}

// Navigation Functions - WITH DEBUGGING
void CarMovement::performNormalNavigation() {
    if (!gps || !targetLatitude || !targetLongitude) return;
    
    // Read fresh GPS data ONLY during navigation
    bool gpsUpdated = false;
    static unsigned long lastGPSTime = 0;
    static unsigned long lastDataReceived = 0;
    int bytesProcessed = 0;
    
    // Process all available GPS data
    while (Serial2.available() > 0) {
        char c = Serial2.read();
        bytesProcessed++;
        lastDataReceived = millis(); // Track when we last received ANY data
        
        if (gps->encode(c)) {
            gpsUpdated = true;
            lastGPSTime = millis();
        }
    }
    
    // Debug: Show data reception
    if (bytesProcessed > 0) {
        Serial.print("GPS bytes processed: ");
        Serial.print(bytesProcessed);
        Serial.print(" | ");
    }
    
    // Get current coordinates directly from GPS
    float currentLat = gps->location.lat();
    float currentLng = gps->location.lng();
    
    // Only proceed if we have valid coordinates (check age instead of update frequency)
    if (!gps->location.isValid() || gps->location.age() > 5000) { // Data older than 5 seconds
        Serial.print("Waiting for fresh GPS data... Age: ");
        Serial.print(gps->location.age());
        Serial.println("ms");
        return;
    }
    
    // Store current coordinates as last known good position
    lastKnownLat = currentLat;
    lastKnownLng = currentLng;
    lastGPSUpdateTime = millis();
    hasValidLastPosition = true;
    
    // DEBUG: Show GPS update status
    Serial.print("GPS Update: ");
    Serial.print(gpsUpdated ? "YES" : "NO");
    Serial.print(" | Current Lat: ");
    Serial.print(currentLat, 8);
    Serial.print(", Lng: ");
    Serial.print(currentLng, 8);
    
    // Check how long since last GPS update (extended timeout for real GPS conditions)
    if (millis() - lastGPSTime > 10000) { // Changed to 10 seconds
        Serial.println(" | INFO: No GPS updates for 10+ seconds");
    } else {
        Serial.println("");
    }
    
    float distanceToTarget = distance2Dest(currentLat, currentLng, *targetLatitude, *targetLongitude);
    
    // DEBUG: Enhanced distance checking
    Serial.print("=== DISTANCE CHECK === Distance: ");
    Serial.print(distanceToTarget);
    Serial.print("m | Threshold: ");
    Serial.print(NavigationConfig::ARRIVAL_THRESHOLD);
    Serial.print("m | Within range: ");
    Serial.println(distanceToTarget <= NavigationConfig::ARRIVAL_THRESHOLD ? "YES - SHOULD STOP!" : "NO");
    
    // Check if destination reached
    if (distanceToTarget <= NavigationConfig::ARRIVAL_THRESHOLD) {
        Serial.println("*** DESTINATION REACHED! STOPPING! ***");
        honk();
        honk();
        stopMotors();
        return;
    }
    
    float bearingToTarget = calculateBearing(currentLat, currentLng, *targetLatitude, *targetLongitude);
    float currentHeading = sensors ? sensors->getAverageHeading() : 0.0f;
    float headingError = normalizeAngle(bearingToTarget - currentHeading);
    
    Serial.print("Heading error: ");
    Serial.print(headingError);
    Serial.print("°, Distance: ");
    Serial.print(distanceToTarget);
    Serial.println("m");
    
    // Execute movement based on heading error with LED indicators
    if (abs(headingError) <= NavigationConfig::HEADING_TOLERANCE) {
        // Heading is within tolerance, move forward
        driveForward(SpeedConfig::BASE_SPEED);
        Serial.println("Moving forward toward target");
    } else {
        // Heading error is outside tolerance, turn in place until corrected
        if (headingError > NavigationConfig::HEADING_TOLERANCE) {
            // Turn right in place (LED will turn on automatically)
            driveRight(SpeedConfig::NAV_SPEED);
            Serial.println("Turning right to correct heading");
        } else if (headingError < -NavigationConfig::HEADING_TOLERANCE) {
            // Turn left in place (LED will turn on automatically)
            driveLeft(SpeedConfig::NAV_SPEED);
            Serial.println("Turning left to correct heading");
        }
    }
}

// Function to check destination using last known coordinates
void CarMovement::checkDestinationWithLastKnownPosition() {
    if (!hasValidLastPosition || !targetLatitude || !targetLongitude) {
        return; // No valid last position to check
    }
    
    float distanceToTarget = distance2Dest(lastKnownLat, lastKnownLng, *targetLatitude, *targetLongitude);
    
    if (distanceToTarget <= NavigationConfig::ARRIVAL_THRESHOLD) {
        Serial.println("*** DESTINATION REACHED (using last known position)! STOPPING! ***");
        Serial.print("Distance from last known position: ");
        Serial.print(distanceToTarget);
        Serial.println("m");
        
        navState = NavState::NAVIGATING; // Reset state
        honk();
        honk();
        stopMotors();
        
        // Stay stopped
        while (true) {
            Serial.println("STOPPED AT DESTINATION");
            delay(1000);
        }
    }
}

void CarMovement::handleEmergencyManeuver() {
    Serial.println("Emergency maneuver - obstacle very close!");
    
    if (!sensors) return;
    
    // Quick assessment of best escape route
    SensorDistances distances = sensors->getAllSensorDistances();
    
    // Choose the side with more space
    bool turnLeft = distances.left > distances.right;
    
    Serial.print("Emergency turn ");
    Serial.println(turnLeft ? "LEFT" : "RIGHT");
    
    // Perform quick but smooth escape maneuver - turning in place only (LEDs will activate automatically)
    for (int i = 0; i < 15; i++) {  // Up to 3 seconds of turning
        if (turnLeft) driveLeft(SpeedConfig::FULL_SPEED);   // Left LED will turn on
        else driveRight(SpeedConfig::FULL_SPEED);           // Right LED will turn on
        
        delay(200);
        
        // Check if we've created enough space to move forward
        SensorDistances newDistances = sensors->getAllSensorDistances();
        if (newDistances.center > sensors->getCurrentMinDistance() * 2.0f) {
            Serial.println("Emergency turn successful - path clear");
            stopMotors();
            delay(300);
            navState = NavState::NAVIGATING;
            return;
        }
    }
    
    // If still stuck after turning, just wait and re-assess
    Serial.println("Could not clear obstacle by turning - waiting and re-assessing");
    stopMotors();
    delay(1000);
    navState = NavState::OBSTACLE_DETECTED;  // Re-assess situation
}

// Utility Functions
float CarMovement::normalizeAngle(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

bool CarMovement::stopWithDistance() {
    if (sensors && sensors->isProximityAlertActive()) {
        Serial.println("PROXIMITY ALERT: Stopping");
        return true;
    }
    
    if (!sensors) return false;
    
    SensorDistances distances = sensors->getAllSensorDistances();
    float minDist = sensors->getCurrentMinDistance();
    
    if (distances.left <= minDist || distances.center <= minDist || distances.right <= minDist) {
        Serial.println("Sensor detection: Stopping");
        return true;
    }
    
    return false;
}

// Continuous drive for manual mode compatibility
void CarMovement::continuousDrive(char command, void (CarMovement::*driveFunction)(int), int speed) {
    if (isProximityBlocked() || stopWithDistance()) {
        stopMotors();
        return;
    }
    
    (this->*driveFunction)(speed);
    delay(10);
    
    if (isProximityBlocked()) {
        stopMotors();
    }
}

bool CarMovement::isProximityBlocked() {
    return sensors && sensors->isProximityAlertActive();
}

// GPS Calculations
float CarMovement::distance2Dest(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371000.0f; // Earth radius in meters
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);
    
    float a = sin(dLat/2) * sin(dLat/2) +
              cos(radians(lat1)) * cos(radians(lat2)) *
              sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

float CarMovement::calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    lat1 = radians(lat1);
    lon1 = radians(lon1);
    lat2 = radians(lat2);
    lon2 = radians(lon2);
    
    float y = sin(lon2 - lon1) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
    float bearing = degrees(atan2(y, x));
    
    return fmod((bearing + 360.0f), 360.0f);
}