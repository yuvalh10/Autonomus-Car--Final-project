// CarStates.cpp - Fixed version for manual mode distance stopping

#include "CarStates.h"
#include "CarMovement.h"
#include "DistanceSensors.h"

// Constructor
CarStates::CarStates() 
    : currentMode(CarMode::MANUAL),
      coordsReceived(false),
      targetLatitude(0.0f),
      targetLongitude(0.0f),
      motorController(nullptr),
      sensors(nullptr),
      bluetoothSerial(nullptr),
      gpsSerial(nullptr) {
}

// Destructor
CarStates::~CarStates() {
    // Ensure car is stopped when object is destroyed
    if (motorController) {
        motorController->stopMotors();
    }
}

// Initialization methods
void CarStates::initCommunication() {
    Serial.begin(SpeedConfig::BAUD_RATE);
    
    // Initialize communication serials
    bluetoothSerial = &Serial3;  // BLUETOOTH
    gpsSerial = &Serial2;        // GPS
    
    bluetoothSerial->begin(SpeedConfig::BAUD_RATE);
    gpsSerial->begin(SpeedConfig::BAUD_RATE);
    Wire.begin();
    
    Serial.println("Communication initialized:");
    Serial.print("Serial: ");
    Serial.println(SpeedConfig::BAUD_RATE);
    Serial.print("Bluetooth (Serial3): ");
    Serial.println(SpeedConfig::BAUD_RATE);
    Serial.print("GPS (Serial2): ");
    Serial.println(SpeedConfig::BAUD_RATE);
}

void CarStates::initLightsnBuzzer() {
    pinMode(PinConfig::BUZZER_PIN, OUTPUT);
    Serial.println("Buzzer initialized");
}

void CarStates::initialize() {
    initCommunication();
    initLightsnBuzzer();
}

// Dependency injection
void CarStates::setMotorController(CarMovement* controller) {
    motorController = controller;
}

void CarStates::setSensors(DistanceSensors* sensorInstance) {
    sensors = sensorInstance;
}

// Mode management
CarMode CarStates::getCurrentMode() const {
    return currentMode;
}

CarMode& CarStates::getCurrentModeRef() {
    return currentMode;
}

CarMode* CarStates::getModePtr() {
    return &currentMode;
}

void CarStates::setMode(CarMode mode) {
    currentMode = mode;
}

bool CarStates::isAutoMode() const {
    return currentMode == CarMode::AUTOMATIC;
}

bool CarStates::isManualMode() const {
    return currentMode == CarMode::MANUAL;
}

// GPS and coordinate management
TinyGPSPlus* CarStates::getGPS() {
    return &gps;
}

bool CarStates::areCoordsReceived() const {
    return coordsReceived;
}

float CarStates::getTargetLatitude() const {
    return targetLatitude;
}

float CarStates::getTargetLongitude() const {
    return targetLongitude;
}

bool* CarStates::getCoordsReceivedFlag() {
    return &coordsReceived;
}

float* CarStates::getTargetLatitudePtr() {
    return &targetLatitude;
}

float* CarStates::getTargetLongitudePtr() {
    return &targetLongitude;
}

// Communication getters
HardwareSerial* CarStates::getBluetoothSerial() {
    return bluetoothSerial;
}

HardwareSerial* CarStates::getGPSSerial() {
    return gpsSerial;
}

// Utility methods
void CarStates::clearInputBuffer() {
    if (bluetoothSerial) {
        delay(200);
        while (bluetoothSerial->available()) {
            bluetoothSerial->read();
        }
    }
}

void CarStates::switchToAutoMode() {
    currentMode = CarMode::AUTOMATIC;
    Serial.println("=== AUTO MODE ===");
}

void CarStates::switchToManualMode() {
    currentMode = CarMode::MANUAL;
    coordsReceived = false;
    if (motorController) {
        motorController->stopMotors();
    }
    Serial.println("=== MANUAL MODE ===");
}

// Streamlined auto driving with integrated sensor processing
void CarStates::autoDriving() {
    Serial.println("=== AUTO MODE ===");
    Serial.print("Min distance: ");
    Serial.print(NavigationConfig::MIN_DISTANCE_AUTO);
    Serial.println("m");
    
    while (isAutoMode() && coordsReceived) {
        if (gpsUpdateFlag && motorController) {
            gpsUpdateFlag = false;
            motorController->updateGPSData();
        }

        if (motorController) {
            motorController->gpsNavigation();  // Navigation logic runs continuously
        }

        if (bluetoothSerial && bluetoothSerial->available() > 0 && bluetoothSerial->read() == 'x') {
            switchToManualMode();
            break;
        }
    }
}

// FIXED: Better proximity checking for manual mode with alert clearing
bool CarStates::isForwardMovementSafe() {
    if (!sensors) {
        return true; // If no sensors, allow movement (fallback)
    }
    
    // Clear any previous proximity alerts before checking
    sensors->clearProximityAlert();
    
    // Get fresh sensor readings
    SensorDistances distances = sensors->getAllSensorDistances();
    float minDist = sensors->getCurrentMinDistance();
    
    // Check if any sensor detects an obstacle
    bool centerBlocked = distances.center <= minDist;
    bool leftBlocked = distances.left <= minDist;
    bool rightBlocked = distances.right <= minDist;
    
    if (centerBlocked || leftBlocked || rightBlocked)
        return false;
    return true;
}

// FIXED: Simplified forward command execution
void CarStates::executeForwardCommand(void (CarMovement::*driveFunc)(int), int speed, const char* cmdName) {
    if (!motorController) return;
    
    // Check if movement is safe BEFORE executing
    if (isForwardMovementSafe()) {
        (motorController->*driveFunc)(speed);
        Serial.print(cmdName);
        Serial.println(" - executing");
    } else {
        Serial.print(cmdName);
        Serial.println(" - BLOCKED by obstacle!");
        motorController->stopMotors();
    }
}

// Process individual manual commands
void CarStates::processManualCommand(char cmd) {
    if (!motorController) return;
    
    // Group commands by safety requirements
    switch (cmd) {
        // Forward movements - require proximity check
        case 'F':
            executeForwardCommand(&CarMovement::driveForward, SpeedConfig::FULL_SPEED, "Forward");
            break;
        case 'G':
            executeForwardCommand(&CarMovement::driveForwardLeft, SpeedConfig::FULL_SPEED / 2, "Forward-left");
            break;
        case 'H':
            executeForwardCommand(&CarMovement::driveForwardRight, SpeedConfig::FULL_SPEED / 2, "Forward-right");
            break;
            
        // Safe movements - no proximity check needed, but clear alerts
        case 'B': 
            if (sensors) sensors->clearProximityAlert(); // Clear alerts when moving away
            motorController->driveBackward(SpeedConfig::FULL_SPEED); 
            break;
        case 'R': 
            if (sensors) sensors->clearProximityAlert(); // Clear alerts when turning
            motorController->driveRight(SpeedConfig::FULL_SPEED); 
            break;
        case 'L': 
            if (sensors) sensors->clearProximityAlert(); // Clear alerts when turning
            motorController->driveLeft(SpeedConfig::FULL_SPEED); 
            break;
        case 'I': 
            if (sensors) sensors->clearProximityAlert(); // Clear alerts when moving away
            motorController->driveBackwardLeft(SpeedConfig::FULL_SPEED / 2); 
            break;
        case 'J': 
            if (sensors) sensors->clearProximityAlert(); // Clear alerts when moving away
            motorController->driveBackwardRight(SpeedConfig::FULL_SPEED / 2); 
            break;
            
        // System commands
        case 'S': 
            motorController->stopMotors(); 
            if (sensors) sensors->clearProximityAlert(); // Clear alerts when stopping
            break;
        case 'Y': 
            motorController->honk(); 
            break;
        case 'X':
            switchToAutoMode();
            break;
            
        default:
            // Unknown command - do nothing
            break;
    }
}

// Optimized manual driving with unified command processing
void CarStates::manualDriving() {
    Serial.println("=== MANUAL MODE ===");
    Serial.print("Min distance: ");
    Serial.print(NavigationConfig::MIN_DISTANCE_MANUAL);
    Serial.println("m");
    
    if (sensors) {
        sensors->enableProximityMonitoring(true);
    }
    
    while (isManualMode()) {
        if (!bluetoothSerial || !bluetoothSerial->available()) {
            continue;
        }
        
        char cmd = bluetoothSerial->read();
        processManualCommand(cmd);
    }
}

// Simplified coordinate selection with preset locations
void CarStates::requestCoordinates() {
    coordsReceived = false;
    
    if (!bluetoothSerial) {
        Serial.println("ERROR: Bluetooth not initialized");
        return;
    }
    
    // Clear input buffer
    clearInputBuffer();
    
    Serial.println("Select destination:");
    for (int i = 0; i < DestinationConfig::NUM_DESTINATIONS; i++) {
        const Destination& dest = DestinationConfig::getDestination(i);
        Serial.print(i + 1);
        Serial.print(". ");
        Serial.println(dest.name);
    }
    
    while (!coordsReceived) {
        while (!bluetoothSerial->available()) {
            // Wait for input
        }
        
        char option = bluetoothSerial->read();
        int index = option - '1'; // Convert '1'-'4' to 0-3
        
        if (index >= 0 && index < DestinationConfig::NUM_DESTINATIONS) {
            const Destination& dest = DestinationConfig::getDestination(index);
            targetLatitude = dest.lat;
            targetLongitude = dest.lng;
            coordsReceived = true;
            
            Serial.print("Destination selected: ");
            Serial.println(dest.name);
            Serial.print("Coordinates: ");
            Serial.print(targetLatitude, 8);
            Serial.print(", ");
            Serial.println(targetLongitude, 8);
        }
    }
}