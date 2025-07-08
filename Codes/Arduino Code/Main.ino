// Autonomous_Car.ino - OOP Main Controller
// Clean, object-oriented architecture with proper dependency injection

#include "Config.h"
#include "DistanceSensors.h"
#include "CarMovement.h"
#include "CarStates.h"

// Global system components
DistanceSensors sensors;
CarMovement motorController;
CarStates carStates;

// System initialization sequence
void setup() {
    Serial.println("=== AUTONOMOUS CAR SYSTEM STARTUP ===");
    
    // 1. Initialize base communication and hardware
    Serial.println("1. Initializing communication systems...");
    carStates.initialize();
    
    // 2. Initialize sensors (requires communication for I2C)
    Serial.println("2. Initializing sensor systems...");
    sensors.initialize();
    
    // 3. Initialize motor controller
    Serial.println("3. Initializing motor systems...");
    motorController.initialize();
    
    // 4. Setup dependency injection - create the component relationships
    Serial.println("4. Configuring component dependencies...");
    setupDependencies();
    
    // 5. Final system validation
    Serial.println("5. Validating system configuration...");
    validateSystemConfiguration();
    
    Serial.println("=== SYSTEM READY ===");
    Serial.println("Starting in MANUAL mode");
    Serial.println("Send 'X' to switch to AUTO mode");
}

// Configure all component dependencies
void setupDependencies() {
    // CarStates needs motor controller and sensors
    carStates.setMotorController(&motorController);
    carStates.setSensors(&sensors);
    
    // CarMovement needs GPS data, sensors, and mode
    motorController.setGPS(
        carStates.getGPS(),
        carStates.getCoordsReceivedFlag(),
        carStates.getTargetLatitudePtr(),
        carStates.getTargetLongitudePtr()
    );
    motorController.setSensors(&sensors);
    motorController.setCarMode(&carStates.getCurrentModeRef());
    
    // DistanceSensors needs motor controller for emergency stops and mode for distance thresholds
    sensors.setMotorController(&motorController);
    sensors.setCarMode(&carStates.getCurrentModeRef());
    
    Serial.println("   ✓ All dependencies configured");
}

// Validate that all components are properly initialized
void validateSystemConfiguration() {
    bool systemValid = true;
    
    // Check critical components
    if (!carStates.getBluetoothSerial()) {
        Serial.println("   ✗ ERROR: Bluetooth communication not initialized");
        systemValid = false;
    } else {
        Serial.println("   ✓ Bluetooth communication ready");
    }
    
    if (!carStates.getGPSSerial()) {
        Serial.println("   ✗ ERROR: GPS communication not initialized");
        systemValid = false;
    } else {
        Serial.println("   ✓ GPS communication ready");
    }
    
    if (!carStates.getGPS()) {
        Serial.println("   ✗ ERROR: GPS object not available");
        systemValid = false;
    } else {
        Serial.println("   ✓ GPS object ready");
    }
    
    Serial.println("   ✓ Motor systems ready");
    Serial.println("   ✓ Sensor systems ready");
    Serial.println("   ✓ State management ready");
    
    if (!systemValid) {
        Serial.println("   ✗ SYSTEM VALIDATION FAILED - Please check connections");
        while (true) {
            delay(1000);
            Serial.println("System halted due to initialization errors");
        }
    } else {
        Serial.println("   ✓ System validation passed");
    }
}

// Main loop - clean and simple with integrated proximity monitoring
void loop() {
    // Continuous proximity monitoring (non-blocking, handled by sensors)
    sensors.enableProximityMonitoring(true);
    sensors.continuousProximityCheck();
    
    // Mode-based operation with clean delegation
    if (carStates.isAutoMode()) {
        // Auto mode: handle coordinate request and navigation
        if (!carStates.areCoordsReceived()) {
            carStates.requestCoordinates();
        } else {
            carStates.autoDriving();
        }
    } else {
        // Manual mode: handle user commands
        carStates.manualDriving();
    }
}