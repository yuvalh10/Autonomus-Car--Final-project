#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include "Config.h"

// Forward declarations
class CarMovement;

class DistanceSensors {
private:
    // Sensor objects
    LIDARLite centerLidar;
    MPU9250_asukiaaa compass;
    
    // Volatile variables for interrupt-based ultrasonic measurement
    volatile unsigned long leftEchoStartTime;
    volatile unsigned long leftEchoDuration;
    volatile bool leftMeasurementReady;
    
    volatile unsigned long rightEchoStartTime;
    volatile unsigned long rightEchoDuration;
    volatile bool rightMeasurementReady;
    
    // Alert flags
    volatile bool proximityAlert;
    volatile bool leftSideAlert;
    volatile bool rightSideAlert;
    
    // Current car mode for distance calculations
    CarMode* currentMode;
    
    // Reference to motor controller for emergency stops
    CarMovement* motorController;
    
    // Private helper methods
    void triggerUltrasonicMeasurement(int trigPin);
    float getCenterLidarDistance();
    void emergencyStopISR();
    
public:
    // Constructor
    DistanceSensors();
    
    // Destructor
    ~DistanceSensors();
    
    // Initialization methods
    void initRaspberry();
    void initUSSensorPins();
    void initLidarSensors();
    void initCompass();
    void initialize();  // Initialize all sensors
    
    // Sensor mode and motor controller setup
    void setCarMode(CarMode* mode);
    void setMotorController(CarMovement* controller);
    
    // ISR functions (need to be static for interrupt attachment)
    static void leftEchoISRWrapper();
    static void rightEchoISRWrapper();
    
    // Instance ISR handlers
    void leftEchoISR();
    void rightEchoISR();
    
    // Measurement functions
    void triggerAllUltrasonicMeasurements();
    float getDistanceUS(int trigger, int echo);
    SensorDistances getAllSensorDistances();
    float getAverageHeading();
    
    // Continuous monitoring
    void continuousProximityCheck();
    void enableProximityMonitoring(bool enable);
    
    // Alert management
    bool isProximityAlertActive() const;
    bool isLeftSideBlocked() const;
    bool isRightSideBlocked() const;
    bool checkProximityAlert();
    void clearProximityAlert();
    
    // Utility functions
    float getCurrentMinDistance() const;
    
    // Static instance pointer for ISR access
    static DistanceSensors* instance;
};

#endif // DISTANCE_SENSORS_H