#ifndef CAR_MOVEMENT_H
#define CAR_MOVEMENT_H

#include "Config.h"

// Forward declarations
class DistanceSensors;

class CarMovement {
private:
    // Motor objects
    AF_DCMotor motorFL;
    AF_DCMotor motorBL;
    AF_DCMotor motorBR;
    AF_DCMotor motorFR;
    
    // Navigation state
    NavState navState;
    unsigned long maneuverStartTime;
    bool maneuverLeft;
    
    // GPS tracking
    float lastKnownLat;
    float lastKnownLng;
    unsigned long lastGPSUpdateTime;
    bool hasValidLastPosition;
    
    // GPS and target coordinates
    TinyGPSPlus* gps;
    bool* coordsReceived;
    float* targetLatitude;
    float* targetLongitude;
    
    // Sensor reference
    DistanceSensors* sensors;
    
    // Current car mode
    CarMode* currentMode;
    
    // Private helper methods
    void executeDrive(int flDir, int frDir, int blDir, int brDir, int speed = SpeedConfig::BASE_SPEED);
    void performNormalNavigation();
    void checkDestinationWithLastKnownPosition();
    void handleObstacleDetection();
    void handleEmergencyManeuver();
    bool handleSideObstacles();
    bool isObstacleAhead();
    float normalizeAngle(float angle);
    float distance2Dest(float lat1, float lon1, float lat2, float lon2);
    float calculateBearing(float lat1, float lon1, float lat2, float lon2);
    
    // NEW: LED control methods
    void initializeLEDs();
    void turnOnLeftLED();
    void turnOnRightLED();
    void turnOffAllLEDs();
    
public:
    // Constructor
    CarMovement();
    
    // Destructor
    ~CarMovement();
    
    // Initialization
    void initialize();
    
    // Dependency injection
    void setGPS(TinyGPSPlus* gpsInstance, bool* coordsReceivedFlag, 
                float* targetLat, float* targetLng);
    void setSensors(DistanceSensors* sensorInstance);
    void setCarMode(CarMode* mode);
    
    // Motor control functions
    void setSpeed(int val);
    void stopMotors();
    void emergencyStop();  // For ISR calls
    void honk();
    bool isPin52Active();
    bool isPersonDetected();
    
    // Basic movement functions
    void driveForward(int speed = SpeedConfig::FULL_SPEED);
    void driveBackward(int speed = SpeedConfig::FULL_SPEED);
    void driveRight(int speed = SpeedConfig::FULL_SPEED);
    void driveLeft(int speed = SpeedConfig::FULL_SPEED);
    
    // Advanced movement functions for manual mode
    void driveForwardLeft(int speed = SpeedConfig::FULL_SPEED);
    void driveForwardRight(int speed = SpeedConfig::FULL_SPEED);
    void driveBackwardLeft(int speed = SpeedConfig::FULL_SPEED);
    void driveBackwardRight(int speed = SpeedConfig::FULL_SPEED);
    
    // Navigation functions
    void gpsNavigation();
    bool stopWithDistance();
    void continuousDrive(char command, void (CarMovement::*driveFunction)(int), int speed);
    void updateGPSData();
    
    // Utility functions
    bool isProximityBlocked();
};

#endif // CAR_MOVEMENT_H