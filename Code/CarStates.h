#ifndef CAR_STATES_H
#define CAR_STATES_H

#include "Config.h"

// Forward declarations
class CarMovement;
class DistanceSensors;

class CarStates {
private:
    // Current operating mode
    CarMode currentMode;
    
    // GPS and navigation state
    TinyGPSPlus gps;
    bool coordsReceived;
    float targetLatitude;
    float targetLongitude;
    
    // Component references
    CarMovement* motorController;
    DistanceSensors* sensors;
    
    // Communication streams
    HardwareSerial* bluetoothSerial;
    HardwareSerial* gpsSerial;
    
    // Private helper methods
    void executeForwardCommand(void (CarMovement::*driveFunc)(int), int speed, const char* cmdName);
    void processManualCommand(char cmd);
    bool isForwardMovementSafe();
    
public:
    // Constructor
    CarStates();
    
    // Destructor
    ~CarStates();
    
    // Initialization methods
    void initCommunication();
    void initLightsnBuzzer();
    void initialize();
    
    // Dependency injection
    void setMotorController(CarMovement* controller);
    void setSensors(DistanceSensors* sensorInstance);
    
    // Mode management
    CarMode getCurrentMode() const;
    CarMode& getCurrentModeRef();  // Returns reference for address taking
    CarMode* getModePtr();         // Returns pointer for address passing
    void setMode(CarMode mode);
    bool isAutoMode() const;
    bool isManualMode() const;
    
    // GPS and coordinate management
    TinyGPSPlus* getGPS();
    bool areCoordsReceived() const;
    float getTargetLatitude() const;
    float getTargetLongitude() const;
    bool* getCoordsReceivedFlag();
    float* getTargetLatitudePtr();
    float* getTargetLongitudePtr();
    
    // Main operation methodsz
    void autoDriving();
    void manualDriving();
    void requestCoordinates();
    
    // Utility methods
    void clearInputBuffer();
    void switchToAutoMode();
    void switchToManualMode();
    
    // Communication getters
    HardwareSerial* getBluetoothSerial();
    HardwareSerial* getGPSSerial();
};

#endif // CAR_STATES_H