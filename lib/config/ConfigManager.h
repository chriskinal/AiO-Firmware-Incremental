#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "Arduino.h"
#include "EEPROM.h"
#include "IPAddress.h"

// Network configuration structure
struct NetConfigStruct
{
    static constexpr uint8_t defaultIP[5] = {192, 168, 5, 126};
    uint8_t currentIP[5] = {192, 168, 5, 126};
    uint8_t gatewayIP[5] = {192, 168, 5, 1};
    uint8_t broadcastIP[5] = {192, 168, 5, 255};
};

// GPS configuration structure
struct GPSConfigStruct
{
    char gpsSync[12];
    bool gpsPass = false;
};

// Autosteer configuration structure
struct SteerConfigStruct
{
    uint8_t InvertWAS = 0;
    uint8_t IsRelayActiveHigh = 0;
    uint8_t MotorDriveDirection = 0;
    uint8_t SingleInputWAS = 1;
    uint8_t CytronDriver = 1;
    uint8_t SteerSwitch = 0;
    uint8_t SteerButton = 0;
    uint8_t ShaftEncoder = 0;
    uint8_t PressureSensor = 0;
    uint8_t CurrentSensor = 0;
    uint8_t PulseCountMax = 3;
    uint8_t IsDanfoss = 0;
    uint8_t IsUseY_Axis = 0;
    uint8_t MinSpeed = 0;
};

// Autosteer settings structure
struct SteerSettingsStruct
{
    uint8_t Kp = 40;
    uint8_t lowPWM = 10;
    int16_t wasOffset = 0;
    uint8_t minPWM = 9;
    uint8_t highPWM = 150;
    float steerSensorCounts = 120;
    float AckermanFix = 1;
};

class ConfigManager
{
public:
    ConfigManager();
    ~ConfigManager();

    // Initialization
    void begin();

    // Network configuration
    const NetConfigStruct &getNetConfig() const { return netConfig_; }
    void setNetConfig(const NetConfigStruct &config);
    void saveNetConfig();
    void loadNetConfig();
    void resetNetConfigToDefaults();

    // GPS configuration
    const GPSConfigStruct &getGPSConfig() const { return gpsConfig_; }
    void setGPSConfig(const GPSConfigStruct &config);
    void saveGPSConfig();
    void loadGPSConfig();
    void resetGPSConfigToDefaults();

    // Autosteer configuration
    const SteerConfigStruct &getSteerConfig() const { return steerConfig_; }
    void setSteerConfig(const SteerConfigStruct &config);
    void saveSteerConfig();
    void loadSteerConfig();
    void resetSteerConfigToDefaults();

    // Autosteer settings
    const SteerSettingsStruct &getSteerSettings() const { return steerSettings_; }
    void setSteerSettings(const SteerSettingsStruct &settings);
    void saveSteerSettings();
    void loadSteerSettings();
    void resetSteerSettingsToDefaults();

    // Utility functions
    uint32_t ipv4ToUint32(const uint8_t ip[4]) const;
    void uint32ToIpv4(uint32_t ip, uint8_t result[4]) const;
    bool isConfigValid() const;
    void resetAllToDefaults();

private:
    // EEPROM addresses
    static constexpr uint16_t EE_VER_ADDR = 1;
    static constexpr uint16_t NET_CONFIG_ADDR = 300;
    static constexpr uint16_t GPS_CONFIG_ADDR = 400;
    static constexpr uint16_t STEER_SETTINGS_ADDR = 100;
    static constexpr uint16_t STEER_CONFIG_ADDR = 200;
    static constexpr uint16_t EE_VERSION = 2404;

    // Configuration data
    NetConfigStruct netConfig_;
    GPSConfigStruct gpsConfig_;
    SteerConfigStruct steerConfig_;
    SteerSettingsStruct steerSettings_;

    // Default configurations
    static const NetConfigStruct defaultNetConfig_;
    static const GPSConfigStruct defaultGPSConfig_;
    static const SteerConfigStruct defaultSteerConfig_;
    static const SteerSettingsStruct defaultSteerSettings_;

    // Helper functions
    bool checkEEPROMVersion();
    void writeEEPROMVersion();
    void updateNetworkDerivedValues();
};

#endif // CONFIG_MANAGER_H