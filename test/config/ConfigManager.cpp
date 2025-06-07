#include "ConfigManager.h"
#include "mongoose_glue.h"

// Static default configurations
const NetConfigStruct ConfigManager::defaultNetConfig_ = {
    {192, 168, 5, 126}, // currentIP
    {192, 168, 5, 1},   // gatewayIP
    {192, 168, 5, 255}  // broadcastIP
};

const GPSConfigStruct ConfigManager::defaultGPSConfig_ = {
    "10ms-UM98x", // gpsSync
    false         // gpsPass
};

const SteerConfigStruct ConfigManager::defaultSteerConfig_ = {
    0, // InvertWAS
    0, // IsRelayActiveHigh
    0, // MotorDriveDirection
    1, // SingleInputWAS
    1, // CytronDriver
    0, // SteerSwitch
    0, // SteerButton
    0, // ShaftEncoder
    0, // PressureSensor
    0, // CurrentSensor
    3, // PulseCountMax
    0, // IsDanfoss
    0, // IsUseY_Axis
    0  // MinSpeed
};

const SteerSettingsStruct ConfigManager::defaultSteerSettings_ = {
    40,  // Kp
    10,  // lowPWM
    0,   // wasOffset
    9,   // minPWM
    150, // highPWM
    120, // steerSensorCounts
    1    // AckermanFix
};

ConfigManager::ConfigManager()
    : netConfig_(defaultNetConfig_), gpsConfig_(defaultGPSConfig_), steerConfig_(defaultSteerConfig_), steerSettings_(defaultSteerSettings_)
{
}

ConfigManager::~ConfigManager()
{
}

void ConfigManager::begin()
{
    if (!checkEEPROMVersion())
    {
        Serial.print("\r\n- EEPROM version mismatch, resetting to defaults");
        resetAllToDefaults();
        writeEEPROMVersion();
    }
    else
    {
        loadNetConfig();
        loadGPSConfig();
        loadSteerConfig();
        loadSteerSettings();
        Serial.print("\r\n- Configuration loaded from EEPROM");
    }
}

bool ConfigManager::checkEEPROMVersion()
{
    uint16_t storedVersion;
    EEPROM.get(EE_VER_ADDR, storedVersion);
    return storedVersion == EE_VERSION;
}

void ConfigManager::writeEEPROMVersion()
{
    EEPROM.put(EE_VER_ADDR, EE_VERSION);
}

void ConfigManager::setNetConfig(const NetConfigStruct &config)
{
    netConfig_ = config;
    updateNetworkDerivedValues();
}

void ConfigManager::saveNetConfig()
{
    updateNetworkDerivedValues();
    EEPROM.put(NET_CONFIG_ADDR, netConfig_);
}

void ConfigManager::loadNetConfig()
{
    EEPROM.get(NET_CONFIG_ADDR, netConfig_);
    updateNetworkDerivedValues();
}

void ConfigManager::resetNetConfigToDefaults()
{
    netConfig_ = defaultNetConfig_;
    updateNetworkDerivedValues();
}

void ConfigManager::updateNetworkDerivedValues()
{
    // Update gateway IP (same subnet, .1)
    netConfig_.gatewayIP[0] = netConfig_.currentIP[0];
    netConfig_.gatewayIP[1] = netConfig_.currentIP[1];
    netConfig_.gatewayIP[2] = netConfig_.currentIP[2];
    netConfig_.gatewayIP[3] = 1;

    // Update broadcast IP (same subnet, .255)
    netConfig_.broadcastIP[0] = netConfig_.currentIP[0];
    netConfig_.broadcastIP[1] = netConfig_.currentIP[1];
    netConfig_.broadcastIP[2] = netConfig_.currentIP[2];
    netConfig_.broadcastIP[3] = 255;
}

void ConfigManager::setGPSConfig(const GPSConfigStruct &config)
{
    gpsConfig_ = config;
}

void ConfigManager::saveGPSConfig()
{
    EEPROM.put(GPS_CONFIG_ADDR, gpsConfig_);
}

void ConfigManager::loadGPSConfig()
{
    EEPROM.get(GPS_CONFIG_ADDR, gpsConfig_);
}

void ConfigManager::resetGPSConfigToDefaults()
{
    gpsConfig_ = defaultGPSConfig_;
}

void ConfigManager::setSteerConfig(const SteerConfigStruct &config)
{
    steerConfig_ = config;
}

void ConfigManager::saveSteerConfig()
{
    EEPROM.put(STEER_CONFIG_ADDR, steerConfig_);
}

void ConfigManager::loadSteerConfig()
{
    EEPROM.get(STEER_CONFIG_ADDR, steerConfig_);
}

void ConfigManager::resetSteerConfigToDefaults()
{
    steerConfig_ = defaultSteerConfig_;
}

void ConfigManager::setSteerSettings(const SteerSettingsStruct &settings)
{
    steerSettings_ = settings;
}

void ConfigManager::saveSteerSettings()
{
    EEPROM.put(STEER_SETTINGS_ADDR, steerSettings_);
}

void ConfigManager::loadSteerSettings()
{
    EEPROM.get(STEER_SETTINGS_ADDR, steerSettings_);
}

void ConfigManager::resetSteerSettingsToDefaults()
{
    steerSettings_ = defaultSteerSettings_;
}

uint32_t ConfigManager::ipv4ToUint32(const uint8_t ip[4]) const
{
    char buf[16];
    mg_snprintf(buf, sizeof(buf), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    struct mg_addr a = {};
    mg_aton(mg_str(buf), &a);
    return *(uint32_t *)&a.ip;
}

void ConfigManager::uint32ToIpv4(uint32_t ip, uint8_t result[4]) const
{
    result[0] = (ip >> 0) & 0xFF;
    result[1] = (ip >> 8) & 0xFF;
    result[2] = (ip >> 16) & 0xFF;
    result[3] = (ip >> 24) & 0xFF;
}

bool ConfigManager::isConfigValid() const
{
    // Basic validation checks
    if (netConfig_.currentIP[0] == 0 || netConfig_.currentIP[0] > 223)
        return false;
    if (steerSettings_.Kp == 0 || steerSettings_.Kp > 200)
        return false;
    if (steerSettings_.steerSensorCounts <= 0)
        return false;
    return true;
}

void ConfigManager::resetAllToDefaults()
{
    resetNetConfigToDefaults();
    resetGPSConfigToDefaults();
    resetSteerConfigToDefaults();
    resetSteerSettingsToDefaults();

    saveNetConfig();
    saveGPSConfig();
    saveSteerConfig();
    saveSteerSettings();
}