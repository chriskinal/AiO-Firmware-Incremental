#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H

#include "Arduino.h"
#include "HardwareSerial.h"

class HardwareManager
{
public:
    HardwareManager();
    ~HardwareManager();

    // Initialization
    void begin();
    void setupPins();
    void setupSerial();
    void setupPWM();
    void setupADC();

    // Pin definitions
    static constexpr uint8_t WAS_SENSOR_PIN = A15;
    static constexpr uint8_t SPEEDPULSE_PIN = 33;
    static constexpr uint8_t SPEEDPULSE10_PIN = 37;
    static constexpr uint8_t BUZZER = 36;
    static constexpr uint8_t SLEEP_PIN = 4;
    static constexpr uint8_t PWM1_PIN = 5;
    static constexpr uint8_t PWM2_PIN = 6;
    static constexpr uint8_t STEER_PIN = 2;
    static constexpr uint8_t WORK_PIN = A17;
    static constexpr uint8_t KICKOUT_D_PIN = 3;
    static constexpr uint8_t CURRENT_PIN = A13;
    static constexpr uint8_t KICKOUT_A_PIN = A12;

    // Serial port references
    HardwareSerial *getSerialIMU() { return &Serial4; }
    HardwareSerial *getSerialRTK() { return &Serial3; }
    HardwareSerial *getSerialGPS1() { return &Serial5; }
    HardwareSerial *getSerialGPS2() { return &Serial8; }
    HardwareSerial *getSerialRS232() { return &Serial7; }
    HardwareSerial *getSerialESP32() { return &Serial2; }

    // Baud rates
    static constexpr int32_t BAUD_GPS = 460800;
    static constexpr int32_t BAUD_RTK = 115200;
    static constexpr int32_t BAUD_RS232 = 38400;
    static constexpr int32_t BAUD_ESP32 = 460800;

    // PWM configuration
    static constexpr uint8_t PWM_FREQUENCY = 4; // 0=490Hz, 1=122Hz, 2=3921Hz, 3=9155Hz, 4=18310Hz
    void setPWMFrequency(uint8_t pin1, uint8_t pin2, uint8_t frequency);

    // CPU frequency management
    void setCpuFrequency(uint32_t frequency);
    uint32_t getCpuFrequency() const { return F_CPU_ACTUAL; }

    // Hardware status
    bool isSerialPortReady(HardwareSerial *port) const;
    bool isI2CDevicePresent(uint8_t address) const;

    // ADC Reading functions
    uint16_t readWAS() const;
    uint16_t readCurrent() const;
    uint16_t readWork() const;
    uint16_t readKickoutAnalog() const;

    // ADC voltage conversion (12-bit ADC, 3.3V reference)
    float readWASVoltage() const;
    float readCurrentVoltage() const;
    float readWorkVoltage() const;
    float readKickoutVoltage() const;

    // Debug output
    void printADCDebug() const;

private:
    // Buffer sizes
    static constexpr size_t GPS1_RX_BUFFER_SIZE = 128;
    static constexpr size_t GPS1_TX_BUFFER_SIZE = 256;
    static constexpr size_t GPS2_RX_BUFFER_SIZE = 128;
    static constexpr size_t GPS2_TX_BUFFER_SIZE = 256;
    static constexpr size_t RTK_RX_BUFFER_SIZE = 64;
    static constexpr size_t RS232_TX_BUFFER_SIZE = 256;
    static constexpr size_t ESP32_RX_BUFFER_SIZE = 256;
    static constexpr size_t ESP32_TX_BUFFER_SIZE = 256;

    // Serial buffers
    uint8_t gps1RxBuffer_[GPS1_RX_BUFFER_SIZE];
    uint8_t gps1TxBuffer_[GPS1_TX_BUFFER_SIZE];
    uint8_t gps2RxBuffer_[GPS2_RX_BUFFER_SIZE];
    uint8_t gps2TxBuffer_[GPS2_TX_BUFFER_SIZE];
    uint8_t rtkRxBuffer_[RTK_RX_BUFFER_SIZE];
    uint8_t rs232TxBuffer_[RS232_TX_BUFFER_SIZE];
    uint8_t esp32RxBuffer_[ESP32_RX_BUFFER_SIZE];
    uint8_t esp32TxBuffer_[ESP32_TX_BUFFER_SIZE];

    // Initialization flags
    bool serialInitialized_;
    bool pinsInitialized_;
    bool pwmInitialized_;
    bool adcInitialized_;

    // Helper functions
    void setupPinModes();
    void setupSerialBuffers();
    uint16_t getPWMFrequencyValue(uint8_t frequency) const;
};

extern "C" uint32_t set_arm_clock(uint32_t frequency);

#endif // HARDWARE_MANAGER_H