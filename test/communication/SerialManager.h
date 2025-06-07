#ifndef SERIAL_MANAGER_H
#define SERIAL_MANAGER_H

#include "Arduino.h"
#include "HardwareSerial.h"
#include "HardwareManager.h"
#include "DiagnosticManager.h"

class SerialManager
{
public:
    SerialManager(HardwareManager &hardwareManager, DiagnosticManager &diagnosticManager);
    ~SerialManager();

    // Initialization
    void begin();
    void update();

    // GPS serial communication
    bool isGPS1Available() const;
    bool isGPS2Available() const;
    uint8_t readGPS1();
    uint8_t readGPS2();
    size_t writeGPS1(const uint8_t *data, size_t length);
    size_t writeGPS2(const uint8_t *data, size_t length);
    size_t writeGPS1(uint8_t data);
    size_t writeGPS2(uint8_t data);
    void clearGPS1();
    void clearGPS2();

    // RTK radio communication
    bool isRTKAvailable() const;
    uint8_t readRTK();
    size_t writeRTK(const uint8_t *data, size_t length);
    size_t writeRTK(uint8_t data);

    // ESP32 communication
    bool isESP32Available() const;
    uint8_t readESP32();
    size_t writeESP32(const uint8_t *data, size_t length);
    size_t writeESP32(uint8_t data);
    void writeESP32Line(); // Write CR/LF

    // RS232 communication
    bool isRS232Available() const;
    uint8_t readRS232();
    size_t writeRS232(const uint8_t *data, size_t length);
    size_t writeRS232(uint8_t data);

    // IMU communication
    bool isIMUAvailable() const;
    uint8_t readIMU();
    size_t writeIMU(const uint8_t *data, size_t length);
    size_t writeIMU(uint8_t data);

    // Bridge mode management
    bool isUSB1DTR() const { return usb1DTR_; }
    bool isUSB2DTR() const { return usb2DTR_; }
    void updateBridgeMode();
    void handleBridgeGPS1();
    void handleBridgeGPS2();

    // Baud rate management
    uint32_t getGPS1Baud() const { return gps1Baud_; }
    uint32_t getGPS2Baud() const { return gps2Baud_; }
    void setGPS1Baud(uint32_t baud);
    void setGPS2Baud(uint32_t baud);

    // Statistics and monitoring
    struct SerialStats
    {
        uint32_t gps1BytesReceived = 0;
        uint32_t gps2BytesReceived = 0;
        uint32_t rtkBytesReceived = 0;
        uint32_t esp32BytesReceived = 0;
        uint32_t rs232BytesReceived = 0;
        uint32_t imuBytesReceived = 0;

        uint32_t gps1BytesSent = 0;
        uint32_t gps2BytesSent = 0;
        uint32_t rtkBytesSent = 0;
        uint32_t esp32BytesSent = 0;
        uint32_t rs232BytesSent = 0;
        uint32_t imuBytesSent = 0;
    };

    const SerialStats &getStats() const { return stats_; }
    void resetStats();

    // Buffer monitoring
    bool checkBufferOverflow();
    void clearAllBuffers();

private:
    HardwareManager &hardwareManager_;
    DiagnosticManager &diagnosticManager_;

    // Serial port references
    HardwareSerial *gps1_;
    HardwareSerial *gps2_;
    HardwareSerial *rtk_;
    HardwareSerial *esp32_;
    HardwareSerial *rs232_;
    HardwareSerial *imu_;

    // Bridge mode state
    bool usb1DTR_;
    bool usb2DTR_;
    bool prevUSB1DTR_;
    bool prevUSB2DTR_;

    // Baud rate tracking
    uint32_t gps1Baud_;
    uint32_t gps2Baud_;

    // Statistics
    SerialStats stats_;

    // CPU usage tracking
    ProcessorUsage gps1Usage_;
    ProcessorUsage gps2Usage_;
    ProcessorUsage rtkUsage_;
    ProcessorUsage esp32Usage_;
    ProcessorUsage rs232Usage_;

    // Buffer overflow detection
    static constexpr size_t BUFFER_WARNING_THRESHOLD = 90; // 90% full
    bool bufferOverflowDetected_;

    // Helper functions
    void initializeSerialPorts();
    void updateBaudRates();
    void handleBridgeMode();
    void updateStatistics(HardwareSerial *port, size_t bytes, bool transmitted);
    bool isBufferNearFull(HardwareSerial *port) const;
};

#endif // SERIAL_MANAGER_H