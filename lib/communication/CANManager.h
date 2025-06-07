#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include "Arduino.h"
#include "FlexCAN_T4.h"
#include "DiagnosticManager.h"
#include "elapsedMillis.h"

class CANManager
{
public:
    CANManager(DiagnosticManager &diagnosticManager);
    ~CANManager();

    // Initialization
    bool begin();
    void update();

    // Keya motor control
    void steerKeya(int16_t steerSpeed);
    bool isKeyaDetected() const { return keyaDetected_; }
    int8_t getKeyaCurrent() const { return keyaCurrentReading_; }
    bool getKeyaMotorStatus() const { return keyaMotorStatus_; }

    // CAN message handling
    void sendKeyaCommand(const uint8_t command[]);
    void queryKeyaCurrent();
    void queryKeyaVersion();
    void queryKeyaFault();
    void queryKeyaVoltage();
    void queryKeyaTemperature();

    // Statistics
    uint32_t getMessagesReceived() const { return messagesReceived_; }
    uint32_t getMessagesSent() const { return messagesSent_; }
    void resetStatistics();

    // Debug control
    void setDebugMode(bool enabled) { debugEnabled_ = enabled; }
    bool isDebugMode() const { return debugEnabled_; }

private:
    DiagnosticManager &diagnosticManager_;
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> keyaBus_;

    // Keya motor state
    bool keyaDetected_;
    bool keyaMotorStatus_;
    int8_t keyaCurrentReading_;
    elapsedMillis keyaCurrentUpdateTimer_;

    // Keya communication
    static constexpr uint64_t KEYA_ID = 0x06000001;
    static constexpr uint32_t KEYA_BAUD = 250000;
    static constexpr uint32_t KEYA_CURRENT_UPDATE_INTERVAL = 100; // ms

    // Statistics
    uint32_t messagesReceived_;
    uint32_t messagesSent_;

    // CPU usage tracking
    ProcessorUsage canUsage_;

    // Debug control
    bool debugEnabled_;

    // Keya command templates
    static const uint8_t keyaDisableCommand_[4];
    static const uint8_t keyaDisableResponse_[4];
    static const uint8_t keyaEnableCommand_[4];
    static const uint8_t keyaEnableResponse_[4];
    static const uint8_t keyaSpeedCommand_[4];
    static const uint8_t keyaSpeedResponse_[4];
    static const uint8_t keyaCurrentQuery_[4];
    static const uint8_t keyaCurrentResponse_[4];
    static const uint8_t keyaFaultQuery_[4];
    static const uint8_t keyaFaultResponse_[4];
    static const uint8_t keyaVoltageQuery_[4];
    static const uint8_t keyaVoltageResponse_[4];
    static const uint8_t keyaTemperatureQuery_[4];
    static const uint8_t keyaTemperatureResponse_[4];
    static const uint8_t keyaVersionQuery_[4];
    static const uint8_t keyaVersionResponse_[4];

    // Helper functions
    void processKeyaMessages();
    void handleHeartbeat(const CAN_message_t &message);
    void handleQueryResponse(const CAN_message_t &message);
    void handleMotorDiagnostics(uint8_t errorByte);
    void handleMotorStatus(uint8_t statusByte);
    bool isPatternMatch(const CAN_message_t &message, const uint8_t *pattern, size_t patternSize);
    void printCANMessage(uint32_t id, const uint8_t data[8], const char *description = nullptr);
    void mapSpeedToPWM(int16_t steerSpeed, CAN_message_t &message);

    // Note: Using Arduino's built-in lowByte() and highByte() macros instead of custom functions
};

#endif // CAN_MANAGER_H