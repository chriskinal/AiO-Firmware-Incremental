#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include "Arduino.h"
#include "SerialManager.h"
#include "DiagnosticManager.h"
#include "elapsedMillis.h"

class IMUManager
{
public:
    IMUManager(SerialManager &serialManager, DiagnosticManager &diagnosticManager);
    ~IMUManager();

    // Initialization
    bool begin();
    void update();

    // IMU data structure
    struct IMUData
    {
        float roll = 0.0f;
        float pitch = 0.0f;
        float heading = 0.0f;
        float rollRate = 0.0f;
        float pitchRate = 0.0f;
        float yawRate = 0.0f;
        uint8_t calibrationStatus = 0;
        bool dataValid = false;
        uint32_t lastUpdateTime = 0;
    };

    // FUSE IMU calculation structure
    struct FUSEData
    {
        float fusedRoll = 0.0f;
        float fusedPitch = 0.0f;
        float fusedHeading = 0.0f;
        float correctedHeading = 0.0f;
        float rollOffset = 0.0f;
        float pitchOffset = 0.0f;
        bool fuseActive = false;
        uint32_t lastFuseTime = 0;
    };

    // Ring buffer for IMU data smoothing
    struct RingBuffer
    {
        static constexpr uint8_t BUFFER_SIZE = 16;
        float rollBuffer[BUFFER_SIZE];
        float pitchBuffer[BUFFER_SIZE];
        float headingBuffer[BUFFER_SIZE];
        uint8_t index = 0;
        uint8_t count = 0;
        bool bufferFull = false;
    };

    // Data access
    const IMUData &getIMUData() const { return imuData_; }
    const FUSEData &getFUSEData() const { return fuseData_; }
    bool isIMUValid() const { return imuData_.dataValid && (millis() - imuData_.lastUpdateTime) < IMU_TIMEOUT; }
    bool isFUSEActive() const { return fuseData_.fuseActive; }

    // IMU configuration
    void setBNOMode(uint8_t mode);
    void calibrateIMU();
    void resetIMU();
    void setIMUOffsets(float rollOffset, float pitchOffset);

    // FUSE calculations
    void enableFUSE(bool enable) { fuseEnabled_ = enable; }
    bool isFUSEEnabled() const { return fuseEnabled_; }
    void updateFUSECalculations();
    void setFUSEParameters(float alpha, float beta);

    // Ring buffer operations
    void addToRingBuffer(float roll, float pitch, float heading);
    float getSmoothedRoll() const;
    float getSmoothedPitch() const;
    float getSmoothedHeading() const;
    void clearRingBuffer();

    // Heading operations
    float getCompassHeading() const { return imuData_.heading; }
    float getCorrectedHeading() const { return fuseData_.correctedHeading; }
    void setHeadingOffset(float offset) { headingOffset_ = offset; }
    float getHeadingOffset() const { return headingOffset_; }

    // Statistics
    uint32_t getIMUMessagesProcessed() const { return imuMessagesProcessed_; }
    uint32_t getIMUParseErrors() const { return imuParseErrors_; }
    uint32_t getFUSECalculations() const { return fuseCalculations_; }
    void resetStatistics();

    // Debug control
    void setDebugMode(bool enabled) { debugEnabled_ = enabled; }
    bool isDebugMode() const { return debugEnabled_; }

private:
    SerialManager &serialManager_;
    DiagnosticManager &diagnosticManager_;

    // IMU data
    IMUData imuData_;
    FUSEData fuseData_;
    RingBuffer ringBuffer_;

    // IMU communication
    static constexpr uint32_t IMU_TIMEOUT = 2000;         // 2 seconds
    static constexpr uint32_t IMU_UPDATE_INTERVAL = 50;   // 50ms
    static constexpr uint32_t FUSE_UPDATE_INTERVAL = 100; // 100ms

    // BNO055 modes
    static constexpr uint8_t BNO_MODE_CONFIG = 0x00;
    static constexpr uint8_t BNO_MODE_NDOF = 0x0C;
    static constexpr uint8_t BNO_MODE_IMU = 0x08;
    static constexpr uint8_t BNO_MODE_COMPASS = 0x09;

    // FUSE parameters
    bool fuseEnabled_;
    float fuseAlpha_; // Complementary filter parameter
    float fuseBeta_;  // Heading correction factor
    float headingOffset_;

    // Statistics
    uint32_t imuMessagesProcessed_;
    uint32_t imuParseErrors_;
    uint32_t fuseCalculations_;

    // CPU usage tracking
    ProcessorUsage imuUsage_;

    // Debug control
    bool debugEnabled_;

    // Update timers
    elapsedMillis imuUpdateTimer_;
    elapsedMillis fuseUpdateTimer_;

    // BNO RVC parsing
    struct RVCMessage
    {
        uint8_t buffer[32];
        uint8_t index = 0;
        bool messageComplete = false;
    } rvcMessage_;

    // IMU processing functions
    void processIMUData();
    bool parseRVCMessage(const uint8_t *data, size_t length);
    void processRVCData(const uint8_t *data);
    bool validateRVCChecksum(const uint8_t *data, size_t length) const;

    // FUSE calculation functions
    void calculateComplementaryFilter();
    void calculateHeadingCorrection();
    void applyCalibrationOffsets();

    // Ring buffer helper functions
    void updateRingBuffer();
    float calculateRingBufferAverage(const float *buffer) const;
    float calculateRingBufferMedian(const float *buffer) const;

    // Angle utility functions
    float normalizeAngle(float angle) const;
    float angleDifference(float angle1, float angle2) const;
    float degreesToRadians(float degrees) const;
    float radiansToDegrees(float radians) const;

    // BNO055 communication
    void sendBNOCommand(uint8_t command);
    void setBNORegister(uint8_t reg, uint8_t value);
    uint8_t readBNORegister(uint8_t reg);

    // Calibration functions
    void startCalibration();
    void updateCalibrationStatus();
    bool isCalibrationComplete() const;

    // Debug output
    void printIMUDebug() const;
    void printFUSEDebug() const;
    void printRingBufferDebug() const;
    void printCalibrationDebug() const;

    // Data validation
    bool isIMUDataValid(const IMUData &data) const;
    bool isAngleReasonable(float angle) const;
    bool isRateReasonable(float rate) const;
};

#endif // IMU_MANAGER_H