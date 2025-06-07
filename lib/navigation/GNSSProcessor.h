#ifndef GNSS_PROCESSOR_H
#define GNSS_PROCESSOR_H

#include "Arduino.h"
#include "SerialManager.h"
#include "DiagnosticManager.h"
#include "elapsedMillis.h"

class GNSSProcessor
{
public:
    GNSSProcessor(SerialManager &serialManager, DiagnosticManager &diagnosticManager);
    ~GNSSProcessor();

    // Initialization
    bool begin();
    void update();

    // Position data structure
    struct PositionData
    {
        double latitude = 0.0;
        double longitude = 0.0;
        float altitude = 0.0f;
        float speed = 0.0f;
        float heading = 0.0f;
        uint8_t fixQuality = 0;
        uint8_t satelliteCount = 0;
        float hdop = 99.9f;
        bool validFix = false;
        uint32_t lastUpdateTime = 0;
    };

    // NMEA sentence structure
    struct NMEASentence
    {
        char sentence[256];
        uint8_t length = 0;
        bool complete = false;
        char talkerID[3] = {0};
        char messageID[4] = {0};
    };

    // GPS data access
    const PositionData &getGPS1Data() const { return gps1Data_; }
    const PositionData &getGPS2Data() const { return gps2Data_; }
    bool isGPS1Valid() const { return gps1Data_.validFix && (millis() - gps1Data_.lastUpdateTime) < GPS_TIMEOUT; }
    bool isGPS2Valid() const { return gps2Data_.validFix && (millis() - gps2Data_.lastUpdateTime) < GPS_TIMEOUT; }

    // Best position selection
    const PositionData &getBestPosition() const;
    bool hasBestPosition() const;

    // NMEA parsing control
    void enableNMEAParsing(bool enable) { nmeaParsingEnabled_ = enable; }
    bool isNMEAParsingEnabled() const { return nmeaParsingEnabled_; }

    // UBX message handling
    void sendUBXMessage(const uint8_t *message, size_t length, bool toGPS1 = true, bool toGPS2 = false);
    bool parseUBXMessage(const uint8_t *data, size_t length);

    // GPS configuration
    void configureGPSOutput(bool gps1 = true, bool gps2 = false);
    void setGPSBaudRate(uint32_t baud, bool gps1 = true, bool gps2 = false);

    // Statistics
    uint32_t getGPS1SentencesProcessed() const { return gps1SentencesProcessed_; }
    uint32_t getGPS2SentencesProcessed() const { return gps2SentencesProcessed_; }
    uint32_t getGPS1ParseErrors() const { return gps1ParseErrors_; }
    uint32_t getGPS2ParseErrors() const { return gps2ParseErrors_; }
    void resetStatistics();

    // Debug control
    void setDebugMode(bool enabled) { debugEnabled_ = enabled; }
    bool isDebugMode() const { return debugEnabled_; }

private:
    SerialManager &serialManager_;
    DiagnosticManager &diagnosticManager_;

    // GPS position data
    PositionData gps1Data_;
    PositionData gps2Data_;
    mutable PositionData bestPosition_;

    // NMEA parsing state
    NMEASentence gps1Sentence_;
    NMEASentence gps2Sentence_;
    bool nmeaParsingEnabled_;

    // UBX parsing state
    uint8_t ubxBuffer_[256];
    uint8_t ubxIndex_;
    bool ubxParsingActive_;

    // GPS timeout and quality
    static constexpr uint32_t GPS_TIMEOUT = 5000;        // 5 seconds
    static constexpr uint32_t GPS_UPDATE_INTERVAL = 100; // 100ms

    // Statistics
    uint32_t gps1SentencesProcessed_;
    uint32_t gps2SentencesProcessed_;
    uint32_t gps1ParseErrors_;
    uint32_t gps2ParseErrors_;

    // CPU usage tracking
    ProcessorUsage gpsProcessorUsage_;

    // Debug control
    bool debugEnabled_;

    // Update timers
    elapsedMillis gpsUpdateTimer_;

    // NMEA parsing functions
    void processGPS1Data();
    void processGPS2Data();
    bool parseNMEAChar(char c, NMEASentence &sentence, PositionData &data, const char *source);
    void processCompleteSentence(const NMEASentence &sentence, PositionData &data, const char *source);

    // NMEA sentence parsers
    bool parseGGA(const char *sentence, PositionData &data);
    bool parseRMC(const char *sentence, PositionData &data);
    bool parseVTG(const char *sentence, PositionData &data);
    bool parseGSA(const char *sentence, PositionData &data);
    bool parseGSV(const char *sentence, PositionData &data);
    bool parseHPR(const char *sentence, PositionData &data);
    bool parseKSXT(const char *sentence, PositionData &data);

    // NMEA utility functions
    bool validateNMEAChecksum(const char *sentence) const;
    uint8_t calculateNMEAChecksum(const char *sentence) const;
    void extractTalkerAndMessage(const char *sentence, char *talkerID, char *messageID) const;
    bool parseCoordinate(const char *coord, const char *direction, double &result) const;
    bool parseTime(const char *timeStr, uint32_t &result) const;
    bool parseDate(const char *dateStr, uint32_t &result) const;

    // UBX utility functions
    bool validateUBXChecksum(const uint8_t *data, size_t length) const;
    void calculateUBXChecksum(const uint8_t *data, size_t length, uint8_t &ckA, uint8_t &ckB) const;

    // Position quality assessment
    void updatePositionQuality(PositionData &data);
    float calculateDistanceTo(const PositionData &from, const PositionData &to) const;
    bool isPositionReasonable(const PositionData &data) const;

    // Best position logic
    void updateBestPosition() const;
    uint8_t calculatePositionScore(const PositionData &data) const;

    // Debug output
    void printNMEADebug(const char *sentence, const char *source) const;
    void printPositionDebug(const PositionData &data, const char *source) const;
    void printUBXDebug(const uint8_t *data, size_t length, const char *direction) const;

    // Helper functions
    float parseFloat(const char *str) const;
    double parseDouble(const char *str) const;
    uint32_t parseUInt32(const char *str) const;
    const char *findNextField(const char *str) const;
    bool extractField(const char *start, char *buffer, size_t bufferSize) const;
    bool isFieldEmpty(const char *field) const;
};

#endif // GNSS_PROCESSOR_H