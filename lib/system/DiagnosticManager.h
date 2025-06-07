#ifndef DIAGNOSTIC_MANAGER_H
#define DIAGNOSTIC_MANAGER_H

#include "Arduino.h"
#include "elapsedMillis.h"

// Forward declarations
class ProcessorUsage;
class HighLowHzStats;

class DiagnosticManager
{
public:
    DiagnosticManager();
    ~DiagnosticManager();

    // Initialization
    void begin();

    // CAN Manager reference (using void pointer to avoid circular dependency)
    void setCANManager(void *canManager);

    // CPU usage tracking
    void registerCpuUsage(const char *name, ProcessorUsage *usage);
    void printCpuUsage();
    void resetCpuUsage();

    // Statistics tracking
    void registerStats(const char *name, HighLowHzStats *stats);
    void printStats();
    void resetStats();

    // Debug command processing
    void processDebugCommands();
    void setDebugFlag(const char *flag, bool value);
    bool getDebugFlag(const char *flag) const;

    // System monitoring
    void updateSystemStats();
    uint32_t getLoopFrequency() const { return loopCounter_ / (updateTimer_ / 1000); }
    float getCpuTemperature() const;
    uint32_t getFreeRAM() const;

    // Debug flags
    struct DebugFlags
    {
        bool printCpuUsages = false;
        bool printStats = false;
        bool adcDebug = false;
        bool nmeaDebug = false;
        bool nmeaDebug2 = false;
        bool pwmDebug = false;
    } debugFlags;

private:
    // CPU usage tracking
    static constexpr uint8_t MAX_CPU_USAGE_TASKS = 20;
    ProcessorUsage *cpuUsageTasks_[MAX_CPU_USAGE_TASKS];
    const char *cpuUsageNames_[MAX_CPU_USAGE_TASKS];
    uint8_t cpuUsageCount_;

    // Statistics tracking
    static constexpr uint8_t MAX_STATS_TASKS = 10;
    HighLowHzStats *statsTasks_[MAX_STATS_TASKS];
    const char *statsNames_[MAX_STATS_TASKS];
    uint8_t statsCount_;

    // System monitoring
    elapsedMillis updateTimer_;
    uint32_t loopCounter_;
    uint32_t lastPrintTime_;
    static constexpr uint32_t PRINT_INTERVAL = 5000; // 5 seconds

    // USB serial management
    bool usbSerialActive_;
    void handleUSBSerial();
    void processSerialCommand(char command);
    void printSystemInfo();
    void printHelp();

    // CAN Manager reference (void pointer to avoid circular dependency)
    void *canManager_;

    // Helper functions
    void printCpuPercent(uint32_t time);
    bool isValidDebugCommand(char command) const;
};

// Simple CPU usage tracking class
class ProcessorUsage
{
public:
    ProcessorUsage(const char *name);
    void timeIn();
    void timeOut();
    uint32_t reportAve(uint32_t offset = 0);
    void reset();

private:
    const char *name_;
    uint32_t startTime_;
    uint32_t totalTime_;
    uint32_t sampleCount_;
    bool timing_;
};

// Statistics tracking class for Hz/frequency monitoring
class HighLowHzStats
{
public:
    HighLowHzStats();
    void incHzCount();
    void update(uint16_t value);
    void printStatsReport(const char *name);
    void resetAll();

    bool startupReset = true;

private:
    uint32_t hzCount_;
    uint16_t highValue_;
    uint16_t lowValue_;
    uint32_t totalValue_;
    uint32_t sampleCount_;
    elapsedMillis hzTimer_;
};

#endif // DIAGNOSTIC_MANAGER_H