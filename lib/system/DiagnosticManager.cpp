#include "DiagnosticManager.h"

// DiagnosticManager implementation
DiagnosticManager::DiagnosticManager()
    : cpuUsageCount_(0), statsCount_(0), loopCounter_(0), lastPrintTime_(0), usbSerialActive_(false)
{
    // Initialize arrays
    for (uint8_t i = 0; i < MAX_CPU_USAGE_TASKS; i++)
    {
        cpuUsageTasks_[i] = nullptr;
        cpuUsageNames_[i] = nullptr;
    }
    for (uint8_t i = 0; i < MAX_STATS_TASKS; i++)
    {
        statsTasks_[i] = nullptr;
        statsNames_[i] = nullptr;
    }
}

DiagnosticManager::~DiagnosticManager()
{
}

void DiagnosticManager::begin()
{
    Serial.print("\r\n- Diagnostic Manager initialization");
    usbSerialActive_ = true;
}

void DiagnosticManager::registerCpuUsage(const char *name, ProcessorUsage *usage)
{
    if (cpuUsageCount_ < MAX_CPU_USAGE_TASKS && usage != nullptr)
    {
        cpuUsageTasks_[cpuUsageCount_] = usage;
        cpuUsageNames_[cpuUsageCount_] = name;
        cpuUsageCount_++;
    }
}

void DiagnosticManager::registerStats(const char *name, HighLowHzStats *stats)
{
    if (statsCount_ < MAX_STATS_TASKS && stats != nullptr)
    {
        statsTasks_[statsCount_] = stats;
        statsNames_[statsCount_] = name;
        statsCount_++;
    }
}

void DiagnosticManager::updateSystemStats()
{
    loopCounter_++;

    if (updateTimer_ > PRINT_INTERVAL)
    {
        if (debugFlags.printCpuUsages)
        {
            Serial.print("\r\n[AUTO] CPU Usage Report:");
            printCpuUsage();
        }
        if (debugFlags.printStats)
        {
            Serial.print("\r\n[AUTO] Statistics Report:");
            printStats();
            Serial.print("\r\nLoop frequency: ");
            Serial.print(getLoopFrequency());
            Serial.print("kHz, CPU temp: ");
            Serial.print(getCpuTemperature());
            Serial.print("°C");
        }

        updateTimer_ = 0;
        loopCounter_ = 0;
    }
}

void DiagnosticManager::processDebugCommands()
{
    if (!usbSerialActive_)
        return;
    handleUSBSerial();
}

void DiagnosticManager::handleUSBSerial()
{
    if (Serial.available())
    {
        char command = Serial.read();
        if (isValidDebugCommand(command))
        {
            processSerialCommand(command);
        }
    }
}

void DiagnosticManager::processSerialCommand(char command)
{
    // Debug: show what command was received
    Serial.print("\r\nDiagnosticManager received: '");
    Serial.print(command);
    Serial.print("' (ASCII: ");
    Serial.print((int)command);
    Serial.print(")");

    switch (command)
    {
    case 'c':
        debugFlags.printCpuUsages = !debugFlags.printCpuUsages;
        Serial.print("\r\nCPU usage debug: ");
        Serial.print(debugFlags.printCpuUsages ? "ON" : "OFF");
        break;

    case 'a':
        debugFlags.adcDebug = !debugFlags.adcDebug;
        Serial.print("\r\nADC debug: ");
        Serial.print(debugFlags.adcDebug ? "ON" : "OFF");
        break;

    case 'n':
        debugFlags.nmeaDebug = !debugFlags.nmeaDebug;
        Serial.print("\r\nNMEA debug: ");
        Serial.print(debugFlags.nmeaDebug ? "ON" : "OFF");
        break;

    case 'p':
        debugFlags.pwmDebug = !debugFlags.pwmDebug;
        Serial.print("\r\nPWM debug: ");
        Serial.print(debugFlags.pwmDebug ? "ON" : "OFF");
        break;

    case 's':
        Serial.print("\r\n=== Current Statistics ===");

        // Show DiagnosticManager stats if any are registered
        if (statsCount_ > 0)
        {
            printStats();
        }
        else
        {
            Serial.print("\r\nNo DiagnosticManager stats registered");
        }

        // Show CPU usage stats if any are registered
        if (cpuUsageCount_ > 0)
        {
            Serial.print("\r\n--- CPU Usage ---");
            printCpuUsage();
        }
        else
        {
            Serial.print("\r\nNo CPU usage stats registered");
        }

        Serial.print("\r\n--- System Info ---");
        Serial.print("\r\nLoop frequency: ");
        Serial.print(getLoopFrequency());
        Serial.print("kHz");
        Serial.print("\r\nCPU temperature: ");
        Serial.print(getCpuTemperature());
        Serial.print("°C");
        Serial.print("\r\nFree RAM: ");
        Serial.print(getFreeRAM());
        Serial.print(" bytes");

        debugFlags.printStats = !debugFlags.printStats;
        Serial.print("\r\nAuto stats display: ");
        Serial.print(debugFlags.printStats ? "ON" : "OFF");
        break;

    case 'r':
        resetStats();
        Serial.print("\r\nStats reset");
        break;

    case 'R':
        Serial.print("\r\nRebooting...");
        delay(100);
        SCB_AIRCR = 0x05FA0004;
        break;

    case 'b':
        Serial.print("\r\n*** BUFFER COMMAND EXECUTED! ***");
        Serial.print("\r\nBuffer overflow toggle recognized!");
        Serial.print("\r\nThis would toggle buffer overflow checking");
        break;

    case 'h':
    case '?':
        printHelp();
        break;

    case 'i':
        printSystemInfo();
        break;

    case 13:
    case 10:
        // Ignore CR/LF
        break;

    default:
        Serial.print("\r\nUnknown command in switch: '");
        Serial.print(command);
        Serial.print("' (ASCII: ");
        Serial.print((int)command);
        Serial.print(") - Press 'h' for help");
        break;
    }

    Serial.print("\r\nCommand processing complete.");
}

void DiagnosticManager::printCpuUsage()
{
    if (cpuUsageCount_ == 0)
        return;

    Serial.print("\r\n\nCPU Usage Report:");
    Serial.print("\r\nLoop freq: ");
    Serial.print(getLoopFrequency());
    Serial.print("kHz");

    for (uint8_t i = 0; i < cpuUsageCount_; i++)
    {
        if (cpuUsageTasks_[i] != nullptr)
        {
            Serial.print("\r\n");
            Serial.print(cpuUsageNames_[i]);
            Serial.print(": ");
            printCpuPercent(cpuUsageTasks_[i]->reportAve());
        }
    }
    Serial.println();
}

void DiagnosticManager::printStats()
{
    if (statsCount_ == 0)
        return;

    Serial.print("\r\n\nStatistics Report:");
    for (uint8_t i = 0; i < statsCount_; i++)
    {
        if (statsTasks_[i] != nullptr)
        {
            statsTasks_[i]->printStatsReport(statsNames_[i]);
        }
    }
    Serial.println();
}

void DiagnosticManager::resetCpuUsage()
{
    for (uint8_t i = 0; i < cpuUsageCount_; i++)
    {
        if (cpuUsageTasks_[i] != nullptr)
        {
            cpuUsageTasks_[i]->reset();
        }
    }
}

void DiagnosticManager::resetStats()
{
    for (uint8_t i = 0; i < statsCount_; i++)
    {
        if (statsTasks_[i] != nullptr)
        {
            statsTasks_[i]->resetAll();
        }
    }
}

void DiagnosticManager::setDebugFlag(const char *flag, bool value)
{
    if (strcmp(flag, "cpu") == 0)
        debugFlags.printCpuUsages = value;
    else if (strcmp(flag, "adc") == 0)
        debugFlags.adcDebug = value;
    else if (strcmp(flag, "nmea") == 0)
        debugFlags.nmeaDebug = value;
    else if (strcmp(flag, "nmea2") == 0)
        debugFlags.nmeaDebug2 = value;
    else if (strcmp(flag, "pwm") == 0)
        debugFlags.pwmDebug = value;
    else if (strcmp(flag, "stats") == 0)
        debugFlags.printStats = value;
}

bool DiagnosticManager::getDebugFlag(const char *flag) const
{
    if (strcmp(flag, "cpu") == 0)
        return debugFlags.printCpuUsages;
    else if (strcmp(flag, "adc") == 0)
        return debugFlags.adcDebug;
    else if (strcmp(flag, "nmea") == 0)
        return debugFlags.nmeaDebug;
    else if (strcmp(flag, "nmea2") == 0)
        return debugFlags.nmeaDebug2;
    else if (strcmp(flag, "pwm") == 0)
        return debugFlags.pwmDebug;
    else if (strcmp(flag, "stats") == 0)
        return debugFlags.printStats;
    return false;
}

void DiagnosticManager::printCpuPercent(uint32_t time)
{
    Serial.printf("%4.1f", (float)time / 10000.0);
    Serial.print("%");
}

float DiagnosticManager::getCpuTemperature() const
{
    return tempmonGetTemp();
}

uint32_t DiagnosticManager::getFreeRAM() const
{
    // Simple estimation - not completely accurate on Teensy
    extern unsigned long _heap_start;
    extern unsigned long _heap_end;
    extern char *__brkval;

    if (__brkval == 0)
    {
        return ((char *)&_heap_end) - ((char *)&_heap_start);
    }
    else
    {
        return ((char *)&_heap_end) - __brkval;
    }
}

void DiagnosticManager::printSystemInfo()
{
    Serial.print("\r\n\nSystem Information:");
    Serial.print("\r\nCPU Frequency: ");
    Serial.print(F_CPU_ACTUAL / 1000000);
    Serial.print("MHz");
    Serial.print("\r\nCPU Temperature: ");
    Serial.print(getCpuTemperature());
    Serial.print("°C");
    Serial.print("\r\nFree RAM: ");
    Serial.print(getFreeRAM());
    Serial.print(" bytes");
    Serial.print("\r\nLoop Frequency: ");
    Serial.print(getLoopFrequency());
    Serial.print("kHz");
}

void DiagnosticManager::printHelp()
{
    Serial.print("\r\n\nDebug Commands:");
    Serial.print("\r\nc - Toggle CPU usage display");
    Serial.print("\r\na - Toggle ADC debug");
    Serial.print("\r\nn - Toggle NMEA debug");
    Serial.print("\r\np - Toggle PWM debug");
    Serial.print("\r\ns - Toggle statistics display");
    Serial.print("\r\nr - Reset statistics");
    Serial.print("\r\nR - Reboot system");
    Serial.print("\r\ni - Show system information");
    Serial.print("\r\nb - Buffer overflow toggle (Phase 2 test)");
    Serial.print("\r\nh/? - Show this help");
}

bool DiagnosticManager::isValidDebugCommand(char command) const
{
    const char validCommands[] = "canpsrRhi?b"; // Added 'b' to valid commands
    for (uint8_t i = 0; i < strlen(validCommands); i++)
    {
        if (command == validCommands[i])
            return true;
    }
    return (command == 13 || command == 10); // CR/LF
}

// ProcessorUsage implementation
ProcessorUsage::ProcessorUsage(const char *name)
    : name_(name), startTime_(0), totalTime_(0), sampleCount_(0), timing_(false)
{
}

void ProcessorUsage::timeIn()
{
    if (!timing_)
    {
        startTime_ = micros();
        timing_ = true;
    }
}

void ProcessorUsage::timeOut()
{
    if (timing_)
    {
        totalTime_ += (micros() - startTime_);
        sampleCount_++;
        timing_ = false;
    }
}

uint32_t ProcessorUsage::reportAve(uint32_t offset)
{
    if (sampleCount_ == 0)
        return 0;
    uint32_t average = totalTime_ / sampleCount_;
    return (average > offset) ? (average - offset) : 0;
}

void ProcessorUsage::reset()
{
    totalTime_ = 0;
    sampleCount_ = 0;
    timing_ = false;
}

// HighLowHzStats implementation
HighLowHzStats::HighLowHzStats()
    : hzCount_(0), highValue_(0), lowValue_(65535), totalValue_(0), sampleCount_(0)
{
}

void HighLowHzStats::incHzCount()
{
    hzCount_++;
}

void HighLowHzStats::update(uint16_t value)
{
    if (value > highValue_)
        highValue_ = value;
    if (value < lowValue_)
        lowValue_ = value;
    totalValue_ += value;
    sampleCount_++;
}

void HighLowHzStats::printStatsReport(const char *name)
{
    uint32_t hz = (hzTimer_ > 0) ? (hzCount_ * 1000 / hzTimer_) : 0;
    uint16_t average = (sampleCount_ > 0) ? (totalValue_ / sampleCount_) : 0;

    Serial.print("\r\n");
    Serial.print(name);
    Serial.print(": ");
    Serial.print(hz);
    Serial.print("Hz, Avg:");
    Serial.print(average);
    Serial.print(", H:");
    Serial.print(highValue_);
    Serial.print(", L:");
    Serial.print(lowValue_);
}

void HighLowHzStats::resetAll()
{
    hzCount_ = 0;
    highValue_ = 0;
    lowValue_ = 65535;
    totalValue_ = 0;
    sampleCount_ = 0;
    hzTimer_ = 0;
    startupReset = false;
}