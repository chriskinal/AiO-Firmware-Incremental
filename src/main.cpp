// Phase 3 Test - GPS and Navigation Classes
// This file tests compilation and basic functionality of Phase 3 classes

#include "Arduino.h"

// Phase 1 classes
#include "ConfigManager.h"
#include "HardwareManager.h"
#include "DiagnosticManager.h"

// Phase 2 classes
#include "NetworkManager.h"
#include "SerialManager.h"
#include "CANManager.h"

// Phase 3 classes
#include "GNSSProcessor.h"
#include "IMUManager.h"

// Function declarations - must be at top
void toggleKeyaDebug();
void printCANStatus();
void printCANStatusInSystemInfo();
void toggleGNSSDebug();
void toggleIMUDebug();
void printGPSStatus();
void printIMUStatus();
void toggleFUSE();
void toggleNMEAVerbose();
void toggleIMUVerbose();
void handlePhase3Command(char command);
void handleDiagnosticCommand(char command);

// Mongoose external declarations
extern "C"
{
  struct mg_mgr g_mgr;

  void ethernet_init(void)
  {
    Serial.print("\r\n  - Ethernet hardware initialized (stub)");
  }

  void mongoose_init(void)
  {
    memset(&g_mgr, 0, sizeof(g_mgr));
    mg_mgr_init(&g_mgr);
    Serial.print("\r\n  - Mongoose manager initialized");
  }

  void mongoose_poll(void)
  {
    static bool initialized = false;
    if (!initialized)
    {
      initialized = true;
      return;
    }
    mg_mgr_poll(&g_mgr, 1);
  }
}

// Global instances
ConfigManager *configManager = nullptr;
HardwareManager *hardwareManager = nullptr;
DiagnosticManager *diagnosticManager = nullptr;
NetworkManager *networkManager = nullptr;
SerialManager *serialManager = nullptr;
CANManager *canManager = nullptr;
GNSSProcessor *gnssProcessor = nullptr;
IMUManager *imuManager = nullptr;

// Debug timers
// Global flags to control our own periodic summaries independently of class debug modes
bool gnssPeriodicSummary = false;
bool imuPeriodicSummary = false;

// Debug timers
elapsedMillis adcDebugTimer;
elapsedMillis gpsDebugTimer;
elapsedMillis imuDebugTimer;
static constexpr uint32_t ADC_DEBUG_INTERVAL = 1000; // 1 second
static constexpr uint32_t GPS_DEBUG_INTERVAL = 5000; // 5 seconds - human readable
static constexpr uint32_t IMU_DEBUG_INTERVAL = 3000; // 3 seconds - human readable

// External functions for DiagnosticManager
void toggleKeyaDebug()
{
  if (canManager)
  {
    bool currentMode = canManager->isDebugMode();
    canManager->setDebugMode(!currentMode);
    Serial.print("\r\nKeya CAN debug: ");
    Serial.print(!currentMode ? "ON" : "OFF");
  }
  else
  {
    Serial.print("\r\nCAN Manager not available");
  }
}

void printCANStatus()
{
  if (canManager)
  {
    Serial.print("\r\n--- CAN Status ---");
    Serial.print("\r\nKeya detected: ");
    Serial.print(canManager->isKeyaDetected() ? "YES" : "NO");
    Serial.print("\r\nKeya debug mode: ");
    Serial.print(canManager->isDebugMode() ? "ON" : "OFF");
    Serial.print("\r\nCAN messages RX: ");
    Serial.print(canManager->getMessagesReceived());
    Serial.print(", TX: ");
    Serial.print(canManager->getMessagesSent());
  }
}

void printCANStatusInSystemInfo()
{
  if (canManager)
  {
    Serial.print("\r\nKeya Motor: ");
    Serial.print(canManager->isKeyaDetected() ? "Detected" : "Not detected");
    Serial.print("\r\nKeya Debug: ");
    Serial.print(canManager->isDebugMode() ? "Enabled" : "Disabled");
  }
}

// Phase 3 specific debug functions
void toggleGNSSDebug()
{
  Serial.print("toggleGNSSDebug() called");

  // Toggle our own periodic summary mode (independent of class debug)
  gnssPeriodicSummary = !gnssPeriodicSummary;

  Serial.print("\r\nGNSS periodic summaries: ");
  Serial.print(gnssPeriodicSummary ? "ON" : "OFF");
  if (gnssPeriodicSummary)
  {
    Serial.print("\r\n(Clean 5-second position summaries only)");
    Serial.print("\r\nNote: Use 'N' for verbose NMEA sentence logging");
  }
}

void toggleIMUDebug()
{
  Serial.print("toggleIMUDebug() called");

  // Toggle our own periodic summary mode (independent of class debug)
  imuPeriodicSummary = !imuPeriodicSummary;

  Serial.print("\r\nIMU periodic summaries: ");
  Serial.print(imuPeriodicSummary ? "ON" : "OFF");
  if (imuPeriodicSummary)
  {
    Serial.print("\r\n(Clean 3-second attitude summaries only)");
    Serial.print("\r\nNote: Use 'V' for verbose IMU data logging");
  }
}

void toggleNMEAVerbose()
{
  Serial.print("toggleNMEAVerbose() called");
  if (gnssProcessor)
  {
    bool currentMode = gnssProcessor->isDebugMode();
    gnssProcessor->setDebugMode(!currentMode);
    Serial.print("\r\nNMEA verbose logging: ");
    Serial.print(!currentMode ? "ON" : "OFF");
    if (!currentMode)
    {
      Serial.print("\r\n(WARNING: High-frequency NMEA sentence spam!)");
    }
  }
  else
  {
    Serial.print("\r\nERROR: GNSS Processor is NULL!");
  }
}

void toggleIMUVerbose()
{
  Serial.print("toggleIMUVerbose() called");
  if (imuManager)
  {
    bool currentMode = imuManager->isDebugMode();
    imuManager->setDebugMode(!currentMode);
    Serial.print("\r\nIMU verbose logging: ");
    Serial.print(!currentMode ? "ON" : "OFF");
    if (!currentMode)
    {
      Serial.print("\r\n(WARNING: High-frequency IMU data spam!)");
    }
  }
  else
  {
    Serial.print("\r\nERROR: IMU Manager is NULL!");
  }
}

void toggleFUSE()
{
  Serial.print("toggleFUSE() called");
  if (imuManager)
  {
    bool currentMode = imuManager->isFUSEEnabled();
    imuManager->enableFUSE(!currentMode);
    Serial.print("\r\nFUSE calculations: ");
    Serial.print(!currentMode ? "ENABLED" : "DISABLED");
    if (!currentMode)
    {
      Serial.print("\r\n(Complementary filter and data fusion active)");
    }
    else
    {
      Serial.print("\r\n(Using raw IMU data only)");
    }
  }
  else
  {
    Serial.print("\r\nERROR: IMU Manager is NULL!");
  }
}

void printGPSStatus()
{
  Serial.print("printGPSStatus() called");
  if (!gnssProcessor)
  {
    Serial.print("\r\nERROR: GNSS Processor is NULL!");
    return;
  }

  Serial.print("\r\n--- GPS Status ---");

  const auto &gps1Data = gnssProcessor->getGPS1Data();
  const auto &gps2Data = gnssProcessor->getGPS2Data();
  const auto &bestPos = gnssProcessor->getBestPosition();

  Serial.print("\r\nGPS1 - Valid: ");
  Serial.print(gnssProcessor->isGPS1Valid() ? "YES" : "NO");
  if (gnssProcessor->isGPS1Valid())
  {
    Serial.print(", Lat: ");
    Serial.print(gps1Data.latitude, 6);
    Serial.print(", Lon: ");
    Serial.print(gps1Data.longitude, 6);
    Serial.print(", Fix: ");
    Serial.print(gps1Data.fixQuality);
    Serial.print(", Sats: ");
    Serial.print(gps1Data.satelliteCount);
  }

  Serial.print("\r\nGPS2 - Valid: ");
  Serial.print(gnssProcessor->isGPS2Valid() ? "YES" : "NO");
  if (gnssProcessor->isGPS2Valid())
  {
    Serial.print(", Lat: ");
    Serial.print(gps2Data.latitude, 6);
    Serial.print(", Lon: ");
    Serial.print(gps2Data.longitude, 6);
    Serial.print(", Fix: ");
    Serial.print(gps2Data.fixQuality);
    Serial.print(", Sats: ");
    Serial.print(gps2Data.satelliteCount);
  }

  Serial.print("\r\nBest Position - Valid: ");
  Serial.print(gnssProcessor->hasBestPosition() ? "YES" : "NO");
  if (gnssProcessor->hasBestPosition())
  {
    Serial.print(", Lat: ");
    Serial.print(bestPos.latitude, 6);
    Serial.print(", Lon: ");
    Serial.print(bestPos.longitude, 6);
    Serial.print(", Speed: ");
    Serial.print(bestPos.speed, 1);
    Serial.print("km/h");
  }

  Serial.print("\r\nGPS1 sentences: ");
  Serial.print(gnssProcessor->getGPS1SentencesProcessed());
  Serial.print(", GPS2 sentences: ");
  Serial.print(gnssProcessor->getGPS2SentencesProcessed());
  Serial.print("\r\nGPS1 errors: ");
  Serial.print(gnssProcessor->getGPS1ParseErrors());
  Serial.print(", GPS2 errors: ");
  Serial.print(gnssProcessor->getGPS2ParseErrors());
}

void printIMUStatus()
{
  Serial.print("printIMUStatus() called");
  if (!imuManager)
  {
    Serial.print("\r\nERROR: IMU Manager is NULL!");
    return;
  }

  Serial.print("\r\n--- IMU Status ---");

  const auto &imuData = imuManager->getIMUData();
  const auto &fuseData = imuManager->getFUSEData();

  Serial.print("\r\nIMU Valid: ");
  Serial.print(imuManager->isIMUValid() ? "YES" : "NO");

  if (imuManager->isIMUValid())
  {
    Serial.print("\r\nRoll: ");
    Serial.print(imuData.roll, 1);
    Serial.print("°, Pitch: ");
    Serial.print(imuData.pitch, 1);
    Serial.print("°, Heading: ");
    Serial.print(imuData.heading, 1);
    Serial.print("°");
    Serial.print("\r\nCalibration: 0x");
    Serial.print(imuData.calibrationStatus, HEX);
  }

  Serial.print("\r\nFUSE Active: ");
  Serial.print(imuManager->isFUSEActive() ? "YES" : "NO");
  Serial.print(" (Enabled: ");
  Serial.print(imuManager->isFUSEEnabled() ? "YES" : "NO");
  Serial.print(")");

  if (imuManager->isFUSEActive())
  {
    Serial.print("\r\nFused Roll: ");
    Serial.print(fuseData.fusedRoll, 1);
    Serial.print("°, Fused Pitch: ");
    Serial.print(fuseData.fusedPitch, 1);
    Serial.print("°");
    Serial.print("\r\nCorrected Heading: ");
    Serial.print(fuseData.correctedHeading, 1);
    Serial.print("°");
  }

  Serial.print("\r\nSmoothed Roll: ");
  Serial.print(imuManager->getSmoothedRoll(), 1);
  Serial.print("°, Smoothed Pitch: ");
  Serial.print(imuManager->getSmoothedPitch(), 1);
  Serial.print("°");

  Serial.print("\r\nIMU messages: ");
  Serial.print(imuManager->getIMUMessagesProcessed());
  Serial.print(", errors: ");
  Serial.print(imuManager->getIMUParseErrors());
  Serial.print(", FUSE calcs: ");
  Serial.print(imuManager->getFUSECalculations());
}

// Test callback functions
void testPGNHandler(struct mg_connection *c, int ev, void *ev_data)
{
  Serial.print("\r\nPGN handler called");
}

void testRTCMHandler(struct mg_connection *c, int ev, void *ev_data)
{
  Serial.print("\r\nRTCM handler called");
  delay(10); // Prevent tight loop
}

void setup()
{
  delay(5000);
  Serial.begin(115200);

  while (!Serial && millis() < 10000)
  {
    delay(10);
  }

  Serial.print("\r\n\n*** Phase 3 Test - GPS and Navigation Classes ***");
  Serial.print("\r\nStarting initialization...");

  // Create Phase 1 objects
  configManager = new ConfigManager();
  hardwareManager = new HardwareManager();
  diagnosticManager = new DiagnosticManager();

  if (!configManager || !hardwareManager || !diagnosticManager)
  {
    Serial.print("\r\nFailed to allocate Phase 1 objects!");
    while (1)
      delay(1000);
  }

  // Initialize Phase 1
  Serial.print("\r\n\nInitializing Phase 1...");
  configManager->begin();
  hardwareManager->begin();
  diagnosticManager->begin();
  Serial.print("\r\n✓ Phase 1 complete");

  // Create Phase 2 objects
  Serial.print("\r\n\nCreating Phase 2 objects...");
  networkManager = new NetworkManager(*configManager, *diagnosticManager);
  serialManager = new SerialManager(*hardwareManager, *diagnosticManager);
  canManager = new CANManager(*diagnosticManager);

  if (!networkManager || !serialManager || !canManager)
  {
    Serial.print("\r\nFailed to allocate Phase 2 objects!");
    while (1)
      delay(1000);
  }

  // Initialize Phase 2
  Serial.print("\r\n\nInitializing Phase 2...");
  diagnosticManager->setCANManager(canManager);
  serialManager->begin();
  if (canManager->begin())
  {
    Serial.print("\r\n✓ CANManager initialized");
  }
  else
  {
    Serial.print("\r\n✗ CANManager failed");
  }

  networkManager->setPGNHandler(testPGNHandler);
  networkManager->setRTCMHandler(testRTCMHandler);
  Serial.print("\r\n✓ Phase 2 complete");

  // Create Phase 3 objects
  Serial.print("\r\n\nCreating Phase 3 objects...");
  gnssProcessor = new GNSSProcessor(*serialManager, *diagnosticManager);
  imuManager = new IMUManager(*serialManager, *diagnosticManager);

  if (!gnssProcessor || !imuManager)
  {
    Serial.print("\r\nFailed to allocate Phase 3 objects!");
    while (1)
      delay(1000);
  }

  // Initialize Phase 3
  Serial.print("\r\n\nInitializing Phase 3...");

  if (gnssProcessor->begin())
  {
    Serial.print("\r\n✓ GNSS Processor initialized");
    gnssProcessor->enableNMEAParsing(true);
    // IMPORTANT: Start with debug mode OFF to prevent spam
    gnssProcessor->setDebugMode(false);
    Serial.print("\r\n  - NMEA parsing enabled, debug mode OFF");
  }
  else
  {
    Serial.print("\r\n✗ GNSS Processor failed");
  }

  if (imuManager->begin())
  {
    Serial.print("\r\n✓ IMU Manager initialized");
    imuManager->enableFUSE(true);
    // IMPORTANT: Start with debug mode OFF to prevent spam
    imuManager->setDebugMode(false);
    Serial.print("\r\n  - FUSE calculations enabled, debug mode OFF");
  }
  else
  {
    Serial.print("\r\n✗ IMU Manager failed");
  }

  Serial.print("\r\n✓ Phase 3 complete");

  // Test initial readings
  Serial.print("\r\n\nTesting initial readings...");

  if (hardwareManager)
  {
    Serial.print("\r\nADC - WAS: ");
    Serial.print(hardwareManager->readWAS());
    Serial.print(" (");
    Serial.print(hardwareManager->readWASVoltage(), 3);
    Serial.print("V)");
  }

  if (serialManager)
  {
    Serial.print("\r\nSerial - GPS1 baud: ");
    Serial.print(serialManager->getGPS1Baud());
    Serial.print(", GPS2 baud: ");
    Serial.print(serialManager->getGPS2Baud());
  }

  if (canManager)
  {
    Serial.print("\r\nCAN - Keya detected: ");
    Serial.print(canManager->isKeyaDetected() ? "YES" : "NO");
  }

  Serial.print("\r\n\n*** Phase 3 Test Complete - All Navigation Features Ready! ***");
  Serial.print("\r\n\nPhase 3 Debug Commands:");
  Serial.print("\r\nh - Help");
  Serial.print("\r\na - Toggle ADC debug");
  Serial.print("\r\nk - Toggle Keya CAN debug");
  Serial.print("\r\nG - Toggle GNSS debug (periodic summaries)");
  Serial.print("\r\nI - Toggle IMU debug (periodic summaries)");
  Serial.print("\r\nF - Toggle FUSE calculations (capital F)");
  Serial.print("\r\nN - Toggle NMEA verbose logging (capital N)");
  Serial.print("\r\nV - Toggle IMU verbose logging (capital V)");
  Serial.print("\r\nP - Show GPS position status (capital P)");
  Serial.print("\r\nM - Show IMU status (capital M)");
  Serial.print("\r\ns - Show all statistics");
  Serial.print("\r\nr - Reset all statistics");
  Serial.print("\r\nR - Reboot system");
  Serial.print("\r\n\nNote: Use CAPITAL letters for Phase 3 commands to avoid conflicts");
  Serial.print("\r\nAll debug modes are OFF by default to prevent message spam");
}

void loop()
{
  // Update all managers
  if (serialManager)
  {
    serialManager->update();
  }

  if (canManager)
  {
    canManager->update();
  }

  if (gnssProcessor)
  {
    gnssProcessor->update();
  }

  if (imuManager)
  {
    imuManager->update();
  }

  if (diagnosticManager)
  {
    diagnosticManager->updateSystemStats();

    // Handle periodic debug output
    if (diagnosticManager->getDebugFlag("adc") && adcDebugTimer > ADC_DEBUG_INTERVAL)
    {
      if (hardwareManager)
      {
        hardwareManager->printADCDebug();
      }
      adcDebugTimer = 0;
    }
  }

  // Handle GPS debug output - clean summaries only when gnssPeriodicSummary is enabled
  if (gnssPeriodicSummary && gpsDebugTimer > GPS_DEBUG_INTERVAL)
  {
    Serial.print("\r\n");
    for (int i = 0; i < 60; i++)
      Serial.print("=");
    Serial.print("\r\n[GPS STATUS UPDATE - ");
    Serial.print(millis() / 1000);
    Serial.print("s]");

    if (gnssProcessor && gnssProcessor->hasBestPosition())
    {
      const auto &pos = gnssProcessor->getBestPosition();
      Serial.print("\r\nBest Position: ");
      Serial.print(pos.latitude, 7);
      Serial.print("°, ");
      Serial.print(pos.longitude, 7);
      Serial.print("°");
      Serial.print("\r\nSpeed: ");
      Serial.print(pos.speed, 1);
      Serial.print(" km/h, Heading: ");
      Serial.print(pos.heading, 1);
      Serial.print("°");
      Serial.print("\r\nFix Quality: ");
      Serial.print(pos.fixQuality);
      Serial.print(", Satellites: ");
      Serial.print(pos.satelliteCount);
      Serial.print(", HDOP: ");
      Serial.print(pos.hdop, 1);

      // Show both GPS receivers status
      Serial.print("\r\nGPS1: ");
      Serial.print(gnssProcessor->isGPS1Valid() ? "VALID" : "INVALID");
      Serial.print(" | GPS2: ");
      Serial.print(gnssProcessor->isGPS2Valid() ? "VALID" : "INVALID");
    }
    else
    {
      Serial.print("\r\nNo valid GPS position available");
      if (gnssProcessor)
      {
        Serial.print("\r\nGPS1: ");
        Serial.print(gnssProcessor->isGPS1Valid() ? "VALID" : "INVALID");
        Serial.print(" | GPS2: ");
        Serial.print(gnssProcessor->isGPS2Valid() ? "VALID" : "INVALID");
      }
    }

    Serial.print("\r\n");
    for (int i = 0; i < 60; i++)
      Serial.print("=");
    gpsDebugTimer = 0;
  }

  // Handle IMU debug output - clean summaries only when imuPeriodicSummary is enabled
  if (imuPeriodicSummary && imuDebugTimer > IMU_DEBUG_INTERVAL)
  {
    Serial.print("\r\n");
    for (int i = 0; i < 60; i++)
      Serial.print("-");
    Serial.print("\r\n[IMU STATUS UPDATE - ");
    Serial.print(millis() / 1000);
    Serial.print("s]");

    if (imuManager && imuManager->isIMUValid())
    {
      const auto &imu = imuManager->getIMUData();
      const auto &fuse = imuManager->getFUSEData();

      Serial.print("\r\nRaw IMU -> Roll: ");
      Serial.print(imu.roll, 1);
      Serial.print("°, Pitch: ");
      Serial.print(imu.pitch, 1);
      Serial.print("°, Heading: ");
      Serial.print(imu.heading, 1);
      Serial.print("°");

      if (imuManager->isFUSEEnabled() && imuManager->isFUSEActive())
      {
        Serial.print("\r\nFUSE Data -> Roll: ");
        Serial.print(fuse.fusedRoll, 1);
        Serial.print("°, Pitch: ");
        Serial.print(fuse.fusedPitch, 1);
        Serial.print("°, Heading: ");
        Serial.print(fuse.correctedHeading, 1);
        Serial.print("°");

        Serial.print("\r\nSmoothed -> Roll: ");
        Serial.print(imuManager->getSmoothedRoll(), 1);
        Serial.print("°, Pitch: ");
        Serial.print(imuManager->getSmoothedPitch(), 1);
        Serial.print("°, Heading: ");
        Serial.print(imuManager->getSmoothedHeading(), 1);
        Serial.print("°");
      }
      else
      {
        Serial.print("\r\nFUSE: DISABLED (Raw data only)");
      }

      Serial.print("\r\nCalibration: 0x");
      Serial.print(imu.calibrationStatus, HEX);
      Serial.print(" | Data Age: ");
      Serial.print(millis() - imu.lastUpdateTime);
      Serial.print("ms");
    }
    else
    {
      Serial.print("\r\nIMU data invalid or too old");
    }

    Serial.print("\r\n");
    for (int i = 0; i < 60; i++)
      Serial.print("-");
    imuDebugTimer = 0;
  }

  // Handle debug commands - DO THIS BEFORE DiagnosticManager
  if (Serial.available())
  {
    char command = Serial.read();

    // Check if it's a Phase 3 command first (capital letters)
    if (command >= 'A' && command <= 'Z')
    {
      // Handle Phase 3 commands directly
      handlePhase3Command(command);
    }
    else
    {
      // Let DiagnosticManager handle lowercase commands
      // Put the character back and let DiagnosticManager process it
      // Since we can't put it back, we'll handle the common ones here
      handleDiagnosticCommand(command);
    }
  }
}

// New function to handle Phase 3 commands separately
void handlePhase3Command(char command)
{
  Serial.print("\r\n[PHASE3] Received command: '");
  Serial.print(command);
  Serial.print("'");

  switch (command)
  {
  case 'G': // GNSS debug
    Serial.print("\r\n[PHASE3] Toggling GNSS debug");
    toggleGNSSDebug();
    break;

  case 'I': // IMU debug
    Serial.print("\r\n[PHASE3] Toggling IMU debug");
    toggleIMUDebug();
    break;

  case 'F': // FUSE toggle
    Serial.print("\r\n[PHASE3] Toggling FUSE calculations");
    toggleFUSE();
    break;

  case 'N': // NMEA verbose toggle
    Serial.print("\r\n[PHASE3] Toggling NMEA verbose logging");
    toggleNMEAVerbose();
    break;

  case 'V': // IMU verbose toggle
    Serial.print("\r\n[PHASE3] Toggling IMU verbose logging");
    toggleIMUVerbose();
    break;

  case 'P': // GPS position
    Serial.print("\r\n[PHASE3] Showing GPS status");
    printGPSStatus();
    break;

  case 'M': // IMU status
    Serial.print("\r\n[PHASE3] Showing IMU status");
    printIMUStatus();
    break;

  case 'D': // Debug object status
    Serial.print("\r\n[PHASE3] === Object & Debug Status ===");
    Serial.print("\r\n--- Object Status ---");
    Serial.print("\r\nconfigManager: ");
    Serial.print(configManager ? "OK" : "NULL");
    Serial.print("\r\nhardwareManager: ");
    Serial.print(hardwareManager ? "OK" : "NULL");
    Serial.print("\r\ndiagnosticManager: ");
    Serial.print(diagnosticManager ? "OK" : "NULL");
    Serial.print("\r\nnetworkManager: ");
    Serial.print(networkManager ? "OK" : "NULL");
    Serial.print("\r\nserialManager: ");
    Serial.print(serialManager ? "OK" : "NULL");
    Serial.print("\r\ncanManager: ");
    Serial.print(canManager ? "OK" : "NULL");
    Serial.print("\r\ngnssProcessor: ");
    Serial.print(gnssProcessor ? "OK" : "NULL");
    Serial.print("\r\nimuManager: ");
    Serial.print(imuManager ? "OK" : "NULL");

    Serial.print("\r\n--- Debug Mode Status ---");
    Serial.print("\r\nGPS Clean Summaries (G): ");
    Serial.print(gnssPeriodicSummary ? "ON" : "OFF");
    Serial.print("\r\nIMU Clean Summaries (I): ");
    Serial.print(imuPeriodicSummary ? "ON" : "OFF");

    if (gnssProcessor)
    {
      Serial.print("\r\nGNSS Class Debug (N): ");
      Serial.print(gnssProcessor->isDebugMode() ? "ON" : "OFF");
    }
    if (imuManager)
    {
      Serial.print("\r\nIMU Class Debug (V): ");
      Serial.print(imuManager->isDebugMode() ? "ON" : "OFF");
      Serial.print("\r\nFUSE Enabled (F): ");
      Serial.print(imuManager->isFUSEEnabled() ? "ON" : "OFF");
    }
    if (diagnosticManager)
    {
      Serial.print("\r\nADC Debug (a): ");
      Serial.print(diagnosticManager->getDebugFlag("adc") ? "ON" : "OFF");
    }
    if (canManager)
    {
      Serial.print("\r\nKeya Debug (k): ");
      Serial.print(canManager->isDebugMode() ? "ON" : "OFF");
    }

    Serial.print("\r\n===============================");
    break;

  default:
    Serial.print("\r\n[PHASE3] Unknown Phase 3 command: '");
    Serial.print(command);
    Serial.print("'");
    break;
  }
}

// Handle diagnostic commands that we want to support
void handleDiagnosticCommand(char command)
{
  Serial.print("\r\n[DIAG] Received command: '");
  Serial.print(command);
  Serial.print("'");

  switch (command)
  {
  case 'h':
    Serial.print("\r\n=== Combined Debug Commands ===");
    Serial.print("\r\n--- DiagnosticManager Commands (lowercase) ---");
    Serial.print("\r\nh - This help");
    Serial.print("\r\na - Toggle ADC debug");
    Serial.print("\r\nc - Toggle CPU usage display");
    Serial.print("\r\nk - Toggle Keya CAN debug");
    Serial.print("\r\nn - Toggle NMEA debug");
    Serial.print("\r\np - Toggle PWM debug");
    Serial.print("\r\ns - Toggle statistics display");
    Serial.print("\r\nr - Reset statistics");
    Serial.print("\r\ni - Show system information");
    Serial.print("\r\n--- Phase 3 Commands (CAPITAL) ---");
    Serial.print("\r\nG - Toggle GNSS debug (periodic summaries)");
    Serial.print("\r\nI - Toggle IMU debug (periodic summaries)");
    Serial.print("\r\nF - Toggle FUSE calculations");
    Serial.print("\r\nN - Toggle NMEA verbose logging (high-freq)");
    Serial.print("\r\nV - Toggle IMU verbose logging (high-freq)");
    Serial.print("\r\nP - Show GPS position status");
    Serial.print("\r\nM - Show IMU status");
    Serial.print("\r\nD - Debug object status");
    Serial.print("\r\nR - Reboot system");
    Serial.print("\r\n===============================");
    break;

  case 'a':
    if (diagnosticManager)
    {
      bool currentMode = diagnosticManager->getDebugFlag("adc");
      diagnosticManager->setDebugFlag("adc", !currentMode);
      Serial.print("\r\n[DIAG] ADC debug: ");
      Serial.print(!currentMode ? "ON" : "OFF");
      if (!currentMode && hardwareManager)
      {
        hardwareManager->printADCDebug();
      }
    }
    break;

  case 'k':
    Serial.print("\r\n[DIAG] ");
    toggleKeyaDebug();
    break;

  case 's':
    Serial.print("\r\n[DIAG] === All Statistics ===");
    printGPSStatus();
    printIMUStatus();
    printCANStatus();

    if (serialManager)
    {
      const auto &stats = serialManager->getStats();
      Serial.print("\r\n--- Serial Stats ---");
      Serial.print("\r\nGPS1 RX: ");
      Serial.print(stats.gps1BytesReceived);
      Serial.print(", TX: ");
      Serial.print(stats.gps1BytesSent);
      Serial.print("\r\nGPS2 RX: ");
      Serial.print(stats.gps2BytesReceived);
      Serial.print(", TX: ");
      Serial.print(stats.gps2BytesSent);
      Serial.print("\r\nIMU RX: ");
      Serial.print(stats.imuBytesReceived);
      Serial.print(", TX: ");
      Serial.print(stats.imuBytesSent);
    }

    if (diagnosticManager)
    {
      Serial.print("\r\n--- System Stats ---");
      Serial.print("\r\nLoop frequency: ");
      Serial.print(diagnosticManager->getLoopFrequency());
      Serial.print("kHz");
      Serial.print("\r\nCPU temperature: ");
      Serial.print(diagnosticManager->getCpuTemperature());
      Serial.print("°C");
      Serial.print("\r\nFree RAM: ");
      Serial.print(diagnosticManager->getFreeRAM());
      Serial.print(" bytes");
    }
    break;

  case 'r':
    Serial.print("\r\n[DIAG] Resetting all statistics...");
    if (serialManager)
      serialManager->resetStats();
    if (canManager)
      canManager->resetStatistics();
    if (gnssProcessor)
      gnssProcessor->resetStatistics();
    if (imuManager)
      imuManager->resetStatistics();
    if (diagnosticManager)
      diagnosticManager->resetStats();
    Serial.print("\r\n[DIAG] All statistics reset");
    break;

  case 'R':
    Serial.print("\r\n[DIAG] Rebooting system...");
    delay(100);
    SCB_AIRCR = 0x05FA0004;
    break;

  case 'c':
    if (diagnosticManager)
    {
      bool currentMode = diagnosticManager->getDebugFlag("cpu");
      diagnosticManager->setDebugFlag("cpu", !currentMode);
      Serial.print("\r\n[DIAG] CPU usage debug: ");
      Serial.print(!currentMode ? "ON" : "OFF");
    }
    break;

  case 'i':
    if (diagnosticManager)
    {
      Serial.print("\r\n[DIAG] System Information:");
      Serial.print("\r\nCPU Frequency: ");
      Serial.print(F_CPU_ACTUAL / 1000000);
      Serial.print("MHz");
      Serial.print("\r\nCPU Temperature: ");
      Serial.print(diagnosticManager->getCpuTemperature());
      Serial.print("°C");
      Serial.print("\r\nFree RAM: ");
      Serial.print(diagnosticManager->getFreeRAM());
      Serial.print(" bytes");
      Serial.print("\r\nLoop Frequency: ");
      Serial.print(diagnosticManager->getLoopFrequency());
      Serial.print("kHz");
      printCANStatusInSystemInfo();
    }
    break;

  case '\r':
  case '\n':
    // Ignore CR/LF
    break;

  default:
    Serial.print("\r\n[DIAG] Unknown diagnostic command: '");
    Serial.print(command);
    Serial.print("' - Press 'h' for help");
    break;
  }
}