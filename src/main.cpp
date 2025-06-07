// Phase 2 Test - Communication Classes with Keya Debug Toggle and ADC Debug
// This file tests compilation of Phase 2 classes along with Phase 1

#include "Arduino.h"

// Phase 1 classes
#include "ConfigManager.h"
#include "HardwareManager.h"
#include "DiagnosticManager.h"

// Phase 2 classes
#include "NetworkManager.h"
#include "SerialManager.h"
#include "CANManager.h"

// Mongoose external declarations needed for compilation
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
    // Only poll if manager is initialized
    static bool initialized = false;
    if (!initialized)
    {
      initialized = true;
      return;
    }
    mg_mgr_poll(&g_mgr, 1);
  }
}

// Global instances for testing - use pointers to prevent stack issues
ConfigManager *configManager = nullptr;
HardwareManager *hardwareManager = nullptr;
DiagnosticManager *diagnosticManager = nullptr;
NetworkManager *networkManager = nullptr;
SerialManager *serialManager = nullptr;
CANManager *canManager = nullptr;

// Timer for ADC debug output
elapsedMillis adcDebugTimer;
static constexpr uint32_t ADC_DEBUG_INTERVAL = 1000; // 1 second

// External functions for DiagnosticManager to avoid circular dependencies
void toggleKeyaDebug()
{
  if (canManager)
  {
    bool currentMode = canManager->isDebugMode();
    canManager->setDebugMode(!currentMode);
    Serial.print("\r\nKeya CAN debug: ");
    Serial.print(!currentMode ? "ON" : "OFF");
    if (!currentMode)
    {
      Serial.print("\r\n(Keya status messages will now be shown)");
    }
    else
    {
      Serial.print("\r\n(Keya status messages are now hidden)");
    }
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

// Test callback functions for network manager
void testPGNHandler(struct mg_connection *c, int ev, void *ev_data)
{
  Serial.print("\r\nPGN handler called");
}

void testRTCMHandler(struct mg_connection *c, int ev, void *ev_data)
{
  Serial.print("\r\nRTCM handler called");
}

void setup()
{
  delay(5000); // Longer delay for serial monitor
  Serial.begin(115200);

  // Wait for serial to be ready
  while (!Serial && millis() < 10000)
  {
    delay(10);
  }

  Serial.print("\r\n\n*** Phase 2 Test - Communication Classes with Debug Features ***");
  Serial.print("\r\nStarting safe initialization...");

  // Create objects on heap to avoid stack overflow
  configManager = new ConfigManager();
  hardwareManager = new HardwareManager();
  diagnosticManager = new DiagnosticManager();

  if (!configManager || !hardwareManager || !diagnosticManager)
  {
    Serial.print("\r\nFailed to allocate Phase 1 objects!");
    while (1)
      delay(1000);
  }

  // Initialize Phase 1 classes first
  Serial.print("\r\n\nInitializing Phase 1 classes...");

  configManager->begin();
  Serial.print("\r\n✓ ConfigManager initialized");

  hardwareManager->begin();
  Serial.print("\r\n✓ HardwareManager initialized");

  diagnosticManager->begin();
  Serial.print("\r\n✓ DiagnosticManager initialized");

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

  Serial.print("\r\n✓ Phase 2 objects created");

  // Link CAN Manager to DiagnosticManager for debug control
  diagnosticManager->setCANManager(canManager);
  Serial.print("\r\n✓ CANManager linked to DiagnosticManager");

  // Test SerialManager first (safest)
  Serial.print("\r\n\nTesting SerialManager...");
  serialManager->begin();
  Serial.print("\r\n✓ SerialManager initialized");

  Serial.print("\r\nGPS1 baud: ");
  Serial.print(serialManager->getGPS1Baud());
  Serial.print("\r\nGPS2 baud: ");
  Serial.print(serialManager->getGPS2Baud());

  // Test CANManager
  Serial.print("\r\n\nTesting CANManager...");
  if (canManager->begin())
  {
    Serial.print("\r\n✓ CANManager initialized");
    Serial.print("\r\nKeya detected: ");
    Serial.print(canManager->isKeyaDetected() ? "YES" : "NO");
    Serial.print("\r\nKeya debug mode: ");
    Serial.print(canManager->isDebugMode() ? "ON" : "OFF");
  }
  else
  {
    Serial.print("\r\n✗ CANManager initialization failed");
  }

  // Test NetworkManager last (most complex)
  Serial.print("\r\n\nTesting NetworkManager...");
  networkManager->setPGNHandler(testPGNHandler);
  networkManager->setRTCMHandler(testRTCMHandler);
  Serial.print("\r\n✓ NetworkManager callbacks set");
  Serial.print("\r\n! NetworkManager begin() skipped for safety");

  // Test ADC readings
  Serial.print("\r\n\nTesting ADC...");
  Serial.print("\r\nWAS (A15) raw reading: ");
  Serial.print(hardwareManager->readWAS());
  Serial.print(" (");
  Serial.print(hardwareManager->readWASVoltage(), 3);
  Serial.print("V)");

  Serial.print("\r\n\n*** Phase 2 Test Complete - All Debug Features Ready! ***");
  Serial.print("\r\nPress 'h' for help");
  Serial.print("\r\nPress 'a' to toggle ADC debug");
  Serial.print("\r\nPress 'k' to toggle Keya debug");
  Serial.print("\r\nKeya debugging is OFF by default (no message spam)");
  Serial.print("\r\nADC debugging is OFF by default");
}

void loop()
{
  // Only update safe components
  if (serialManager)
  {
    serialManager->update();
  }

  if (canManager)
  {
    canManager->update();
  }

  if (diagnosticManager)
  {
    diagnosticManager->updateSystemStats();
    diagnosticManager->processDebugCommands();

    // Handle ADC debug output if enabled
    if (diagnosticManager->getDebugFlag("adc") && adcDebugTimer > ADC_DEBUG_INTERVAL)
    {
      if (hardwareManager)
      {
        hardwareManager->printADCDebug();
      }
      adcDebugTimer = 0;
    }
  }

  // Handle simple debug commands (most are now handled by DiagnosticManager)
  if (Serial.available())
  {
    char command = Serial.read();

    // Debug: show what character was received
    Serial.print("\r\nReceived command in main: '");
    Serial.print(command);
    Serial.print("' (ASCII: ");
    Serial.print((int)command);
    Serial.print(")");

    if (command == 'h')
    {
      Serial.print("\r\n=== Phase 2 Test Commands ===");
      Serial.print("\r\nh - This help");
      Serial.print("\r\na - Toggle ADC debug");
      Serial.print("\r\nk - Toggle Keya CAN debug");
      Serial.print("\r\ns - Show statistics");
      Serial.print("\r\nr - Reset statistics");
      Serial.print("\r\nb - Toggle buffer overflow checking");
      Serial.print("\r\nR - Reboot");
      Serial.print("\r\n=============================");
      Serial.print("\r\nNote: Most commands are handled by DiagnosticManager");
    }
    else if (command == 'a')
    {
      Serial.print("\r\nExecuting ADC debug toggle...");
      if (diagnosticManager)
      {
        bool currentMode = diagnosticManager->getDebugFlag("adc");
        diagnosticManager->setDebugFlag("adc", !currentMode);
        Serial.print("\r\nADC debug: ");
        Serial.print(!currentMode ? "ON" : "OFF");
        if (!currentMode)
        {
          Serial.print("\r\n(ADC readings will be shown every second)");
          // Show immediate reading
          if (hardwareManager)
          {
            hardwareManager->printADCDebug();
          }
        }
        else
        {
          Serial.print("\r\n(ADC readings are now hidden)");
        }
      }
    }
    else if (command == 's')
    {
      Serial.print("\r\nExecuting statistics command...");
      if (serialManager)
      {
        const auto &stats = serialManager->getStats();
        Serial.print("\r\nSerial Stats - GPS1 RX: ");
        Serial.print(stats.gps1BytesReceived);
        Serial.print(", GPS2 RX: ");
        Serial.print(stats.gps2BytesReceived);
      }
      if (canManager)
      {
        Serial.print("\r\nCAN Stats - RX: ");
        Serial.print(canManager->getMessagesReceived());
        Serial.print(", TX: ");
        Serial.print(canManager->getMessagesSent());
        Serial.print("\r\nKeya debug: ");
        Serial.print(canManager->isDebugMode() ? "ON" : "OFF");
      }
      if (hardwareManager)
      {
        Serial.print("\r\nCurrent ADC readings:");
        hardwareManager->printADCDebug();
      }
      if (diagnosticManager)
      {
        Serial.print("\r\nADC debug: ");
        Serial.print(diagnosticManager->getDebugFlag("adc") ? "ON" : "OFF");
      }
    }
    else if (command == 'k')
    {
      Serial.print("\r\nExecuting Keya debug toggle...");
      toggleKeyaDebug();
    }
    else if (command == 'r')
    {
      Serial.print("\r\nExecuting reset command...");
      if (serialManager)
        serialManager->resetStats();
      if (canManager)
        canManager->resetStatistics();
      Serial.print("\r\nStatistics reset");
    }
    else if (command == 'b')
    {
      Serial.print("\r\nExecuting buffer toggle command...");
      if (serialManager)
      {
        static bool bufferCheckEnabled = false;
        bufferCheckEnabled = !bufferCheckEnabled;
        serialManager->setBufferOverflowCheckEnabled(bufferCheckEnabled);
        Serial.print("\r\nBuffer overflow checking: ");
        Serial.print(bufferCheckEnabled ? "ENABLED" : "DISABLED");
      }
      else
      {
        Serial.print("\r\nSerialManager not available");
      }
    }
    else if (command == 'R')
    {
      Serial.print("\r\nExecuting reboot command...");
      delay(100);
      SCB_AIRCR = 0x05FA0004;
    }
    else if (command != '\r' && command != '\n')
    {
      Serial.print("\r\nCommand passed to DiagnosticManager for processing");
      // Let DiagnosticManager handle all other commands
    }
  }

  delay(10); // Prevent tight loop
}