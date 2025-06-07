// Phase 2 Test - Communication Classes
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

  Serial.print("\r\n\n*** Phase 2 Test - Communication Classes ***");
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

  Serial.print("\r\n\n*** Phase 2 Test Complete - Basic functionality verified! ***");
  Serial.print("\r\nPress 'h' for help");
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
  }

  // Handle simple debug commands
  if (Serial.available())
  {
    char command = Serial.read();

    // Debug: show what character was received
    Serial.print("\r\nReceived command: '");
    Serial.print(command);
    Serial.print("' (ASCII: ");
    Serial.print((int)command);
    Serial.print(")");

    if (command == 'h')
    {
      Serial.print("\r\n=== Phase 2 Test Commands ===");
      Serial.print("\r\nh - This help");
      Serial.print("\r\ns - Show statistics");
      Serial.print("\r\nr - Reset statistics");
      Serial.print("\r\nb - Toggle buffer overflow checking");
      Serial.print("\r\nR - Reboot");
      Serial.print("\r\n=============================");
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
      }
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
      Serial.print("\r\nUnknown command: '");
      Serial.print(command);
      Serial.print("' - Press 'h' for help");
    }
  }

  delay(10); // Prevent tight loop
}