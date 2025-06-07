#include "CANManager.h"

// Keya command templates
const uint8_t CANManager::keyaDisableCommand_[4] = {0x23, 0x0C, 0x20, 0x01};
const uint8_t CANManager::keyaDisableResponse_[4] = {0x60, 0x0C, 0x20, 0x00};
const uint8_t CANManager::keyaEnableCommand_[4] = {0x23, 0x0D, 0x20, 0x01};
const uint8_t CANManager::keyaEnableResponse_[4] = {0x60, 0x0D, 0x20, 0x00};
const uint8_t CANManager::keyaSpeedCommand_[4] = {0x23, 0x00, 0x20, 0x01};
const uint8_t CANManager::keyaSpeedResponse_[4] = {0x60, 0x00, 0x20, 0x00};
const uint8_t CANManager::keyaCurrentQuery_[4] = {0x40, 0x00, 0x21, 0x01};
const uint8_t CANManager::keyaCurrentResponse_[4] = {0x60, 0x00, 0x21, 0x01};
const uint8_t CANManager::keyaFaultQuery_[4] = {0x40, 0x12, 0x21, 0x01};
const uint8_t CANManager::keyaFaultResponse_[4] = {0x60, 0x12, 0x21, 0x01};
const uint8_t CANManager::keyaVoltageQuery_[4] = {0x40, 0x0D, 0x21, 0x02};
const uint8_t CANManager::keyaVoltageResponse_[4] = {0x60, 0x0D, 0x21, 0x02};
const uint8_t CANManager::keyaTemperatureQuery_[4] = {0x40, 0x0F, 0x21, 0x01};
const uint8_t CANManager::keyaTemperatureResponse_[4] = {0x60, 0x0F, 0x21, 0x01};
const uint8_t CANManager::keyaVersionQuery_[4] = {0x40, 0x01, 0x11, 0x11};
const uint8_t CANManager::keyaVersionResponse_[4] = {0x60, 0x01, 0x11, 0x11};

CANManager::CANManager(DiagnosticManager &diagnosticManager)
    : diagnosticManager_(diagnosticManager), keyaDetected_(false), keyaMotorStatus_(false), keyaCurrentReading_(0), messagesReceived_(0), messagesSent_(0), canUsage_("CAN"), debugEnabled_(false)
{
}

CANManager::~CANManager()
{
}

bool CANManager::begin()
{
    Serial.print("\r\n- CAN Manager initialization");

    // Register CPU usage tracking
    diagnosticManager_.registerCpuUsage("CAN", &canUsage_);

    // Initialize CAN3 for Keya
    keyaBus_.begin();
    keyaBus_.setBaudRate(KEYA_BAUD);

    Serial.print("\r\n  - CAN3 initialized at ");
    Serial.print(keyaBus_.getBaudRate());
    Serial.print(" bps for Keya motor");

    delay(100); // Allow CAN to settle

    Serial.print("\r\n- CAN Manager initialization complete");
    return true;
}

void CANManager::update()
{
    canUsage_.timeIn();

    // Process incoming CAN messages
    processKeyaMessages();

    // Periodic current query for motors with slow heartbeat
    if (keyaCurrentUpdateTimer_ > KEYA_CURRENT_UPDATE_INTERVAL)
    {
        queryKeyaCurrent();
        keyaCurrentUpdateTimer_ = 0;
    }

    canUsage_.timeOut();
}

void CANManager::processKeyaMessages()
{
    static uint32_t keyaCheckTime = 0;
    uint32_t currentTime = millis();

    // Only check for new data every millisecond
    if (currentTime < keyaCheckTime)
        return;
    keyaCheckTime = currentTime + 1;

    CAN_message_t receivedMessage;
    if (keyaBus_.read(receivedMessage))
    {
        messagesReceived_++;

        if (debugEnabled_)
        {
            printCANMessage(receivedMessage.id, receivedMessage.buf, "RX");
        }

        // Handle different message types
        if (receivedMessage.id == 0x07000001)
        {
            handleHeartbeat(receivedMessage);
        }
        else if (receivedMessage.id == 0x05800001)
        {
            handleQueryResponse(receivedMessage);
        }
    }
}

void CANManager::handleHeartbeat(const CAN_message_t &message)
{
    // Heartbeat pattern: 00:07:00:00:00:00:00:[ID]
    if (!keyaDetected_)
    {
        Serial.print("\r\n- Keya motor detected via heartbeat");
        keyaDetected_ = true;
        queryKeyaVersion(); // Query version on first detection
    }

    if (debugEnabled_)
    {
        static uint32_t lastHeartbeatTime = 0;
        uint32_t currentTime = millis();
        Serial.print("\r\nHeartbeat interval: ");
        Serial.print(currentTime - lastHeartbeatTime);
        Serial.print("ms");
        lastHeartbeatTime = currentTime;

        // Parse speed from heartbeat
        int16_t speed = 0;
        if (message.buf[2] == 0xFF)
        {
            speed = -(255 - message.buf[3]);
        }
        else
        {
            speed = message.buf[3];
        }
        Serial.print(" Speed: ");
        Serial.print(speed);

        // Parse current from heartbeat
        int16_t current = 0;
        if (message.buf[4] == 0xFF)
        {
            current = -(255 - message.buf[5]);
        }
        else
        {
            current = message.buf[5];
        }
        Serial.print(" Current: ");
        Serial.print(current);
    }

    // Check for motor diagnostics/errors
    if (message.buf[7] != 0)
    {
        handleMotorDiagnostics(message.buf[7]);
    }

    if (message.buf[6] != 0)
    {
        handleMotorStatus(message.buf[6]);
    }

    // Update motor status from diagnostic bits
    keyaMotorStatus_ = !bitRead(message.buf[7], 0); // Motor disabled bit inverted
}

void CANManager::handleQueryResponse(const CAN_message_t &message)
{
    // Current query response
    if (isPatternMatch(message, keyaCurrentResponse_, sizeof(keyaCurrentResponse_)))
    {
        keyaCurrentReading_ = message.buf[4] * 2.5; // Scale for display in "amps"

        if (debugEnabled_)
        {
            Serial.print("\r\nCurrent reading: ");
            Serial.print(keyaCurrentReading_ / 2.5);
            Serial.print("A");
        }
        return;
    }

    // Version query response
    if (isPatternMatch(message, keyaVersionResponse_, sizeof(keyaVersionResponse_)))
    {
        if (debugEnabled_)
        {
            Serial.print("\r\nKeya version: ");
            Serial.print(message.buf[4]);
            Serial.print(".");
            Serial.print(message.buf[5]);
            Serial.print(".");
            Serial.print(message.buf[6]);
            Serial.print(".");
            Serial.print(message.buf[7]);
        }
        return;
    }

    // Fault query response
    if (isPatternMatch(message, keyaFaultResponse_, sizeof(keyaFaultResponse_)))
    {
        if (debugEnabled_)
        {
            Serial.print("\r\nFault status: ");
            Serial.print(message.buf[4]);
            Serial.print(":");
            Serial.print(message.buf[5]);
        }
        return;
    }

    // Voltage query response
    if (isPatternMatch(message, keyaVoltageResponse_, sizeof(keyaVoltageResponse_)))
    {
        if (debugEnabled_)
        {
            Serial.print("\r\nVoltage: ");
            Serial.print(message.buf[4]);
            Serial.print("V");
        }
        return;
    }

    // Temperature query response
    if (isPatternMatch(message, keyaTemperatureResponse_, sizeof(keyaTemperatureResponse_)))
    {
        if (debugEnabled_)
        {
            Serial.print("\r\nTemperature: ");
            Serial.print(message.buf[4]);
            Serial.print("°C");
        }
        return;
    }

    // Handle other response types
    if (isPatternMatch(message, keyaDisableResponse_, sizeof(keyaDisableResponse_)))
    {
        if (debugEnabled_)
            Serial.print("\r\nDisable confirmed");
    }
    else if (isPatternMatch(message, keyaEnableResponse_, sizeof(keyaEnableResponse_)))
    {
        if (debugEnabled_)
            Serial.print("\r\nEnable confirmed");
    }
    else if (isPatternMatch(message, keyaSpeedResponse_, sizeof(keyaSpeedResponse_)))
    {
        if (debugEnabled_)
            Serial.print("\r\nSpeed command confirmed");
    }
    else
    {
        if (debugEnabled_)
        {
            printCANMessage(message.id, message.buf, "Unknown response");
        }
    }
}

void CANManager::handleMotorDiagnostics(uint8_t errorByte)
{
    if (bitRead(errorByte, 0))
    {
        Serial.print("\r\n- Keya motor disabled");
    }
    if (bitRead(errorByte, 1))
    {
        Serial.print("\r\n- Keya over voltage");
    }
    if (bitRead(errorByte, 2))
    {
        Serial.print("\r\n- Keya hardware protection");
    }
    if (bitRead(errorByte, 3))
    {
        Serial.print("\r\n- Keya E2PROM error");
    }
    if (bitRead(errorByte, 4))
    {
        Serial.print("\r\n- Keya under voltage");
    }
    if (bitRead(errorByte, 6))
    {
        Serial.print("\r\n- Keya over current");
    }
    if (bitRead(errorByte, 7))
    {
        Serial.print("\r\n- Keya mode failure");
    }
}

void CANManager::handleMotorStatus(uint8_t statusByte)
{
    if (bitRead(statusByte, 0))
    {
        Serial.print("\r\n- Keya less phase");
    }
    if (bitRead(statusByte, 1))
    {
        Serial.print("\r\n- Keya motor stall");
    }
    if (bitRead(statusByte, 3))
    {
        Serial.print("\r\n- Keya hall failure");
    }
    if (bitRead(statusByte, 4))
    {
        Serial.print("\r\n- Keya current sensing error");
    }
    if (bitRead(statusByte, 5))
    {
        Serial.print("\r\n- Keya 232 disconnected");
    }
    if (bitRead(statusByte, 6))
    {
        Serial.print("\r\n- Keya CAN disconnected");
    }
    if (bitRead(statusByte, 7))
    {
        Serial.print("\r\n- Keya motor stalled");
    }
}

void CANManager::steerKeya(int16_t steerSpeed)
{
    if (!keyaDetected_)
        return;

    if (steerSpeed == 0)
    {
        sendKeyaCommand(keyaDisableCommand_);
        if (debugEnabled_)
        {
            Serial.print("\r\nSteering disabled (speed = 0)");
        }
        return;
    }

    // Send speed command
    CAN_message_t speedMessage;
    speedMessage.id = KEYA_ID;
    speedMessage.flags.extended = true;
    speedMessage.len = 8;
    memcpy(speedMessage.buf, keyaSpeedCommand_, 4);

    mapSpeedToPWM(steerSpeed, speedMessage);

    if (keyaBus_.write(speedMessage))
    {
        messagesSent_++;

        if (debugEnabled_)
        {
            Serial.print("\r\nSpeed command sent: ");
            Serial.print(steerSpeed);
        }
    }

    // Send enable command
    sendKeyaCommand(keyaEnableCommand_);
}

void CANManager::mapSpeedToPWM(int16_t steerSpeed, CAN_message_t &message)
{
    int16_t actualSpeed = map(steerSpeed, -255, 255, -995, 995);

    if (steerSpeed < 0)
    {
        // Clockwise rotation
        message.buf[4] = highByte(actualSpeed);
        message.buf[5] = lowByte(actualSpeed);
        message.buf[6] = 0xFF;
        message.buf[7] = 0xFF;
    }
    else
    {
        // Counter-clockwise rotation
        message.buf[4] = highByte(actualSpeed);
        message.buf[5] = lowByte(actualSpeed);
        message.buf[6] = 0x00;
        message.buf[7] = 0x00;
    }
}

void CANManager::sendKeyaCommand(const uint8_t command[])
{
    if (!keyaDetected_)
        return;

    CAN_message_t message;
    message.id = KEYA_ID;
    message.flags.extended = true;
    message.len = 8;
    memcpy(message.buf, command, 4);
    memset(&message.buf[4], 0, 4); // Clear remaining bytes

    if (keyaBus_.write(message))
    {
        messagesSent_++;
    }
}

void CANManager::queryKeyaCurrent()
{
    sendKeyaCommand(keyaCurrentQuery_);
}

void CANManager::queryKeyaVersion()
{
    sendKeyaCommand(keyaVersionQuery_);
}

void CANManager::queryKeyaFault()
{
    sendKeyaCommand(keyaFaultQuery_);
}

void CANManager::queryKeyaVoltage()
{
    sendKeyaCommand(keyaVoltageQuery_);
}

void CANManager::queryKeyaTemperature()
{
    sendKeyaCommand(keyaTemperatureQuery_);
}

void CANManager::resetStatistics()
{
    messagesReceived_ = 0;
    messagesSent_ = 0;
}

bool CANManager::isPatternMatch(const CAN_message_t &message, const uint8_t *pattern, size_t patternSize)
{
    return memcmp(message.buf, pattern, patternSize) == 0;
}

void CANManager::printCANMessage(uint32_t id, const uint8_t data[8], const char *description)
{
    Serial.print("\r\nCAN ");
    if (description)
    {
        Serial.print(description);
        Serial.print(" ");
    }
    Serial.print("ID:0x");
    Serial.print(id, HEX);
    Serial.print(" Data:");

    for (uint8_t i = 0; i < 8; i++)
    {
        if (data[i] < 16)
            Serial.print("0");
        Serial.print(data[i], HEX);
        if (i < 7)
            Serial.print(":");
    }
}