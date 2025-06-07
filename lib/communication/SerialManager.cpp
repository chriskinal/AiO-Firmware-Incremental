#include "SerialManager.h"

SerialManager::SerialManager(HardwareManager &hardwareManager, DiagnosticManager &diagnosticManager)
    : hardwareManager_(hardwareManager), diagnosticManager_(diagnosticManager), gps1_(nullptr), gps2_(nullptr), rtk_(nullptr), esp32_(nullptr), rs232_(nullptr), imu_(nullptr), usb1DTR_(false), usb2DTR_(false), prevUSB1DTR_(false), prevUSB2DTR_(false), gps1Baud_(HardwareManager::BAUD_GPS), gps2Baud_(HardwareManager::BAUD_GPS), gps1Usage_("GPS1"), gps2Usage_("GPS2"), rtkUsage_("RTK"), esp32Usage_("ESP32"), rs232Usage_("RS232"), bufferOverflowDetected_(false), bufferOverflowCheckEnabled_(false) // Disabled by default for testing
{
}

SerialManager::~SerialManager()
{
}

void SerialManager::begin()
{
    Serial.print("\r\n- Serial Manager initialization");

    // Register CPU usage tracking
    diagnosticManager_.registerCpuUsage("GPS1", &gps1Usage_);
    diagnosticManager_.registerCpuUsage("GPS2", &gps2Usage_);
    diagnosticManager_.registerCpuUsage("RTK", &rtkUsage_);
    diagnosticManager_.registerCpuUsage("ESP32", &esp32Usage_);
    diagnosticManager_.registerCpuUsage("RS232", &rs232Usage_);

    // Get serial port references from hardware manager
    initializeSerialPorts();

    Serial.print("\r\n- Serial Manager initialization complete");
}

void SerialManager::initializeSerialPorts()
{
    gps1_ = hardwareManager_.getSerialGPS1();
    gps2_ = hardwareManager_.getSerialGPS2();
    rtk_ = hardwareManager_.getSerialRTK();
    esp32_ = hardwareManager_.getSerialESP32();
    rs232_ = hardwareManager_.getSerialRS232();
    imu_ = hardwareManager_.getSerialIMU();

    Serial.print("\r\n  - Serial ports assigned");
}

void SerialManager::update()
{
    updateBridgeMode();

    // Only check buffer overflow if enabled
    if (bufferOverflowCheckEnabled_)
    {
        checkBufferOverflow();
    }
}

void SerialManager::updateBridgeMode()
{
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
    // Update USB1 DTR status
    usb1DTR_ = SerialUSB1.dtr();
    if (usb1DTR_ != prevUSB1DTR_)
    {
        Serial.print("\r\n- SerialUSB1 ");
        Serial.print(usb1DTR_ ? "bridged with GPS1" : "disconnected");

        if (usb1DTR_)
        {
            if (SerialUSB1.baud() == gps1Baud_)
            {
                Serial.print(", baud matches GPS1");
            }
        }
        else
        {
            // Revert to default baud when disconnected
            if (gps1Baud_ != HardwareManager::BAUD_GPS)
            {
                setGPS1Baud(HardwareManager::BAUD_GPS);
                Serial.print(", baud reverted to default");
            }
        }
        prevUSB1DTR_ = usb1DTR_;
    }

    // Handle baud rate changes in bridge mode
    if (usb1DTR_ && SerialUSB1.baud() != gps1Baud_)
    {
        setGPS1Baud(SerialUSB1.baud());
        Serial.print("\r\n- GPS1 baud changed to ");
        Serial.print(gps1Baud_);
    }
#endif

#if defined(USB_TRIPLE_SERIAL)
    // Update USB2 DTR status
    usb2DTR_ = SerialUSB2.dtr();
    if (usb2DTR_ != prevUSB2DTR_)
    {
        Serial.print("\r\n- SerialUSB2 ");
        Serial.print(usb2DTR_ ? "bridged with GPS2" : "disconnected");

        if (usb2DTR_)
        {
            if (SerialUSB2.baud() == gps2Baud_)
            {
                Serial.print(", baud matches GPS2");
            }
        }
        else
        {
            // Revert to default baud when disconnected
            if (gps2Baud_ != HardwareManager::BAUD_GPS)
            {
                setGPS2Baud(HardwareManager::BAUD_GPS);
                Serial.print(", baud reverted to default");
            }
        }
        prevUSB2DTR_ = usb2DTR_;
    }

    // Handle baud rate changes in bridge mode
    if (usb2DTR_ && SerialUSB2.baud() != gps2Baud_)
    {
        setGPS2Baud(SerialUSB2.baud());
        Serial.print("\r\n- GPS2 baud changed to ");
        Serial.print(gps2Baud_);
    }
#endif
}

void SerialManager::handleBridgeGPS1()
{
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
    if (usb1DTR_ && gps1_)
    {
        // GPS1 to USB1
        if (gps1_->available())
        {
            while (gps1_->available())
            {
                SerialUSB1.write(gps1_->read());
            }
        }

        // USB1 to GPS1
        if (SerialUSB1.available())
        {
            while (SerialUSB1.available())
            {
                gps1_->write(SerialUSB1.read());
            }
        }
    }
#endif
}

void SerialManager::handleBridgeGPS2()
{
#if defined(USB_TRIPLE_SERIAL)
    if (usb2DTR_ && gps2_)
    {
        // GPS2 to USB2
        if (gps2_->available())
        {
            while (gps2_->available())
            {
                SerialUSB2.write(gps2_->read());
            }
        }

        // USB2 to GPS2
        if (SerialUSB2.available())
        {
            while (SerialUSB2.available())
            {
                gps2_->write(SerialUSB2.read());
            }
        }
    }
#endif
}

// GPS1 communication
bool SerialManager::isGPS1Available() const
{
    return gps1_ && gps1_->available();
}

uint8_t SerialManager::readGPS1()
{
    gps1Usage_.timeIn();
    uint8_t data = gps1_->read();
    stats_.gps1BytesReceived++;
    gps1Usage_.timeOut();
    return data;
}

size_t SerialManager::writeGPS1(const uint8_t *data, size_t length)
{
    if (!gps1_ || !data)
        return 0;
    size_t written = gps1_->write(data, length);
    stats_.gps1BytesSent += written;
    return written;
}

size_t SerialManager::writeGPS1(uint8_t data)
{
    if (!gps1_)
        return 0;
    size_t written = gps1_->write(data);
    stats_.gps1BytesSent += written;
    return written;
}

void SerialManager::clearGPS1()
{
    if (gps1_)
    {
        while (gps1_->available())
            gps1_->read();
        Serial.print("\r\n- GPS1 buffer cleared");
    }
}

// GPS2 communication
bool SerialManager::isGPS2Available() const
{
    return gps2_ && gps2_->available();
}

uint8_t SerialManager::readGPS2()
{
    gps2Usage_.timeIn();
    uint8_t data = gps2_->read();
    stats_.gps2BytesReceived++;
    gps2Usage_.timeOut();
    return data;
}

size_t SerialManager::writeGPS2(const uint8_t *data, size_t length)
{
    if (!gps2_ || !data)
        return 0;
    size_t written = gps2_->write(data, length);
    stats_.gps2BytesSent += written;
    return written;
}

size_t SerialManager::writeGPS2(uint8_t data)
{
    if (!gps2_)
        return 0;
    size_t written = gps2_->write(data);
    stats_.gps2BytesSent += written;
    return written;
}

void SerialManager::clearGPS2()
{
    if (gps2_)
    {
        while (gps2_->available())
            gps2_->read();
        Serial.print("\r\n- GPS2 buffer cleared");
    }
}

// RTK communication
bool SerialManager::isRTKAvailable() const
{
    return rtk_ && rtk_->available();
}

uint8_t SerialManager::readRTK()
{
    rtkUsage_.timeIn();
    uint8_t data = rtk_->read();
    stats_.rtkBytesReceived++;
    rtkUsage_.timeOut();
    return data;
}

size_t SerialManager::writeRTK(const uint8_t *data, size_t length)
{
    if (!rtk_ || !data)
        return 0;
    size_t written = rtk_->write(data, length);
    stats_.rtkBytesSent += written;
    return written;
}

size_t SerialManager::writeRTK(uint8_t data)
{
    if (!rtk_)
        return 0;
    size_t written = rtk_->write(data);
    stats_.rtkBytesSent += written;
    return written;
}

// ESP32 communication
bool SerialManager::isESP32Available() const
{
    return esp32_ && esp32_->available();
}

uint8_t SerialManager::readESP32()
{
    esp32Usage_.timeIn();
    uint8_t data = esp32_->read();
    stats_.esp32BytesReceived++;
    esp32Usage_.timeOut();
    return data;
}

size_t SerialManager::writeESP32(const uint8_t *data, size_t length)
{
    if (!esp32_ || !data)
        return 0;
    size_t written = esp32_->write(data, length);
    stats_.esp32BytesSent += written;
    return written;
}

size_t SerialManager::writeESP32(uint8_t data)
{
    if (!esp32_)
        return 0;
    size_t written = esp32_->write(data);
    stats_.esp32BytesSent += written;
    return written;
}

void SerialManager::writeESP32Line()
{
    if (esp32_)
    {
        esp32_->println();
        stats_.esp32BytesSent += 2; // CR+LF
    }
}

// RS232 communication
bool SerialManager::isRS232Available() const
{
    return rs232_ && rs232_->available();
}

uint8_t SerialManager::readRS232()
{
    rs232Usage_.timeIn();
    uint8_t data = rs232_->read();
    stats_.rs232BytesReceived++;
    rs232Usage_.timeOut();
    return data;
}

size_t SerialManager::writeRS232(const uint8_t *data, size_t length)
{
    if (!rs232_ || !data)
        return 0;
    size_t written = rs232_->write(data, length);
    stats_.rs232BytesSent += written;
    return written;
}

size_t SerialManager::writeRS232(uint8_t data)
{
    if (!rs232_)
        return 0;
    size_t written = rs232_->write(data);
    stats_.rs232BytesSent += written;
    return written;
}

// IMU communication
bool SerialManager::isIMUAvailable() const
{
    return imu_ && imu_->available();
}

uint8_t SerialManager::readIMU()
{
    uint8_t data = imu_->read();
    stats_.imuBytesReceived++;
    return data;
}

size_t SerialManager::writeIMU(const uint8_t *data, size_t length)
{
    if (!imu_ || !data)
        return 0;
    size_t written = imu_->write(data, length);
    stats_.imuBytesSent += written;
    return written;
}

size_t SerialManager::writeIMU(uint8_t data)
{
    if (!imu_)
        return 0;
    size_t written = imu_->write(data);
    stats_.imuBytesSent += written;
    return written;
}

// Baud rate management
void SerialManager::setGPS1Baud(uint32_t baud)
{
    if (gps1_ && baud != gps1Baud_)
    {
        gps1_->begin(baud);
        gps1Baud_ = baud;
    }
}

void SerialManager::setGPS2Baud(uint32_t baud)
{
    if (gps2_ && baud != gps2Baud_)
    {
        gps2_->begin(baud);
        gps2Baud_ = baud;
    }
}

// Statistics and monitoring
void SerialManager::resetStats()
{
    stats_ = SerialStats{};
}

bool SerialManager::checkBufferOverflow()
{
    bool overflow = false;

    // Only check for overflow if there's actually data in the buffers
    if (gps1_ && gps1_->available() > 200)
    {
        if (isBufferNearFull(gps1_))
        {
            clearGPS1();
            overflow = true;
        }
    }

    if (gps2_ && gps2_->available() > 200)
    {
        if (isBufferNearFull(gps2_))
        {
            clearGPS2();
            overflow = true;
        }
    }

    // Only report overflow if it actually occurred and wasn't recently reported
    if (overflow && !bufferOverflowDetected_)
    {
        Serial.print("\r\n- Serial buffer overflow detected and cleared");
        bufferOverflowDetected_ = true;
    }
    else if (!overflow)
    {
        bufferOverflowDetected_ = false;
    }

    return overflow;
}

void SerialManager::clearAllBuffers()
{
    clearGPS1();
    clearGPS2();

    if (rtk_)
        while (rtk_->available())
            rtk_->read();
    if (esp32_)
        while (esp32_->available())
            esp32_->read();
    if (rs232_)
        while (rs232_->available())
            rs232_->read();
    if (imu_)
        while (imu_->available())
            imu_->read();

    Serial.print("\r\n- All serial buffers cleared");
}

bool SerialManager::isBufferNearFull(HardwareSerial *port) const
{
    if (!port)
        return false;

    // Only consider it "near full" if there's actually a lot of data
    // available() returns the number of bytes available to read
    // For Teensy, typical buffer sizes are 64-256 bytes
    // Only trigger overflow if buffer has more than 200 bytes waiting
    return port->available() > 200;
}