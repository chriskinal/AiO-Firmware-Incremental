#include "IMUManager.h"
#include <cmath>

IMUManager::IMUManager(SerialManager &serialManager, DiagnosticManager &diagnosticManager)
    : serialManager_(serialManager), diagnosticManager_(diagnosticManager), fuseEnabled_(true), fuseAlpha_(0.98f), fuseBeta_(0.1f), headingOffset_(0.0f), imuMessagesProcessed_(0), imuParseErrors_(0), fuseCalculations_(0), imuUsage_("IMU"), debugEnabled_(false)
{
    // Initialize data structures
    memset(&imuData_, 0, sizeof(imuData_));
    memset(&fuseData_, 0, sizeof(fuseData_));
    memset(&ringBuffer_, 0, sizeof(ringBuffer_));
    memset(&rvcMessage_, 0, sizeof(rvcMessage_));
}

IMUManager::~IMUManager()
{
}

bool IMUManager::begin()
{
    Serial.print("\r\n- IMU Manager initialization");

    // Register CPU usage tracking
    diagnosticManager_.registerCpuUsage("IMU", &imuUsage_);

    // Initialize ring buffer
    clearRingBuffer();

    // Set default BNO mode (NDOF - 9DOF fusion)
    setBNOMode(BNO_MODE_NDOF);

    // Start calibration sequence
    delay(100);
    calibrateIMU();

    Serial.print("\r\n- IMU Manager initialization complete");
    return true;
}

void IMUManager::update()
{
    imuUsage_.timeIn();

    // Process incoming IMU data
    processIMUData();

    // Update FUSE calculations if enabled and data is valid
    if (fuseEnabled_ && fuseUpdateTimer_ > FUSE_UPDATE_INTERVAL)
    {
        if (isIMUValid())
        {
            updateFUSECalculations();
            fuseCalculations_++;
        }
        fuseUpdateTimer_ = 0;
    }

    // Update ring buffer for smoothing
    if (imuUpdateTimer_ > IMU_UPDATE_INTERVAL)
    {
        if (isIMUValid())
        {
            addToRingBuffer(imuData_.roll, imuData_.pitch, imuData_.heading);
        }
        imuUpdateTimer_ = 0;
    }

    imuUsage_.timeOut();
}

void IMUManager::processIMUData()
{
    while (serialManager_.isIMUAvailable())
    {
        uint8_t byte = serialManager_.readIMU();

        // Add byte to RVC message buffer
        if (rvcMessage_.index < sizeof(rvcMessage_.buffer))
        {
            rvcMessage_.buffer[rvcMessage_.index++] = byte;
        }
        else
        {
            // Buffer overflow - reset
            rvcMessage_.index = 0;
            imuParseErrors_++;
            continue;
        }

        // Check for complete RVC message (17 bytes total)
        if (rvcMessage_.index >= 17)
        {
            if (parseRVCMessage(rvcMessage_.buffer, rvcMessage_.index))
            {
                imuMessagesProcessed_++;

                if (debugEnabled_)
                {
                    printIMUDebug();
                }
            }
            else
            {
                imuParseErrors_++;
            }

            rvcMessage_.index = 0;
            rvcMessage_.messageComplete = false;
        }
    }
}

bool IMUManager::parseRVCMessage(const uint8_t *data, size_t length)
{
    if (!data || length < 17)
        return false;

    // BNO055 RVC format: YPR + Linear Accel + Gravity + Status
    // Bytes 0-1: Yaw (heading)
    // Bytes 2-3: Pitch
    // Bytes 4-5: Roll
    // Bytes 6-7: Linear Accel X
    // Bytes 8-9: Linear Accel Y
    // Bytes 10-11: Linear Accel Z
    // Bytes 12-13: Gravity X
    // Bytes 14-15: Gravity Y
    // Bytes 16: Status

    // Parse heading (yaw) - bytes 0-1, LSB first
    int16_t rawHeading = (int16_t)(data[1] << 8 | data[0]);
    imuData_.heading = rawHeading / 16.0f; // Scale factor for degrees

    // Parse pitch - bytes 2-3, LSB first
    int16_t rawPitch = (int16_t)(data[3] << 8 | data[2]);
    imuData_.pitch = rawPitch / 16.0f;

    // Parse roll - bytes 4-5, LSB first
    int16_t rawRoll = (int16_t)(data[5] << 8 | data[4]);
    imuData_.roll = rawRoll / 16.0f;

    // Parse calibration status from last byte
    imuData_.calibrationStatus = data[16];

    // Normalize angles
    imuData_.heading = normalizeAngle(imuData_.heading);
    imuData_.pitch = normalizeAngle(imuData_.pitch);
    imuData_.roll = normalizeAngle(imuData_.roll);

    // Validate data
    imuData_.dataValid = isIMUDataValid(imuData_);
    imuData_.lastUpdateTime = millis();

    return imuData_.dataValid;
}

void IMUManager::updateFUSECalculations()
{
    if (!isIMUValid())
        return;

    // Apply calibration offsets
    applyCalibrationOffsets();

    // Calculate complementary filter for roll and pitch
    calculateComplementaryFilter();

    // Calculate heading correction
    calculateHeadingCorrection();

    // Update FUSE timestamps
    fuseData_.lastFuseTime = millis();
    fuseData_.fuseActive = true;

    if (debugEnabled_)
    {
        printFUSEDebug();
    }
}

void IMUManager::calculateComplementaryFilter()
{
    // Simple complementary filter - can be enhanced with gyro data
    // For now, just apply smoothing and offset correction

    static float previousRoll = 0.0f;
    static float previousPitch = 0.0f;
    static bool firstRun = true;

    if (firstRun)
    {
        fuseData_.fusedRoll = imuData_.roll;
        fuseData_.fusedPitch = imuData_.pitch;
        previousRoll = imuData_.roll;
        previousPitch = imuData_.pitch;
        firstRun = false;
        return;
    }

    // Apply complementary filter
    fuseData_.fusedRoll = fuseAlpha_ * previousRoll + (1.0f - fuseAlpha_) * imuData_.roll;
    fuseData_.fusedPitch = fuseAlpha_ * previousPitch + (1.0f - fuseAlpha_) * imuData_.pitch;

    previousRoll = fuseData_.fusedRoll;
    previousPitch = fuseData_.fusedPitch;
}

void IMUManager::calculateHeadingCorrection()
{
    // Apply heading offset and normalize
    fuseData_.correctedHeading = normalizeAngle(imuData_.heading + headingOffset_);
    fuseData_.fusedHeading = fuseData_.correctedHeading;
}

void IMUManager::applyCalibrationOffsets()
{
    // Apply stored offsets to roll and pitch
    float correctedRoll = imuData_.roll - fuseData_.rollOffset;
    float correctedPitch = imuData_.pitch - fuseData_.pitchOffset;

    // Store corrected values back (non-destructive to raw data)
    fuseData_.fusedRoll = correctedRoll;
    fuseData_.fusedPitch = correctedPitch;
}

void IMUManager::addToRingBuffer(float roll, float pitch, float heading)
{
    ringBuffer_.rollBuffer[ringBuffer_.index] = roll;
    ringBuffer_.pitchBuffer[ringBuffer_.index] = pitch;
    ringBuffer_.headingBuffer[ringBuffer_.index] = heading;

    ringBuffer_.index++;
    if (ringBuffer_.index >= RingBuffer::BUFFER_SIZE)
    {
        ringBuffer_.index = 0;
        ringBuffer_.bufferFull = true;
    }

    if (!ringBuffer_.bufferFull)
    {
        ringBuffer_.count++;
    }
    else
    {
        ringBuffer_.count = RingBuffer::BUFFER_SIZE;
    }
}

float IMUManager::getSmoothedRoll() const
{
    return calculateRingBufferAverage(ringBuffer_.rollBuffer);
}

float IMUManager::getSmoothedPitch() const
{
    return calculateRingBufferAverage(ringBuffer_.pitchBuffer);
}

float IMUManager::getSmoothedHeading() const
{
    return calculateRingBufferAverage(ringBuffer_.headingBuffer);
}

float IMUManager::calculateRingBufferAverage(const float *buffer) const
{
    if (ringBuffer_.count == 0)
        return 0.0f;

    float sum = 0.0f;
    for (uint8_t i = 0; i < ringBuffer_.count; i++)
    {
        sum += buffer[i];
    }

    return sum / ringBuffer_.count;
}

void IMUManager::clearRingBuffer()
{
    memset(&ringBuffer_, 0, sizeof(ringBuffer_));
}

void IMUManager::setBNOMode(uint8_t mode)
{
    // Set BNO055 to config mode first
    setBNORegister(0x3D, BNO_MODE_CONFIG);
    delay(25);

    // Set desired mode
    setBNORegister(0x3D, mode);
    delay(25);

    if (debugEnabled_)
    {
        Serial.print("\r\nBNO055 mode set to: 0x");
        Serial.print(mode, HEX);
    }
}

void IMUManager::calibrateIMU()
{
    if (debugEnabled_)
    {
        Serial.print("\r\nStarting IMU calibration...");
    }

    startCalibration();
}

void IMUManager::startCalibration()
{
    // Reset calibration data
    fuseData_.rollOffset = 0.0f;
    fuseData_.pitchOffset = 0.0f;

    // Clear any existing calibration
    clearRingBuffer();

    if (debugEnabled_)
    {
        Serial.print("\r\nCalibration started - keep IMU steady");
    }
}

void IMUManager::resetIMU()
{
    // Send reset command to BNO055
    setBNORegister(0x3F, 0x20);
    delay(1000); // Wait for reset

    // Reinitialize
    setBNOMode(BNO_MODE_NDOF);
    delay(100);

    if (debugEnabled_)
    {
        Serial.print("\r\nIMU reset complete");
    }
}

void IMUManager::setIMUOffsets(float rollOffset, float pitchOffset)
{
    fuseData_.rollOffset = rollOffset;
    fuseData_.pitchOffset = pitchOffset;

    if (debugEnabled_)
    {
        Serial.print("\r\nIMU offsets set - Roll: ");
        Serial.print(rollOffset, 2);
        Serial.print("°, Pitch: ");
        Serial.print(pitchOffset, 2);
        Serial.print("°");
    }
}

void IMUManager::setFUSEParameters(float alpha, float beta)
{
    fuseAlpha_ = constrain(alpha, 0.0f, 1.0f);
    fuseBeta_ = constrain(beta, 0.0f, 1.0f);

    if (debugEnabled_)
    {
        Serial.print("\r\nFUSE parameters - Alpha: ");
        Serial.print(fuseAlpha_, 3);
        Serial.print(", Beta: ");
        Serial.print(fuseBeta_, 3);
    }
}

void IMUManager::setBNORegister(uint8_t reg, uint8_t value)
{
    // Send register write command via serial (if BNO is in UART mode)
    // This is a simplified implementation
    uint8_t command[4] = {0xAA, 0x00, reg, value};
    serialManager_.writeIMU(command, sizeof(command));
}

uint8_t IMUManager::readBNORegister(uint8_t reg)
{
    // Send register read command via serial
    uint8_t command[3] = {0xAA, 0x01, reg};
    serialManager_.writeIMU(command, sizeof(command));

    // Wait for response (simplified)
    delay(10);

    if (serialManager_.isIMUAvailable())
    {
        return serialManager_.readIMU();
    }

    return 0;
}

void IMUManager::resetStatistics()
{
    imuMessagesProcessed_ = 0;
    imuParseErrors_ = 0;
    fuseCalculations_ = 0;
}

bool IMUManager::isIMUDataValid(const IMUData &data) const
{
    // Check if angles are reasonable
    if (!isAngleReasonable(data.roll) ||
        !isAngleReasonable(data.pitch) ||
        !isAngleReasonable(data.heading))
    {
        return false;
    }

    // Check calibration status (at least partially calibrated)
    if (data.calibrationStatus == 0)
        return false;

    return true;
}

bool IMUManager::isAngleReasonable(float angle) const
{
    return (angle >= -180.0f && angle <= 180.0f);
}

bool IMUManager::isRateReasonable(float rate) const
{
    return (rate >= -500.0f && rate <= 500.0f); // degrees per second
}

float IMUManager::normalizeAngle(float angle) const
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

float IMUManager::angleDifference(float angle1, float angle2) const
{
    float diff = angle1 - angle2;
    return normalizeAngle(diff);
}

float IMUManager::degreesToRadians(float degrees) const
{
    return degrees * M_PI / 180.0f;
}

float IMUManager::radiansToDegrees(float radians) const
{
    return radians * 180.0f / M_PI;
}

void IMUManager::printIMUDebug() const
{
    Serial.print("\r\n[IMU] Roll:");
    Serial.print(imuData_.roll, 1);
    Serial.print("° Pitch:");
    Serial.print(imuData_.pitch, 1);
    Serial.print("° Heading:");
    Serial.print(imuData_.heading, 1);
    Serial.print("° Cal:0x");
    Serial.print(imuData_.calibrationStatus, HEX);
    Serial.print(" Valid:");
    Serial.print(imuData_.dataValid ? "YES" : "NO");
}

void IMUManager::printFUSEDebug() const
{
    Serial.print("\r\n[FUSE] FRoll:");
    Serial.print(fuseData_.fusedRoll, 1);
    Serial.print("° FPitch:");
    Serial.print(fuseData_.fusedPitch, 1);
    Serial.print("° CHead:");
    Serial.print(fuseData_.correctedHeading, 1);
    Serial.print("° Active:");
    Serial.print(fuseData_.fuseActive ? "YES" : "NO");
}

void IMUManager::printRingBufferDebug() const
{
    Serial.print("\r\n[RING] Count:");
    Serial.print(ringBuffer_.count);
    Serial.print(" SRoll:");
    Serial.print(getSmoothedRoll(), 1);
    Serial.print("° SPitch:");
    Serial.print(getSmoothedPitch(), 1);
    Serial.print("° SHead:");
    Serial.print(getSmoothedHeading(), 1);
    Serial.print("°");
}

void IMUManager::printCalibrationDebug() const
{
    Serial.print("\r\n[CAL] Status:0x");
    Serial.print(imuData_.calibrationStatus, HEX);
    Serial.print(" Roll offset:");
    Serial.print(fuseData_.rollOffset, 2);
    Serial.print("° Pitch offset:");
    Serial.print(fuseData_.pitchOffset, 2);
    Serial.print("° Head offset:");
    Serial.print(headingOffset_, 2);
    Serial.print("°");
}