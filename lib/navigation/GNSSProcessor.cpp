#include "GNSSProcessor.h"
#include <cmath>

GNSSProcessor::GNSSProcessor(SerialManager &serialManager, DiagnosticManager &diagnosticManager)
    : serialManager_(serialManager), diagnosticManager_(diagnosticManager), nmeaParsingEnabled_(true), ubxIndex_(0), ubxParsingActive_(false), gps1SentencesProcessed_(0), gps2SentencesProcessed_(0), gps1ParseErrors_(0), gps2ParseErrors_(0), gpsProcessorUsage_("GPS_PROC"), debugEnabled_(false)
{
    // Initialize NMEA sentence structures
    memset(&gps1Sentence_, 0, sizeof(gps1Sentence_));
    memset(&gps2Sentence_, 0, sizeof(gps2Sentence_));
    memset(ubxBuffer_, 0, sizeof(ubxBuffer_));
}

GNSSProcessor::~GNSSProcessor()
{
}

bool GNSSProcessor::begin()
{
    Serial.print("\r\n- GNSS Processor initialization");

    // Register CPU usage tracking
    diagnosticManager_.registerCpuUsage("GPS_PROC", &gpsProcessorUsage_);

    // Initialize position data
    gps1Data_ = PositionData{};
    gps2Data_ = PositionData{};
    bestPosition_ = PositionData{};

    Serial.print("\r\n- GNSS Processor initialization complete");
    return true;
}

void GNSSProcessor::update()
{
    gpsProcessorUsage_.timeIn();

    if (nmeaParsingEnabled_)
    {
        processGPS1Data();
        processGPS2Data();
    }

    // Update best position every 100ms
    if (gpsUpdateTimer_ > GPS_UPDATE_INTERVAL)
    {
        updateBestPosition();
        gpsUpdateTimer_ = 0;
    }

    gpsProcessorUsage_.timeOut();
}

void GNSSProcessor::processGPS1Data()
{
    while (serialManager_.isGPS1Available())
    {
        char c = serialManager_.readGPS1();

        if (parseNMEAChar(c, gps1Sentence_, gps1Data_, "GPS1"))
        {
            gps1SentencesProcessed_++;
        }
    }
}

void GNSSProcessor::processGPS2Data()
{
    while (serialManager_.isGPS2Available())
    {
        char c = serialManager_.readGPS2();

        if (parseNMEAChar(c, gps2Sentence_, gps2Data_, "GPS2"))
        {
            gps2SentencesProcessed_++;
        }
    }
}

bool GNSSProcessor::parseNMEAChar(char c, NMEASentence &sentence, PositionData &data, const char *source)
{
    // Handle sentence start
    if (c == '$')
    {
        sentence.length = 0;
        sentence.complete = false;
        sentence.sentence[sentence.length++] = c;
        return false;
    }

    // Ignore if no sentence started
    if (sentence.length == 0)
        return false;

    // Add character to sentence
    if (sentence.length < sizeof(sentence.sentence) - 1)
    {
        sentence.sentence[sentence.length++] = c;
    }
    else
    {
        // Buffer overflow - restart
        sentence.length = 0;
        if (source == "GPS1")
            gps1ParseErrors_++;
        else
            gps2ParseErrors_++;
        return false;
    }

    // Check for sentence end
    if (c == '\n' || c == '\r')
    {
        sentence.sentence[sentence.length] = '\0';
        sentence.complete = true;

        if (sentence.length > 6) // Minimum valid sentence length
        {
            processCompleteSentence(sentence, data, source);
            return true;
        }
        else
        {
            if (source == "GPS1")
                gps1ParseErrors_++;
            else
                gps2ParseErrors_++;
        }

        sentence.length = 0;
        sentence.complete = false;
    }

    return false;
}

void GNSSProcessor::processCompleteSentence(const NMEASentence &sentence, PositionData &data, const char *source)
{
    if (!validateNMEAChecksum(sentence.sentence))
    {
        if (debugEnabled_)
        {
            Serial.print("\r\n[");
            Serial.print(source);
            Serial.print("] Checksum error in: ");
            Serial.print(sentence.sentence);
        }
        if (source == "GPS1")
            gps1ParseErrors_++;
        else
            gps2ParseErrors_++;
        return;
    }

    // Extract talker and message IDs
    extractTalkerAndMessage(sentence.sentence, sentence.talkerID, sentence.messageID);

    if (debugEnabled_)
    {
        printNMEADebug(sentence.sentence, source);
    }

    // Parse different sentence types
    bool parsed = false;
    if (strncmp(sentence.messageID, "GGA", 3) == 0)
    {
        parsed = parseGGA(sentence.sentence, data);
    }
    else if (strncmp(sentence.messageID, "RMC", 3) == 0)
    {
        parsed = parseRMC(sentence.sentence, data);
    }
    else if (strncmp(sentence.messageID, "VTG", 3) == 0)
    {
        parsed = parseVTG(sentence.sentence, data);
    }
    else if (strncmp(sentence.messageID, "GSA", 3) == 0)
    {
        parsed = parseGSA(sentence.sentence, data);
    }
    else if (strncmp(sentence.messageID, "GSV", 3) == 0)
    {
        parsed = parseGSV(sentence.sentence, data);
    }
    else if (strncmp(sentence.messageID, "HPR", 3) == 0)
    {
        parsed = parseHPR(sentence.sentence, data);
    }
    else if (strncmp(sentence.sentence, "$KSXT", 5) == 0)
    {
        parsed = parseKSXT(sentence.sentence, data);
    }

    if (parsed)
    {
        data.lastUpdateTime = millis();
        updatePositionQuality(data);

        if (debugEnabled_)
        {
            printPositionDebug(data, source);
        }
    }
}

bool GNSSProcessor::parseGGA(const char *sentence, PositionData &data)
{
    // $GNGGA,155812.70,3235.42578812,N,08710.82386611,W,1,27,0.6,58.8092,M,-29.6648,M,,*75
    const char *field = sentence;
    char fieldBuffer[32];

    // Skip $GNGGA (field 0)
    field = findNextField(field);
    if (!field)
        return false;

    // Skip time field (field 1)
    field = findNextField(field);
    if (!field)
        return false;

    // Extract latitude (field 2)
    if (!extractField(field, fieldBuffer, sizeof(fieldBuffer)) || isFieldEmpty(fieldBuffer))
        return false;
    char latStr[32];
    strcpy(latStr, fieldBuffer);

    // Move to latitude direction (field 3)
    field = findNextField(field);
    if (!field)
        return false;

    if (!extractField(field, fieldBuffer, sizeof(fieldBuffer)) || isFieldEmpty(fieldBuffer))
        return false;
    char latDir[4];
    strcpy(latDir, fieldBuffer);

    // Move to longitude (field 4)
    field = findNextField(field);
    if (!field)
        return false;

    if (!extractField(field, fieldBuffer, sizeof(fieldBuffer)) || isFieldEmpty(fieldBuffer))
        return false;
    char lonStr[32];
    strcpy(lonStr, fieldBuffer);

    // Move to longitude direction (field 5)
    field = findNextField(field);
    if (!field)
        return false;

    if (!extractField(field, fieldBuffer, sizeof(fieldBuffer)) || isFieldEmpty(fieldBuffer))
        return false;
    char lonDir[4];
    strcpy(lonDir, fieldBuffer);

    // Move to fix quality (field 6)
    field = findNextField(field);
    if (!field)
        return false;

    if (!extractField(field, fieldBuffer, sizeof(fieldBuffer)) || isFieldEmpty(fieldBuffer))
        return false;
    data.fixQuality = parseUInt32(fieldBuffer);

    // Move to satellite count (field 7)
    field = findNextField(field);
    if (!field)
        return false;

    if (!extractField(field, fieldBuffer, sizeof(fieldBuffer)) || isFieldEmpty(fieldBuffer))
        return false;
    data.satelliteCount = parseUInt32(fieldBuffer);

    // Move to HDOP (field 8)
    field = findNextField(field);
    if (!field)
        return false;

    if (!extractField(field, fieldBuffer, sizeof(fieldBuffer)) || isFieldEmpty(fieldBuffer))
        return false;
    data.hdop = parseFloat(fieldBuffer);

    // Move to altitude (field 9)
    field = findNextField(field);
    if (!field)
        return false;

    if (!extractField(field, fieldBuffer, sizeof(fieldBuffer)) || isFieldEmpty(fieldBuffer))
        return false;
    data.altitude = parseFloat(fieldBuffer);

    // Parse coordinates
    if (!parseCoordinate(latStr, latDir, data.latitude) ||
        !parseCoordinate(lonStr, lonDir, data.longitude))
    {
        return false;
    }

    data.validFix = (data.fixQuality > 0);
    return true;
}

bool GNSSProcessor::parseRMC(const char *sentence, PositionData &data)
{
    // $GPRMC,hhmmss.ss,A,ddmm.mmm,N,dddmm.mmm,E,s.s,h.h,ddmmyy,d.d,E,m*hh
    const char *field = findNextField(sentence); // Skip header
    if (!field)
        return false;

    // Time
    field = findNextField(field);
    if (!field)
        return false;

    // Status
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    bool active = (*field == 'A');

    // Latitude
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    const char *latStr = field;

    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    const char *latDir = field;

    // Longitude
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    const char *lonStr = field;

    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    const char *lonDir = field;

    // Speed (knots)
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    float speedKnots = parseFloat(field);
    data.speed = speedKnots * 1.852f; // Convert to km/h

    // Course
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    data.heading = parseFloat(field);

    // Parse coordinates
    if (!parseCoordinate(latStr, latDir, data.latitude) ||
        !parseCoordinate(lonStr, lonDir, data.longitude))
    {
        return false;
    }

    data.validFix = active;
    return true;
}

bool GNSSProcessor::parseVTG(const char *sentence, PositionData &data)
{
    // $GPVTG,h.h,T,h.h,M,s.s,N,s.s,K,m*hh
    const char *field = findNextField(sentence); // Skip header
    if (!field)
        return false;

    // True course
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    data.heading = parseFloat(field);

    // Skip T
    field = findNextField(field);
    if (!field)
        return false;

    // Magnetic course (skip)
    field = findNextField(field);
    if (!field)
        return false;

    // Skip M
    field = findNextField(field);
    if (!field)
        return false;

    // Speed in knots (skip)
    field = findNextField(field);
    if (!field)
        return false;

    // Skip N
    field = findNextField(field);
    if (!field)
        return false;

    // Speed in km/h
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    data.speed = parseFloat(field);

    return true;
}

bool GNSSProcessor::parseGSA(const char *sentence, PositionData &data)
{
    // $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
    // For now, just skip to HDOP field (not implementing full GSA parsing)
    return true;
}

bool GNSSProcessor::parseGSV(const char *sentence, PositionData &data)
{
    // $GPGSV,2,1,07,07,79,048,42,02,51,062,43,26,36,256,42,27,27,138,42*71
    // For now, just return true (not implementing full GSV parsing)
    return true;
}

bool GNSSProcessor::parseHPR(const char *sentence, PositionData &data)
{
    // $GNHPR,155812.60,291.4968,000.7811,000.0000,4,20,0.00,0999*58
    // HPR: Time, Heading, Pitch, Roll, Quality, Sats, HDOP, ???
    const char *field = findNextField(sentence); // Skip header
    if (!field)
        return false;

    // Time
    field = findNextField(field);
    if (!field)
        return false;

    // Heading
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    data.heading = parseFloat(field);

    // Pitch (skip for now)
    field = findNextField(field);
    if (!field)
        return false;

    // Roll (skip for now)
    field = findNextField(field);
    if (!field)
        return false;

    // Quality
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    data.fixQuality = parseUInt32(field);

    // Satellite count
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    data.satelliteCount = parseUInt32(field);

    // HDOP
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    data.hdop = parseFloat(field);

    if (debugEnabled_)
    {
        Serial.print("\r\n[HPR] Heading: ");
        Serial.print(data.heading, 2);
        Serial.print("° Fix: ");
        Serial.print(data.fixQuality);
        Serial.print(" Sats: ");
        Serial.print(data.satelliteCount);
    }

    return true;
}

bool GNSSProcessor::parseKSXT(const char *sentence, PositionData &data)
{
    // $KSXT,20190909084745.00,116.23662400,40.07897925,68.3830,299.22,-67.03,190.28,0.022,,1,3,46,28,,,,-0.004,-0.021,-0.020,,*27
    // KSXT: Timestamp, Longitude, Latitude, Height, Heading, Pitch, Roll, Speed, Reserved, Fix, Quality, Sats, ???

    const char *field = findNextField(sentence); // Skip $KSXT
    if (!field)
        return false;

    // Timestamp (skip for now)
    field = findNextField(field);
    if (!field)
        return false;

    // Longitude (already in decimal degrees)
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    data.longitude = parseDouble(field);

    // Latitude (already in decimal degrees)
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    data.latitude = parseDouble(field);

    // Height/Altitude
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    data.altitude = parseFloat(field);

    // Heading
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    data.heading = parseFloat(field);

    // Pitch (skip)
    field = findNextField(field);
    if (!field)
        return false;

    // Roll (skip)
    field = findNextField(field);
    if (!field)
        return false;

    // Speed (m/s)
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    float speedMS = parseFloat(field);
    data.speed = speedMS * 3.6f; // Convert m/s to km/h

    // Skip reserved field
    field = findNextField(field);
    if (!field)
        return false;

    // Fix type
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    data.fixQuality = parseUInt32(field);

    // Quality indicator
    field = findNextField(field);
    if (!field)
        return false;

    // Satellite count
    field = findNextField(field);
    if (!field || isFieldEmpty(field))
        return false;
    data.satelliteCount = parseUInt32(field);

    if (debugEnabled_)
    {
        Serial.print("\r\n[KSXT] Lat: ");
        Serial.print(data.latitude, 8);
        Serial.print("° Lon: ");
        Serial.print(data.longitude, 8);
        Serial.print("° Alt: ");
        Serial.print(data.altitude, 1);
        Serial.print("m");
        Serial.print(" Speed: ");
        Serial.print(data.speed, 2);
        Serial.print("km/h");
        Serial.print(" Heading: ");
        Serial.print(data.heading, 1);
        Serial.print("°");
    }

    // KSXT provides high-quality position data
    data.validFix = (data.fixQuality > 0);
    return true;
}

bool GNSSProcessor::validateNMEAChecksum(const char *sentence) const
{
    const char *checksumPos = strrchr(sentence, '*');
    if (!checksumPos)
        return false;

    uint8_t expectedChecksum = strtoul(checksumPos + 1, nullptr, 16);
    uint8_t calculatedChecksum = calculateNMEAChecksum(sentence);

    return expectedChecksum == calculatedChecksum;
}

uint8_t GNSSProcessor::calculateNMEAChecksum(const char *sentence) const
{
    uint8_t checksum = 0;
    const char *ptr = sentence + 1; // Skip '$'

    while (*ptr && *ptr != '*')
    {
        checksum ^= *ptr++;
    }

    return checksum;
}

void GNSSProcessor::extractTalkerAndMessage(const char *sentence, char *talkerID, char *messageID) const
{
    if (strlen(sentence) >= 6)
    {
        strncpy(talkerID, sentence + 1, 2);
        talkerID[2] = '\0';
        strncpy(messageID, sentence + 3, 3);
        messageID[3] = '\0';
    }
}

bool GNSSProcessor::parseCoordinate(const char *coord, const char *direction, double &result) const
{
    if (!coord || !direction || isFieldEmpty(coord) || isFieldEmpty(direction))
        return false;

    double coordinate = parseDouble(coord);
    if (coordinate == 0.0)
        return false;

    // Convert DDMM.MMMMMMM to decimal degrees
    // For latitude: DDMM.MMMMMMM where DD = degrees (2 digits)
    // For longitude: DDDMM.MMMMMMM where DDD = degrees (3 digits)

    int wholeDegrees;
    double minutes;

    // Determine if this is latitude (2 digit degrees) or longitude (3 digit degrees)
    if (coordinate >= 10000.0)
    {
        // Longitude: DDDMM.MMMMMMM (3 digit degrees)
        wholeDegrees = (int)(coordinate / 100.0);
        minutes = coordinate - (wholeDegrees * 100.0);
    }
    else
    {
        // Latitude: DDMM.MMMMMMM (2 digit degrees)
        wholeDegrees = (int)(coordinate / 100.0);
        minutes = coordinate - (wholeDegrees * 100.0);
    }

    result = wholeDegrees + (minutes / 60.0);

    // Apply direction
    if (*direction == 'S' || *direction == 'W')
    {
        result = -result;
    }

    if (debugEnabled_)
    {
        Serial.print("\r\n[COORD] Raw: ");
        Serial.print(coord);
        Serial.print(" Dir: ");
        Serial.print(direction);
        Serial.print(" -> ");
        Serial.print(result, 8);
        Serial.print("°");
    }

    return true;
}

const GNSSProcessor::PositionData &GNSSProcessor::getBestPosition() const
{
    updateBestPosition();
    return bestPosition_;
}

bool GNSSProcessor::hasBestPosition() const
{
    return getBestPosition().validFix;
}

void GNSSProcessor::updateBestPosition() const
{
    uint8_t gps1Score = 0;
    uint8_t gps2Score = 0;

    if (isGPS1Valid())
        gps1Score = calculatePositionScore(gps1Data_);
    if (isGPS2Valid())
        gps2Score = calculatePositionScore(gps2Data_);

    if (gps1Score >= gps2Score && gps1Score > 0)
    {
        bestPosition_ = gps1Data_;
    }
    else if (gps2Score > 0)
    {
        bestPosition_ = gps2Data_;
    }
    else
    {
        bestPosition_.validFix = false;
    }
}

uint8_t GNSSProcessor::calculatePositionScore(const PositionData &data) const
{
    if (!data.validFix)
        return 0;

    uint8_t score = 0;

    // Fix quality (0-20 points)
    score += (data.fixQuality * 5);
    if (score > 20)
        score = 20;

    // Satellite count (0-20 points)
    uint8_t satScore = data.satelliteCount * 2;
    if (satScore > 20)
        satScore = 20;
    score += satScore;

    // HDOP (0-20 points, lower is better)
    if (data.hdop < 1.0f)
        score += 20;
    else if (data.hdop < 2.0f)
        score += 15;
    else if (data.hdop < 3.0f)
        score += 10;
    else if (data.hdop < 5.0f)
        score += 5;

    // Age penalty (0-10 points deducted)
    uint32_t age = millis() - data.lastUpdateTime;
    if (age > 2000)
        score = (score > 10) ? score - 10 : 0;
    else if (age > 1000)
        score = (score > 5) ? score - 5 : 0;

    return score;
}

void GNSSProcessor::updatePositionQuality(PositionData &data)
{
    // Validate position is reasonable
    data.validFix = data.validFix && isPositionReasonable(data);
}

bool GNSSProcessor::isPositionReasonable(const PositionData &data) const
{
    // Check latitude range
    if (data.latitude < -90.0 || data.latitude > 90.0)
        return false;

    // Check longitude range
    if (data.longitude < -180.0 || data.longitude > 180.0)
        return false;

    // Check for null island (0,0) - but allow small values near zero
    if (fabs(data.latitude) < 0.01 && fabs(data.longitude) < 0.01)
        return false;

    return true;
}

void GNSSProcessor::sendUBXMessage(const uint8_t *message, size_t length, bool toGPS1, bool toGPS2)
{
    if (!message || length == 0)
        return;

    if (toGPS1)
    {
        serialManager_.writeGPS1(message, length);
    }

    if (toGPS2)
    {
        serialManager_.writeGPS2(message, length);
    }

    if (debugEnabled_)
    {
        printUBXDebug(message, length, "TX");
    }
}

void GNSSProcessor::resetStatistics()
{
    gps1SentencesProcessed_ = 0;
    gps2SentencesProcessed_ = 0;
    gps1ParseErrors_ = 0;
    gps2ParseErrors_ = 0;
}

// Utility functions
float GNSSProcessor::parseFloat(const char *str) const
{
    if (!str || isFieldEmpty(str))
        return 0.0f;
    return atof(str);
}

double GNSSProcessor::parseDouble(const char *str) const
{
    if (!str || isFieldEmpty(str))
        return 0.0;
    return atof(str);
}

uint32_t GNSSProcessor::parseUInt32(const char *str) const
{
    if (!str || isFieldEmpty(str))
        return 0;
    return strtoul(str, nullptr, 10);
}

const char *GNSSProcessor::findNextField(const char *str) const
{
    if (!str)
        return nullptr;

    const char *comma = strchr(str, ',');
    return comma ? (comma + 1) : nullptr;
}

// Helper function to extract a single field value
bool GNSSProcessor::extractField(const char *start, char *buffer, size_t bufferSize) const
{
    if (!start || !buffer)
        return false;

    const char *end = strchr(start, ',');
    if (!end)
    {
        // Last field - look for end of sentence
        end = strchr(start, '*');
        if (!end)
            end = strchr(start, '\r');
        if (!end)
            end = strchr(start, '\n');
        if (!end)
            end = start + strlen(start);
    }

    size_t fieldLen = end - start;
    if (fieldLen >= bufferSize)
        fieldLen = bufferSize - 1;

    strncpy(buffer, start, fieldLen);
    buffer[fieldLen] = '\0';

    return true;
}

bool GNSSProcessor::isFieldEmpty(const char *field) const
{
    return !field || *field == ',' || *field == '\0' || *field == '\r' || *field == '\n';
}

void GNSSProcessor::printNMEADebug(const char *sentence, const char *source) const
{
    Serial.print("\r\n[NMEA ");
    Serial.print(source);
    Serial.print("] ");
    Serial.print(sentence);
}

void GNSSProcessor::printPositionDebug(const PositionData &data, const char *source) const
{
    Serial.print("\r\n[POS ");
    Serial.print(source);
    Serial.print("] Lat:");
    Serial.print(data.latitude, 6);
    Serial.print(" Lon:");
    Serial.print(data.longitude, 6);
    Serial.print(" Fix:");
    Serial.print(data.fixQuality);
    Serial.print(" Sats:");
    Serial.print(data.satelliteCount);
    Serial.print(" HDOP:");
    Serial.print(data.hdop, 1);
    Serial.print(" Valid:");
    Serial.print(data.validFix ? "YES" : "NO");
}

void GNSSProcessor::printUBXDebug(const uint8_t *data, size_t length, const char *direction) const
{
    Serial.print("\r\n[UBX ");
    Serial.print(direction);
    Serial.print("] ");
    for (size_t i = 0; i < length && i < 32; i++)
    {
        if (data[i] < 16)
            Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    if (length > 32)
        Serial.print("...");
}