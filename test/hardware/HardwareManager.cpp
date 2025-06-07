#include "HardwareManager.h"
#include "Wire.h"

HardwareManager::HardwareManager()
    : serialInitialized_(false), pinsInitialized_(false), pwmInitialized_(false), adcInitialized_(false)
{
}

HardwareManager::~HardwareManager()
{
}

void HardwareManager::begin()
{
    Serial.print("\r\n- Hardware Manager initialization");

    setupPins();
    setupSerial();
    setupPWM();
    setupADC();

    Serial.print("\r\n- Hardware Manager initialization complete");
}

void HardwareManager::setupPins()
{
    if (pinsInitialized_)
        return;

    Serial.print("\r\n  - Setting up pins");
    setupPinModes();
    pinsInitialized_ = true;
}

void HardwareManager::setupPinModes()
{
    // Buzzer control
    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);

    // Motor driver control
    pinMode(SLEEP_PIN, OUTPUT);
    digitalWrite(SLEEP_PIN, LOW);

    // Input pins with appropriate pull resistors
    pinMode(STEER_PIN, INPUT_PULLUP);
    pinMode(KICKOUT_D_PIN, INPUT_PULLUP);

    // Analog input pins - disable internal resistors
    pinMode(WORK_PIN, INPUT_DISABLE);
    pinMode(CURRENT_PIN, INPUT_DISABLE);
    pinMode(KICKOUT_A_PIN, INPUT_DISABLE);
    pinMode(WAS_SENSOR_PIN, INPUT_DISABLE);

    // Speed pulse outputs
    pinMode(SPEEDPULSE_PIN, OUTPUT);
    pinMode(SPEEDPULSE10_PIN, OUTPUT);
    digitalWrite(SPEEDPULSE_PIN, LOW);
    digitalWrite(SPEEDPULSE10_PIN, LOW);
}

void HardwareManager::setupSerial()
{
    if (serialInitialized_)
        return;

    Serial.print("\r\n  - Setting up serial ports");

    // Setup GPS1 serial
    Serial5.begin(BAUD_GPS);
    Serial5.addMemoryForRead(gps1RxBuffer_, GPS1_RX_BUFFER_SIZE);
    Serial5.addMemoryForWrite(gps1TxBuffer_, GPS1_TX_BUFFER_SIZE);

    // Setup GPS2 serial
    Serial8.begin(BAUD_GPS);
    Serial8.addMemoryForRead(gps2RxBuffer_, GPS2_RX_BUFFER_SIZE);
    Serial8.addMemoryForWrite(gps2TxBuffer_, GPS2_TX_BUFFER_SIZE);

    // Setup RTK radio serial
    Serial3.begin(BAUD_RTK);
    Serial3.addMemoryForRead(rtkRxBuffer_, RTK_RX_BUFFER_SIZE);

    // Setup RS232 serial
    Serial7.begin(BAUD_RS232);
    Serial7.addMemoryForWrite(rs232TxBuffer_, RS232_TX_BUFFER_SIZE);

    // Setup ESP32 serial
    Serial2.begin(BAUD_ESP32);
    Serial2.addMemoryForRead(esp32RxBuffer_, ESP32_RX_BUFFER_SIZE);
    Serial2.addMemoryForWrite(esp32TxBuffer_, ESP32_TX_BUFFER_SIZE);

    // Setup IMU serial
    Serial4.begin(115200); // BNO RVC mode default baud

    serialInitialized_ = true;
}

void HardwareManager::setupPWM()
{
    if (pwmInitialized_)
        return;

    Serial.print("\r\n  - Setting up PWM frequencies");
    setPWMFrequency(PWM1_PIN, PWM2_PIN, PWM_FREQUENCY);
    pwmInitialized_ = true;
}

void HardwareManager::setPWMFrequency(uint8_t pin1, uint8_t pin2, uint8_t frequency)
{
    uint16_t freq = getPWMFrequencyValue(frequency);
    analogWriteFrequency(pin1, freq);
    analogWriteFrequency(pin2, freq);

    Serial.print("\r\n    - PWM frequency set to ");
    Serial.print(freq);
    Serial.print("Hz");
}

uint16_t HardwareManager::getPWMFrequencyValue(uint8_t frequency) const
{
    switch (frequency)
    {
    case 0:
        return 490;
    case 1:
        return 122;
    case 2:
        return 3921;
    case 3:
        return 9155;
    case 4:
        return 18310;
    default:
        return 490;
    }
}

void HardwareManager::setupADC()
{
    if (adcInitialized_)
        return;

    Serial.print("\r\n  - Setting up ADC");
    analogReadResolution(12);
    analogReadAveraging(16);
    adcInitialized_ = true;
}

void HardwareManager::setCpuFrequency(uint32_t frequency)
{
    set_arm_clock(frequency);
    Serial.print("\r\n  - CPU frequency set to ");
    Serial.print(F_CPU_ACTUAL / 1000000);
    Serial.print("MHz");
}

bool HardwareManager::isSerialPortReady(HardwareSerial *port) const
{
    return port && serialInitialized_;
}

bool HardwareManager::isI2CDevicePresent(uint8_t address) const
{
    Wire.beginTransmission(address);
    return (Wire.endTransmission() == 0);
}