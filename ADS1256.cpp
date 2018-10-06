#include "ADS1256.h"

#include "Arduino.h"
#include "SPI.h"

// TODO: Magic number from another library
#define SPI_SPEED (1700000)


ADS1256::ADS1256(int cs, int drdy, int rst, int timeout)
{
    // Save pin numbers
    mPinCS = cs;
    mPinDRDY = drdy;
    mPinRST = rst;

    mTimeout = timeout;
}

#ifdef TI_ADS_ISR
void ADS1256::AttachISR(void)
{
    mNewData = false;

    void callISR(void)
    {
        this->EndOfConversionISR();
    }

    attachInterrupt(digitalPinToInterrupt(drdy), callISR, FALLING);
}
#endif // TI_ADS_ISR

ADS12xxStatus ADS1256::begin()
{
    // Setup default pin states
    digitalWrite(mPinCS, HIGH);
    digitalWrite(mPinRST, HIGH);

    // Configure pin modes
    pinMode(mPinCS, OUTPUT);
    pinMode(mPinDRDY, INPUT);
    pinMode(mPinRST, OUTPUT);

    // Setup SPI
    SPI.begin();

    // RESET to initiate a self-calibration
    SendCommand(ADS1256_CMD_RESET);

    // Wait until we've completed a conversion so we have data to read
    WaitForDRDY(-1);

    return ADS12xxStatus::SUCCESS;
}

void ADS1256::StartTransaction(void)
{
    // Start transaction and select chip
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
    digitalWrite(mPinCS, LOW);
}

void ADS1256::FinishTransaction(void)
{
    // Finish transaction and deselect chip
    digitalWrite(mPinCS, HIGH);
    SPI.endTransaction();
}

ADS12xxStatus ADS1256::WaitForDRDY(const int timeout)
{
#ifndef TI_ADS_ISR
    // No ISR, we need to poll
    int value = HIGH;
    unsigned long start = millis();

    // Wait until DRDY is LOW (or timed out if timeout != -1)
    do
    {
        value = digitalRead(mPinDRDY);
    }
    while ((value != LOW) &&
           ((timeout == -1) || ((long)(millis() - start) < timeout)));

    if (value == LOW)
    {
        return ADS12xxStatus::SUCCESS;
    }
#else // TI_ADS_ISR
    unsigned long start = millis();

    // Wait for ISR to signal new data
    do {
        continue;
    }
    while ((mNewData != true) &&
           ((timeout == -1) || ((long)(millis() - start) < timeout)));

    if (mNewData)
    {
        return ADS12xxStatus::SUCCESS;
    }
#endif // TI_ADS_ISR

    return ADS12xxStatus::TIMEOUT;
}

ADS12xxStatus ADS1256::SendCommand(const int cmd)
{
    StartTransaction();
    SPI.transfer(cmd);
    FinishTransaction();

    return ADS12xxStatus::SUCCESS;
}

ADS12xxStatus ADS1256::ReadRegisters(const int addr, uint8_t *dst, const size_t length)
{
    if (addr < 0 || addr > 0x0A)
    {
        return ADS12xxStatus::INVALID_PARAMETERS;
    }

    // Output the data from up to 11 registers... (pg 36)
    if (length < 1 || length > 11)
    {
        return ADS12xxStatus::INVALID_PARAMETERS;
    }

    StartTransaction();

    // Send command
    SPI.transfer(ADS1256_CMD_RREG | addr);
    SPI.transfer(length - 1);

    // Wait for data to be ready
    delayMicroseconds(10); // should be 6.51 us according to the datasheet

    // Read out data int uint8_t ptr dst
    for (size_t i = 0; i < length; i++)
    {
        uint8_t data = SPI.transfer(0);
        dst[i] = data;
    }

    FinishTransaction();

    return ADS12xxStatus::SUCCESS;
}

ADS12xxStatus ADS1256::ReadRegister(const int addr, uint8_t *dst)
{
    return ReadRegisters(addr, dst, 1);
}

ADS12xxStatus ADS1256::WriteRegisters(const int addr, const uint8_t *src, const size_t length)
{
    if (addr < 0 || addr > 0x0A)
    {
        return ADS12xxStatus::INVALID_PARAMETERS;
    }

    // We only have 11 registers
    if (length < 1 || length > 11)
    {
        return ADS12xxStatus::INVALID_PARAMETERS;
    }

    StartTransaction();

    // Send command
    SPI.transfer(ADS1256_CMD_WREG | addr);
    SPI.transfer(length - 1);

    // Write data from src ptr
    for (size_t i = 0; i < length; i++)
    {
        SPI.transfer(src[i]);
    }

    FinishTransaction();

    return ADS12xxStatus::SUCCESS;
}

ADS12xxStatus ADS1256::WriteRegister(const int addr, const uint8_t value)
{
    return WriteRegisters(addr, &value, 1);
}

ADS12xxStatus ADS1256::SetMux(const int pos_ain, const int neg_ain)
{
    if (neg_ain < ADS1256_MUX_AIN0 || neg_ain > ADS1256_MUX_AINCOM ||
        pos_ain < ADS1256_MUX_AIN0 || pos_ain > ADS1256_MUX_AINCOM)
    {
        return ADS12xxStatus::INVALID_PARAMETERS;
    }

    // 4 bits - pos ain, 4 bits - neg ain
    return WriteRegister(ADS1256_REG_MUX, (pos_ain << 4) | neg_ain);
}

ADS12xxStatus ADS1256::ReadData(int32_t *dst)
{
    // We need to wait until a conversion has finished
    ADS12xxStatus ret = WaitForDRDY(mTimeout);
    if (ret != ADS12xxStatus::SUCCESS)
    {
        return ret;
    }

    StartTransaction();

    // Send RDATA
    SPI.transfer(ADS1256_CMD_RDATA);

    // Wait for data to be ready
    delayMicroseconds(10); // should be 6.51 us according to the datasheet

    uint32_t raw_data = 0;

    // Shift out 24 bits
    raw_data |= SPI.transfer(0);
    raw_data <<= 8;
    raw_data |= SPI.transfer(0);
    raw_data <<= 8;
    raw_data |= SPI.transfer(0);

    // Check the if the twos compliment bit is set
    if (raw_data > 0x00800000)
    {
        // This is a negative number, fill in the higher bits with 1s
        raw_data |= 0xFF000000;
    }

    // Cast signed preserved number to a signed int 32
    *dst = (int32_t)raw_data;

    return ADS12xxStatus::SUCCESS;
}

ADS12xxStatus ADS1256::StartReadDataContinuous()
{
    return SendCommand(ADS1256_CMD_RDATAC);
}

ADS12xxStatus ADS1256::ShiftOutData(int32_t *dst)
{
    // We need to wait until a conversion has finished
    ADS12xxStatus ret = WaitForDRDY(mTimeout);
    if (ret != ADS12xxStatus::SUCCESS)
    {
        return ret;
    }

    StartTransaction();

    // Wait for data to be ready
    delayMicroseconds(10); // should be 6.51 us according to the datasheet

    uint32_t raw_data = 0;

    // Shift out 24 bits
    raw_data |= SPI.transfer(0);
    raw_data <<= 8;
    raw_data |= SPI.transfer(0);
    raw_data <<= 8;
    raw_data |= SPI.transfer(0);

    // Check the if the twos compliment bit is set
    if (raw_data > 0x00800000)
    {
        // This is a negative number, fill in the higher bits with 1s
        raw_data |= 0xFF000000;
    }

    // Cast signed preserved number to a signed int 32
    *dst = (int32_t)raw_data;

    return ADS12xxStatus::SUCCESS;
}

ADS12xxStatus ADS1256::EndReadDataContinuous()
{
    return SendCommand(ADS1256_CMD_SDATAC);
}

#ifdef TI_ADS_ISR
void WaitForNewData(void)
{
    const unsigned long curr_millis = mLastMillis;
    while (curr_millis == mLastMillis)
    {
        continue;
    }
}

int32_t ADS1256::GetLatestValue(void)
{
    return mLastValue;
}

void ADS1256::EndOfConversionISR(void)
{
    // EOC ISR
    // Shift out data from ADC
    int32_t value = 0;
    mNewData = true;
    this->ShiftOutData(&value);
    mNewData = false;

    // Update member values
    mLastMillis = millis();
    mLastValue = value;
}
#endif // TI_ADS_ISR
