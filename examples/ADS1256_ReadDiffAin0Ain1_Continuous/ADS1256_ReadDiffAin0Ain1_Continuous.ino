#include "ADS1256.h"

// Pin configuration for Arduino UNO
#define ADS1256_PIN_CS      (10)
#define ADS1256_PIN_DRDY    (9)
#define ADS1256_PIN_RST     (8)

#define ZERO_POINT (252000)
#define PER_LB (110000 / 8.3454)


ADS1256 adc = ADS1256(ADS1256_PIN_CS, ADS1256_PIN_DRDY, ADS1256_PIN_RST);

void setup()
{
    // Setup serial
    Serial.begin(115200);

    // Serial.println("Initializing...");

    // Setup pins and initialize the ADS1256 chip
    adc.begin();

    // Configure reading between AIN0 and AIN1
    adc.SetMux(ADS1256_MUX_AIN1, ADS1256_MUX_AIN0);
    adc.WriteRegister(ADS1256_REG_ADCON, 0b00100111); // Set PGA = 64
    adc.WriteRegister(ADS1256_REG_DRATE, 0b10100001); // DRATE = 1000 SPS

    // Serial.println("Complete!");

    adc.StartReadDataContinuous();
}

void loop()
{
    unsigned long time = millis();
    int32_t value = 0;
    adc.ShiftOutData(&value);

    float weight = (value - ZERO_POINT) / (float)PER_LB;

    Serial.print(time);
    Serial.print(" ");
    Serial.println(weight, 8);
    Serial.flush(); // Ensure the data has been transmitted
}
