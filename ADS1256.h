#ifndef ADS1256_H
#define ADS1256_H

#include "stdint.h"
#include "stdlib.h"

#include "ADS12xxCommon.hpp"

// Define registers
#define ADS1256_REG_STATUS  (0x00)
#define ADS1256_REG_MUX     (0x01)
#define ADS1256_REG_ADCON   (0x02)
#define ADS1256_REG_DRATE   (0x03)
#define ADS1256_REG_IO      (0x04)
#define ADS1256_REG_OFC0    (0x05)
#define ADS1256_REG_OFC1    (0x06)
#define ADS1256_REG_OFC2    (0x07)
#define ADS1256_REG_FSC0    (0x08)
#define ADS1256_REG_FSC1    (0x09)
#define ADS1256_REG_FSC2    (0x0A)

// Mux values
#define ADS1256_MUX_AIN0    (0x0)
#define ADS1256_MUX_AIN1    (0x1)
#define ADS1256_MUX_AIN2    (0x2)
#define ADS1256_MUX_AIN3    (0x3)
#define ADS1256_MUX_AIN4    (0x4)
#define ADS1256_MUX_AIN5    (0x5)
#define ADS1256_MUX_AIN6    (0x6)
#define ADS1256_MUX_AIN7    (0x7)
#define ADS1256_MUX_AINCOM  (0x8)

// Define command enums
#define ADS1256_CMD_WAKEUP      (0x00)  // Completes SYNC and Exits Standby Mode
#define ADS1256_CMD_RDATA       (0x01)  // Read Data
#define ADS1256_CMD_RDATAC      (0x03)  // Read Data Continuously
#define ADS1256_CMD_SDATAC      (0x0F)  // Stop Read Data Continuously
#define ADS1256_CMD_RREG        (0x10)  // Reads N registers, in form [ ADS1256_CMD_RREG | (addr), (length) ]
#define ADS1256_CMD_WREG        (0x50)  // Writes N registers, in form [ ADS1256_CMD_WREG | (addr), (length) ]
#define ADS1256_CMD_SELFCAL     (0xF0)  // Offset and Gain Self-Calibration
#define ADS1256_CMD_SELFOCAL    (0xF1)  // Offset Self-Calibration
#define ADS1256_CMD_SELFGCAL    (0xF2)  // Gain Self-Calibration
#define ADS1256_CMD_SYSOCAL     (0xF3)  // System Offset Calibration
#define ADS1256_CMD_SYSGCAL     (0xF4)  // System Gain Calibration
#define ADS1256_CMD_SYNC        (0xFC)  // Synchronize the A/D Conversion
#define ADS1256_CMD_STANDBY     (0xFD)  // Begin Standby Mode
#define ADS1256_CMD_RESET       (0xFE)  // Reset to Power-Up Values
#define ADS1256_CMD_WAKEUP      (0xFF)  // Completes SYNC and Exits Standby Mode


class ADS1256 {
    public:
        ADS1256(int cs, int drdy, int rst, int timeout = -1);

        ADS12xxStatus begin();

        // Low Level API
        ADS12xxStatus SendCommand(const int cmd);
        ADS12xxStatus ReadRegisters(const int addr, uint8_t *dst, const size_t length);
        ADS12xxStatus ReadRegister(const int addr, uint8_t *dst);
        ADS12xxStatus WriteRegisters(const int addr, const uint8_t *src, const size_t length);
        ADS12xxStatus WriteRegister(const int addr, uint8_t value);

        // High Level API
        ADS12xxStatus SetMux(const int pos_ain, const int neg_ain = ADS1256_MUX_AINCOM);

        ADS12xxStatus ReadData(int32_t *dst);

        ADS12xxStatus StartReadDataContinuous();
        ADS12xxStatus ShiftOutData(int32_t *dst);
        ADS12xxStatus EndReadDataContinuous();

    private:
        int mPinCS;
        int mPinDRDY;
        int mPinRST;

        int mTimeout;

        ADS12xxStatus WaitForDRDY(const int timeout);

        void StartTransaction(void);
        void FinishTransaction(void);
};

#endif // ADS1256_H
