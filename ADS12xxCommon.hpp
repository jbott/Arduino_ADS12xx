#ifndef COMMON_H
#define COMMON_H

enum ADS12xxStatus
{
    SUCCESS             = 0x00,
    INVALID_PARAMETERS,
    TIMEOUT,
    UNKNOWN_FAILURE     = 0xFF,
};

#endif //COMMON_H
