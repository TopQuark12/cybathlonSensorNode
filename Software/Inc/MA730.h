#ifndef MA730_H
#define MA730_H

#define MA730_SPI_DRIVER	        &hspi2
#define MA730_TIMEOUT_TICKS	        100

typedef enum
{
    ZERO_LOW = 0x00,
    ZERO_HIGH,
    BCT,
    EN_TRIM,
    ABZ_CFG,
    PPT_CFG,
    MG_THRS,
    R_DIR = 0x09,
    MG_STATUS = 0x1B,
} ma730Reg;

uint16_t ma730ReadAngle(void);

#endif //MA730_H