#ifndef MA730_H
#define MA730_H

#define MA730_SPI_DRIVER	        &hspi2
#define MA730_TIMEOUT_TICKS	        100

uint16_t ma730ReadAngle(void);

#endif //MA730_H