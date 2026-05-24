#ifndef BMP390_H
#define BMP390_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Sensor I2C Address
#define BMP390_I2C_ADDR         (0x77 << 1)

// Register Map
#define BMP390_REG_CALIB_DATA   0x31
#define BMP390_REG_CHIP_ID      0x00
#define BMP390_REG_DATA_0       0x04
#define BMP390_REG_INT_CTRL     0x19
#define BMP390_REG_ODR          0x1D
#define BMP390_REG_PWR_CTRL     0x1B

// Calibration Data Structure
typedef struct {
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6;
    float par_p7, par_p8, par_p9, par_p10, par_p11;
    float t_lin; // For pressure calculation
} BMP390_Calib_t;

// Driver Context Structure
typedef struct {
    I2C_HandleTypeDef *i2c_handle;

    uint32_t raw_pressure;
    uint32_t raw_temperature;

    BMP390_Calib_t calib;

    float temperature; // Final output in Celsius
    float pressure;    // Final output in Pascals
} BMP390_t;

// Function Prototypes
bool BMP390_Init(BMP390_t *sensor, I2C_HandleTypeDef *hi2c);
bool BMP390_ReadData(BMP390_t *sensor);

#endif // BMP390_H
