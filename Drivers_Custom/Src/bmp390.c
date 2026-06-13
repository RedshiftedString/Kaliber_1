#include "bmp390.h"

// Helper function to write a single register
static bool BMP390_WriteReg(BMP390_t *sensor, uint8_t reg, uint8_t value) {
    return (HAL_I2C_Mem_Write(sensor->i2c_handle, BMP390_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100) == HAL_OK);
}

// Reads the 21 bytes of NVM factory calibration data and converts them to floats
static bool BMP390_GetCalibrationData(BMP390_t *sensor) {
    uint8_t calib[21];

    // Read 21 bytes starting at 0x31
    if (HAL_I2C_Mem_Read(sensor->i2c_handle, BMP390_I2C_ADDR, BMP390_REG_CALIB_DATA, I2C_MEMADD_SIZE_8BIT, calib, 21, 100) != HAL_OK) {
        return false;
    }

    // Parse bytes and cast to correct signed/unsigned integer types per Table 24
    uint16_t nvm_t1  = (calib[1] << 8) | calib[0];
    uint16_t nvm_t2  = (calib[3] << 8) | calib[2];
    int8_t   nvm_t3  = (int8_t)calib[4];
    int16_t  nvm_p1  = (int16_t)((calib[6] << 8) | calib[5]);
    int16_t  nvm_p2  = (int16_t)((calib[8] << 8) | calib[7]);
    int8_t   nvm_p3  = (int8_t)calib[9];
    int8_t   nvm_p4  = (int8_t)calib[10];
    uint16_t nvm_p5  = (calib[12] << 8) | calib[11];
    uint16_t nvm_p6  = (calib[14] << 8) | calib[13];
    int8_t   nvm_p7  = (int8_t)calib[15];
    int8_t   nvm_p8  = (int8_t)calib[16];
    int16_t  nvm_p9  = (int16_t)((calib[18] << 8) | calib[17]);
    int8_t   nvm_p10 = (int8_t)calib[19];
    int8_t   nvm_p11 = (int8_t)calib[20];

    // Scale logic based on Appendix 8.4 formulas
    sensor->calib.par_t1  = (float)nvm_t1 * 256.0f;                  // / 2^-8
    sensor->calib.par_t2  = (float)nvm_t2 / 1073741824.0f;           // / 2^30
    sensor->calib.par_t3  = (float)nvm_t3 / 281474976710656.0f;      // / 2^48
    sensor->calib.par_p1  = ((float)nvm_p1 - 16384.0f) / 1048576.0f; // / 2^20
    sensor->calib.par_p2  = ((float)nvm_p2 - 16384.0f) / 536870912.0f;// / 2^29
    sensor->calib.par_p3  = (float)nvm_p3 / 4294967296.0f;           // / 2^32
    sensor->calib.par_p4  = (float)nvm_p4 / 137438953472.0f;         // / 2^37
    sensor->calib.par_p5  = (float)nvm_p5 * 8.0f;                    // / 2^-3
    sensor->calib.par_p6  = (float)nvm_p6 / 64.0f;                   // / 2^6
    sensor->calib.par_p7  = (float)nvm_p7 / 256.0f;                  // / 2^8
    sensor->calib.par_p8  = (float)nvm_p8 / 32768.0f;                // / 2^15
    sensor->calib.par_p9  = (float)nvm_p9 / 281474976710656.0f;      // / 2^48
    sensor->calib.par_p10 = (float)nvm_p10 / 281474976710656.0f;     // / 2^48
    sensor->calib.par_p11 = (float)nvm_p11 / 36893488147419103232.0f;// / 2^65

    return true;
}

// Bosch Compensation Formula for Temperature
static float BMP390_CompensateTemp(BMP390_t *sensor) {
    float partial_data1 = (float)(sensor->raw_temperature) - sensor->calib.par_t1;
    float partial_data2 = partial_data1 * sensor->calib.par_t2;

    // Store t_lin in struct; it is required for pressure calculation
    sensor->calib.t_lin = partial_data2 + (partial_data1 * partial_data1) * sensor->calib.par_t3;

    return sensor->calib.t_lin;
}

// Bosch Compensation Formula for Pressure
static float BMP390_CompensatePress(BMP390_t *sensor) {
    float partial_data1, partial_data2, partial_data3, partial_data4;
    float partial_out1, partial_out2;

    float t_lin = sensor->calib.t_lin;
    float uncomp_press = (float)(sensor->raw_pressure);

    partial_data1 = sensor->calib.par_p6 * t_lin;
    partial_data2 = sensor->calib.par_p7 * (t_lin * t_lin);
    partial_data3 = sensor->calib.par_p8 * (t_lin * t_lin * t_lin);
    partial_out1  = sensor->calib.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = sensor->calib.par_p2 * t_lin;
    partial_data2 = sensor->calib.par_p3 * (t_lin * t_lin);
    partial_data3 = sensor->calib.par_p4 * (t_lin * t_lin * t_lin);
    partial_out2  = uncomp_press * (sensor->calib.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = uncomp_press * uncomp_press;
    partial_data2 = sensor->calib.par_p9 + sensor->calib.par_p10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + (uncomp_press * uncomp_press * uncomp_press) * sensor->calib.par_p11;

    return partial_out1 + partial_out2 + partial_data4;
}

bool BMP390_Init(BMP390_t *sensor, I2C_HandleTypeDef *hi2c) {
    sensor->i2c_handle = hi2c;
    sensor->raw_pressure = 0;
    sensor->raw_temperature = 0;

    // 1. Check if device is ready
    if (HAL_I2C_IsDeviceReady(sensor->i2c_handle, BMP390_I2C_ADDR, 3, 100) != HAL_OK) return false;

    // 2. Fetch Factory NVM Coefficients
    if (!BMP390_GetCalibrationData(sensor)) return false;

    // 3. Configure Interrupts (0x19 = 0x42)
    if (!BMP390_WriteReg(sensor, BMP390_REG_INT_CTRL, 0x42)) return false;

    // 4. Configure IIR filter coefficient (0x1F = 0x04)
    if (!BMP390_WriteReg(sensor, BMP390_REG_CONFIG, 0x04)) return false;

    // 5. Configure Oversampling: Press x8; Temp x1 (0x1C = 0x03)
    if (!BMP390_WriteReg(sensor, BMP390_REG_OSR, 0x03)) return false;

    // 6. Configure Output Data Rate (0x1D = 0x00..0x11) (ODR = 200 Hz / 2 ^ odr_sel)
    if (!BMP390_WriteReg(sensor, BMP390_REG_ODR, 0x02)) return false;

    // 7. Configure Power and Mode (0x1B = 0x33)
    if (!BMP390_WriteReg(sensor, BMP390_REG_PWR_CTRL, 0x33)) return false;

    return true;
}

bool BMP390_ReadData(BMP390_t *sensor) {
    uint8_t rx_buf[6] = {0};

    // Burst read 6 bytes starting from 0x04
    if (HAL_I2C_Mem_Read(sensor->i2c_handle, BMP390_I2C_ADDR, BMP390_REG_DATA_0, I2C_MEMADD_SIZE_8BIT, rx_buf, 6, 100) != HAL_OK) {
        return false;
    }

    // Reconstruct 24-bit raw pressure; temperature (0x06, 0x05, 0x04; 0x09, 0x08, 0x07)
    sensor->raw_pressure = (rx_buf[2] << 16) | (rx_buf[1] << 8) | rx_buf[0];
    sensor->raw_temperature = (rx_buf[5] << 16) | (rx_buf[4] << 8) | rx_buf[3];

    // Calculate physical values (temperature must be calculated first)
    sensor->temperature = BMP390_CompensateTemp(sensor);
    sensor->pressure	= BMP390_CompensatePress(sensor);

    return true;
}
