#ifndef INC_COMPAT_HELPERS_H_
#define INC_COMPAT_HELPERS_H_

#include "stm32h7xx_hal.h"

/* Debug/utility print functions used from main.c */
void print_tim3_regs(const char *tag);
void print_servo_pulse_us(const char *tag);

/* Minimal BMP280 compatibility stubs (return non-zero if not implemented) */
int bmp280_read_calib(I2C_HandleTypeDef *hi2c, uint8_t addr);
int bmp280_read_measurement(I2C_HandleTypeDef *hi2c, uint8_t addr, float *tempC, float *press_hPa);

#endif /* INC_COMPAT_HELPERS_H_ */
