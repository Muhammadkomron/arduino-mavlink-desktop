/*
 * MPUXX50_h7.h
 * Unique header name for STM32H7 devebox project to avoid include collisions
 */

#ifndef MPUXX50_H7_H_
#define MPUXX50_H7_H_

#include <stdint.h>
#include <math.h>
#include "main.h" // HAL types

#define RAD2DEG 57.2957795131f

#define WHO_AM_I_6050_ANS 0x68
#define WHO_AM_I_9250_ANS 0x71
#define WHO_AM_I          0x75
#define AD0_LOW           0x68
#define AD0_HIGH          0x69
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define PWR_MGMT_1        0x6B
#define ACCEL_XOUT_H      0x3B
#define I2C_TIMOUT_MS     1000

enum gyroscopeFullScaleRange { GFSR_250DPS, GFSR_500DPS, GFSR_1000DPS, GFSR_2000DPS };
enum accelerometerFullScaleRange { AFSR_2G, AFSR_4G, AFSR_8G, AFSR_16G };

/* Ensure macros for older code compatibility */
#ifndef AFSR_2G
#define AFSR_2G 0
#define AFSR_4G 1
#define AFSR_8G 2
#define AFSR_16G 3
#endif

#ifndef GFSR_250DPS
#define GFSR_250DPS 0
#define GFSR_500DPS 1
#define GFSR_1000DPS 2
#define GFSR_2000DPS 3
#endif

struct RawData { int16_t ax, ay, az, gx, gy, gz; };
struct SensorData { float ax, ay, az, gx, gy, gz; };
struct GyroCal { float x, y, z; };
struct Attitude { float r, p, y; };

extern struct RawData rawData;
extern struct SensorData sensorData;
extern struct GyroCal gyroCal;
extern struct Attitude attitude;

extern uint8_t _addr;
extern float _dt, _tau;
extern float aScaleFactor, gScaleFactor;

uint8_t MPU_begin(I2C_HandleTypeDef *I2Cx, uint8_t addr, uint8_t aScale, uint8_t gScale, float tau, float dt);
void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints);
void MPU_calcAttitude(I2C_HandleTypeDef *I2Cx);
void MPU_readRawData(I2C_HandleTypeDef *I2Cx);
void MPU_readProcessedData(I2C_HandleTypeDef *I2Cx);
void MPU_writeGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale);
void MPU_writeAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale);

float MPU_getRoll(void);
float MPU_getPitch(void);
float MPU_getYaw(void);

#endif /* MPUXX50_H7_H_ */
