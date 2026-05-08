/*
 * MPUXX50.c
 *
 *  Created on: Mar 25, 2026 (adapted)
 */

#include "MPUXX50_h7.h"

// Define globals (one-definition in .c)
struct RawData rawData;
struct SensorData sensorData;
struct GyroCal gyroCal;
struct Attitude attitude;

uint8_t _addr;
float _dt, _tau;
float aScaleFactor, gScaleFactor;

/// @brief Set the IMU address, check for connection, reset IMU, and set full range scale.
uint8_t MPU_begin(I2C_HandleTypeDef *I2Cx, uint8_t addr, uint8_t aScale, uint8_t gScale, float tau, float dt)
{
    // Save values
    _addr = addr << 1;
    _tau = tau;
    _dt = dt;

    // Initialize variables
    uint8_t check;
    uint8_t select;

    // Confirm device
    HAL_I2C_Mem_Read(I2Cx, _addr, WHO_AM_I, 1, &check, 1, I2C_TIMOUT_MS);

    if ((check == WHO_AM_I_9250_ANS) || (check == WHO_AM_I_6050_ANS))
    {
        // Startup / reset the sensor
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS);

        // Set the full scale ranges
        MPU_writeAccFullScaleRange(I2Cx, aScale);
        MPU_writeGyroFullScaleRange(I2Cx, gScale);

        return 1;
    }
    else
    {
        return 0;
    }
}

void MPU_writeAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale)
{
    uint8_t select;
    switch (aScale)
    {
    case AFSR_2G:
        aScaleFactor = 16384.0f;
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_4G:
        aScaleFactor = 8192.0f;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_8G:
        aScaleFactor = 4096.0f;
        select = 0x10;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_16G:
        aScaleFactor = 2048.0f;
        select = 0x18;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        aScaleFactor = 8192.0f;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

void MPU_writeGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale)
{
    uint8_t select;
    switch (gScale)
    {
    case GFSR_250DPS:
        gScaleFactor = 131.0f;
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_500DPS:
        gScaleFactor = 65.5f;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_1000DPS:
        gScaleFactor = 32.8f;
        select = 0x10;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_2000DPS:
        gScaleFactor = 16.4f;
        select = 0x18;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        gScaleFactor = 65.5f;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

void MPU_readRawData(I2C_HandleTypeDef *I2Cx)
{
    uint8_t buf[14];
    HAL_I2C_Mem_Read(I2Cx, _addr, ACCEL_XOUT_H, 1, buf, 14, I2C_TIMOUT_MS);
    rawData.ax = (int16_t)(buf[0] << 8 | buf[1]);
    rawData.ay = (int16_t)(buf[2] << 8 | buf[3]);
    rawData.az = (int16_t)(buf[4] << 8 | buf[5]);
    rawData.gx = (int16_t)(buf[8] << 8 | buf[9]);
    rawData.gy = (int16_t)(buf[10] << 8 | buf[11]);
    rawData.gz = (int16_t)(buf[12] << 8 | buf[13]);
}

void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints)
{
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;
    int64_t ax_sum = 0;
    int64_t ay_sum = 0;
    int64_t az_sum = 0;

    if (numCalPoints == 0)
    {
        numCalPoints = 1;
    }

    // Discard a small number of initial samples to let sensor settle
    const uint16_t discard = 50;
    for (uint16_t d = 0; d < discard; d++) {
        MPU_readRawData(I2Cx);
        HAL_Delay(2);
    }

    for (uint16_t ii = 0; ii < numCalPoints; ii++)
    {
        MPU_readRawData(I2Cx);
        x += rawData.gx;
        y += rawData.gy;
        z += rawData.gz;
        ax_sum += rawData.ax;
        ay_sum += rawData.ay;
        az_sum += rawData.az;
        HAL_Delay(3);
    }

    gyroCal.x = (float)x / (float)numCalPoints;
    gyroCal.y = (float)y / (float)numCalPoints;
    gyroCal.z = (float)z / (float)numCalPoints;

    // Seed attitude roll and pitch from average accelerometer reading to avoid zero-start jumps
    float ax_avg = (float)ax_sum / (float)numCalPoints;
    float ay_avg = (float)ay_sum / (float)numCalPoints;
    float az_avg = (float)az_sum / (float)numCalPoints;

    float accelPitch = atan2f(ay_avg / aScaleFactor, az_avg / aScaleFactor) * RAD2DEG;
    float accelRoll = atan2f(ax_avg / aScaleFactor, az_avg / aScaleFactor) * RAD2DEG;

    attitude.r = accelRoll;
    attitude.p = accelPitch;
    attitude.y = 0.0f; // start yaw at zero; will integrate gyro from here
}

void MPU_readProcessedData(I2C_HandleTypeDef *I2Cx)
{
    MPU_readRawData(I2Cx);
    sensorData.ax = rawData.ax / aScaleFactor;
    sensorData.ay = rawData.ay / aScaleFactor;
    sensorData.az = rawData.az / aScaleFactor;
    sensorData.gx = rawData.gx - gyroCal.x;
    sensorData.gy = rawData.gy - gyroCal.y;
    sensorData.gz = rawData.gz - gyroCal.z;
    sensorData.gx /= gScaleFactor;
    sensorData.gy /= gScaleFactor;
    sensorData.gz /= gScaleFactor;
}

void MPU_calcAttitude(I2C_HandleTypeDef *I2Cx)
{
    MPU_readProcessedData(I2Cx);
    float accelPitch = atan2f(sensorData.ay, sensorData.az) * RAD2DEG;
    float accelRoll = atan2f(sensorData.ax, sensorData.az) * RAD2DEG;
    attitude.r = _tau * (attitude.r - sensorData.gy * _dt) + (1 - _tau) * accelRoll;
    attitude.p = _tau * (attitude.p + sensorData.gx * _dt) + (1 - _tau) * accelPitch;
    attitude.y += sensorData.gz * _dt;
}

float MPU_getRoll(void)
{
    return attitude.r;
}

float MPU_getPitch(void)
{
    return attitude.p;
}

float MPU_getYaw(void)
{
    return attitude.y;
}