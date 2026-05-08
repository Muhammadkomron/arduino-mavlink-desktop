#include "compat_helpers.h"
#include "SERVO.h"
#include "SERVO_cfg.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

extern UART_HandleTypeDef huart4;

// Local storage for BMP/BME calibration parameters (copied from BME280 driver types)
static uint16_t calib_dig_T1;
static int16_t  calib_dig_T2, calib_dig_T3;
static uint16_t calib_dig_P1;
static int16_t  calib_dig_P2, calib_dig_P3, calib_dig_P4, calib_dig_P5;
static int16_t  calib_dig_P6, calib_dig_P7, calib_dig_P8, calib_dig_P9;
static uint8_t  calib_dig_H1;
static int16_t  calib_dig_H2, calib_dig_H3, calib_dig_H4, calib_dig_H5;
static int8_t   calib_dig_H6;
static int32_t  calib_t_fine_local;

void print_tim3_regs(const char *tag)
{
    char buf[512];
    uint32_t psc = TIM3->PSC;
    uint32_t arr = TIM3->ARR;
    uint32_t cnt = TIM3->CNT;
    uint32_t ccr1 = TIM3->CCR1;
    uint32_t ccr2 = TIM3->CCR2;
    uint32_t ccr3 = TIM3->CCR3;
    uint32_t ccr4 = TIM3->CCR4;

    uint32_t cr1 = TIM3->CR1;
    uint32_t ccmr1 = TIM3->CCMR1;
    uint32_t ccmr2 = TIM3->CCMR2;
    uint32_t ccer = TIM3->CCER;

    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint64_t tclk_pclk = (uint64_t)pclk1;
    uint64_t tclk_pclk2 = (uint64_t)pclk1 * 2ULL; // common case when APB prescaler != 1

    double us_per_tick_pclk = ((double)(psc + 1)) / (double)tclk_pclk * 1e6;
    double us_per_tick_pclk2 = ((double)(psc + 1)) / (double)tclk_pclk2 * 1e6;

    // Read GPIOC PC6 config
    uint32_t moder = GPIOC->MODER; // 2 bits per pin
    uint32_t afr = GPIOC->AFR[0]; // AFRL for pins 0..7
    uint32_t pupdr = GPIOC->PUPDR;

    int n = snprintf(buf, sizeof(buf),
        "[%s] TIM3: CR1=0x%08lX PSC=%lu ARR=%lu CNT=%lu\r\n"
        "      CCMR1=0x%08lX CCMR2=0x%08lX CCER=0x%08lX\r\n"
        "      CCR1=%lu CCR2=%lu CCR3=%lu CCR4=%lu\r\n"
        "      PCLK1=%lu Hz, assumed TIMclk=PCLK1 -> us/tick=%.3f us, CCR1=%.1f us\r\n"
        "      assumed TIMclk=2*PCLK1 -> us/tick=%.3f us, CCR1=%.1f us\r\n"
        "      GPIOC MODER=0x%08lX AFRL=0x%08lX PUPDR=0x%08lX (PC6 uses AFRL[6])\r\n",
        tag ? tag : "",
        (unsigned long)cr1, (unsigned long)psc, (unsigned long)arr, (unsigned long)cnt,
        (unsigned long)ccmr1, (unsigned long)ccmr2, (unsigned long)ccer,
        (unsigned long)ccr1, (unsigned long)ccr2, (unsigned long)ccr3, (unsigned long)ccr4,
        (unsigned long)pclk1,
        us_per_tick_pclk, (double)ccr1 * us_per_tick_pclk,
        us_per_tick_pclk2, (double)ccr1 * us_per_tick_pclk2,
        (unsigned long)moder, (unsigned long)afr, (unsigned long)pupdr);

    if (n > 0) HAL_UART_Transmit(&huart4, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
}

void print_servo_pulse_us(const char *tag)
{
    char buf[512];
    int len = 0;
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint64_t tclk_pclk = (uint64_t)pclk1;
    uint64_t tclk_pclk2 = (uint64_t)pclk1 * 2ULL;

    len += snprintf(buf + len, sizeof(buf) - len, "[%s] Servo pulses (us):\r\n", tag ? tag : "");

    for (int i = 0; i < SERVO_NUM && len < (int)sizeof(buf) - 160; ++i)
    {
        uint32_t ccr = 0;
        uint32_t *pccr = SERVO_CfgParam[i].TIM_CCRx;
        if (pccr) ccr = *pccr;

        // determine PSC and CCER based on which timer this servo uses
        TIM_TypeDef *tim = SERVO_CfgParam[i].TIM_Instance;
        uint32_t psc = 0;
        uint32_t ccer = 0;
        if (tim == TIM3)
        {
            psc = TIM3->PSC;
            ccer = TIM3->CCER;
        }
        else if (tim == TIM4)
        {
            psc = TIM4->PSC;
            ccer = TIM4->CCER;
        }
        else
        {
            psc = TIM3->PSC; // fallback
            ccer = TIM3->CCER;
        }

        double us_per_tick_pclk = ((double)(psc + 1)) / (double)tclk_pclk * 1e6;
        double us_per_tick_pclk2 = ((double)(psc + 1)) / (double)tclk_pclk2 * 1e6;

        int enabled = 0;
        uint32_t ch = SERVO_CfgParam[i].PWM_TIM_CH;
        if (ch == TIM_CHANNEL_1) enabled = (ccer & TIM_CCER_CC1E) ? 1 : 0;
        else if (ch == TIM_CHANNEL_2) enabled = (ccer & TIM_CCER_CC2E) ? 1 : 0;
        else if (ch == TIM_CHANNEL_3) enabled = (ccer & TIM_CCER_CC3E) ? 1 : 0;
        else if (ch == TIM_CHANNEL_4) enabled = (ccer & TIM_CCER_CC4E) ? 1 : 0;

        len += snprintf(buf + len, sizeof(buf) - len,
            "  Servo %d: CCR=%lu -> %.1f us (if TIMclk=PCLK1), %.1f us (if TIMclk=2*PCLK1) - output %s\r\n",
            i, (unsigned long)ccr, (double)ccr * us_per_tick_pclk, (double)ccr * us_per_tick_pclk2,
            enabled ? "ENABLED" : "DISABLED");
    }

    if (len > 0) HAL_UART_Transmit(&huart4, (uint8_t*)buf, (uint16_t)len, HAL_MAX_DELAY);
}

/* ---------------- BMP280 compatibility implementation (local) ---------------- */

// Compensation functions adapted from BME280 driver (uses local calib_* vars)
static int32_t comp_T_int32(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((int32_t)calib_dig_T1<<1))) * ((int32_t)calib_dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)calib_dig_T1)) * ((adc_T>>4) - ((int32_t)calib_dig_T1)))>> 12) *((int32_t)calib_dig_T3)) >> 14;
    calib_t_fine_local = var1 + var2;
    T = (calib_t_fine_local * 5 + 128) >> 8;
    return T;
}

static uint32_t comp_P_int64(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)calib_t_fine_local) - 128000;
    var2 = var1 * var1 * (int64_t)calib_dig_P6;
    var2 = var2 + ((var1*(int64_t)calib_dig_P5)<<17);
    var2 = var2 + (((int64_t)calib_dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)calib_dig_P3)>>8) + ((var1 * (int64_t)calib_dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calib_dig_P1)>>33;
    if (var1 == 0)
    {
        return 0;
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)calib_dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)calib_dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_dig_P7)<<4);
    return (uint32_t)p;
}

static uint32_t comp_H_int32(int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (calib_t_fine_local - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib_dig_H4) << 20) - (((int32_t)calib_dig_H5) *
            v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
                    ((int32_t)calib_dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calib_dig_H3)) >> 11) +
                            ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)calib_dig_H2) +
                    8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
            ((int32_t)calib_dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r>>12);
}

int bmp280_read_calib(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    uint8_t trimdata[32];
    HAL_StatusTypeDef rc;
    uint16_t devaddr = (uint16_t)(addr << 1);

    // Read first block 0x88..0xA1 (25 bytes)
    rc = HAL_I2C_Mem_Read(hi2c, devaddr, 0x88, I2C_MEMADD_SIZE_8BIT, trimdata, 25, 1000);
    if (rc != HAL_OK) return 1;

    // Read second block 0xE1..0xE7 (7 bytes)
    rc = HAL_I2C_Mem_Read(hi2c, devaddr, 0xE1, I2C_MEMADD_SIZE_8BIT, trimdata + 25, 7, 1000);
    if (rc != HAL_OK) return 2;

    // Parse values (exactly as BME280 driver)
    calib_dig_T1 = (uint16_t)(trimdata[1] << 8 | trimdata[0]);
    calib_dig_T2 = (int16_t)(trimdata[3] << 8 | trimdata[2]);
    calib_dig_T3 = (int16_t)(trimdata[5] << 8 | trimdata[4]);

    calib_dig_P1 = (uint16_t)(trimdata[7] << 8 | trimdata[6]);
    calib_dig_P2 = (int16_t)(trimdata[9] << 8 | trimdata[8]);
    calib_dig_P3 = (int16_t)(trimdata[11] << 8 | trimdata[10]);
    calib_dig_P4 = (int16_t)(trimdata[13] << 8 | trimdata[12]);
    calib_dig_P5 = (int16_t)(trimdata[15] << 8 | trimdata[14]);
    calib_dig_P6 = (int16_t)(trimdata[17] << 8 | trimdata[16]);
    calib_dig_P7 = (int16_t)(trimdata[19] << 8 | trimdata[18]);
    calib_dig_P8 = (int16_t)(trimdata[21] << 8 | trimdata[20]);
    calib_dig_P9 = (int16_t)(trimdata[23] << 8 | trimdata[22]);

    calib_dig_H1 = trimdata[24];
    calib_dig_H2 = (int16_t)(trimdata[26] << 8 | trimdata[25]);
    calib_dig_H3 = trimdata[27];
    calib_dig_H4 = (int16_t)((trimdata[28] << 4) | (trimdata[29] & 0x0F));
    calib_dig_H5 = (int16_t)((trimdata[30] << 4) | (trimdata[29] >> 4));
    calib_dig_H6 = (int8_t)trimdata[31];

    return 0; // success
}

int bmp280_read_measurement(I2C_HandleTypeDef *hi2c, uint8_t addr, float *tempC, float *press_hPa)
{
    uint8_t RawData[8];
    HAL_StatusTypeDef rc;
    uint16_t devaddr = (uint16_t)(addr << 1);

    // Read registers 0xF7..0xFE (8 bytes)
    rc = HAL_I2C_Mem_Read(hi2c, devaddr, 0xF7, I2C_MEMADD_SIZE_8BIT, RawData, 8, HAL_MAX_DELAY);
    if (rc != HAL_OK) return 1;

    int32_t pRaw = (RawData[0]<<12)|(RawData[1]<<4)|(RawData[2]>>4);
    int32_t tRaw = (RawData[3]<<12)|(RawData[4]<<4)|(RawData[5]>>4);
    int32_t hRaw = (RawData[6]<<8)|(RawData[7]);

    if (tRaw == 0x800000) *tempC = 0;
    else *tempC = (comp_T_int32(tRaw)) / 100.0f;

    if (pRaw == 0x800000) *press_hPa = 0;
    else *press_hPa = (comp_P_int64(pRaw)) / 256.0f / 100.0f; // convert Pa to hPa

    // humidity not used by main, but compute if needed
    (void)hRaw;

    return 0; // success
}
