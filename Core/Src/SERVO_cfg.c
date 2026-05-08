/*
 * File: SERVO_cfg.c
 * Driver Name: [[ SERVO Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "SERVO.h"

const SERVO_CfgType SERVO_CfgParam[SERVO_NUM] =
{
	// Servo 0 -> PB8 -> TIM4 CH3 (changed from PC6 TIM3 CH1)
    {
        GPIOB,
        GPIO_PIN_8,
        TIM4,
        (uint32_t*)&TIM4->CCR3,
        TIM_CHANNEL_3,
        240000000,
        1.0,
        2.0
    },
    // Servo 1 -> PC9 -> TIM3 CH4
    {
        GPIOC,
        GPIO_PIN_9,
        TIM3,
        (uint32_t*)&TIM3->CCR4,
        TIM_CHANNEL_4,
        240000000,
        1.0,
        2.0
    }
};