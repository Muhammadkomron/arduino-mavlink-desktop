/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "MPUXX50_h7.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "INA219.h"
// removed SERVO.h (servo functionality removed from main)
#include "compat_helpers.h"
// Enable minmea compatibility include and NMEA parsing
#define MINMEA_INCLUDE_COMPAT
#include "minmea.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4; // added handle for TIM4
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
INA219_t ina219;
INA219_t ina219_2; // second INA219 on I2C2 (PB10/PB11)
/* USER CODE BEGIN PV */
// Increased serial buffer to hold JSON payload safely
uint8_t serialBuf[512];
volatile uint8_t imuTick = 0; // set by timer ISR to indicate IMU sample is due
uint8_t ina_present = 0; // flag set when INA219 detected on I2C1
uint8_t ina2_present = 0; // flag set when INA219 detected on I2C2

// UART1 (GPS) RX handling
volatile uint8_t uart1_rx_byte = 0;
#define NMEA_BUF_LEN 128
static char nmea_buf[NMEA_BUF_LEN];
static uint16_t nmea_idx = 0;

// Parsed GPS values
static float gps_lat = 0.0f;
static float gps_lon = 0.0f;
static float gps_alt = 0.0f;
static float gps_speed = 0.0f; // in knots (from RMC)
static int gps_fixq = 0; // GGA fix quality
static int gps_valid = 0; // RMC validity flag

// JSON frame counter
static uint32_t frame_seq = 0;

// BMP280 state
uint8_t bmp_present = 0;
uint32_t lastBmpTry = 0;
const uint32_t bmpRetryIntervalMs = 5000; // attempt calibration every 5s if not present
int32_t bmp_t_fine = 0;
uint16_t bmp_dig_T1; int16_t bmp_dig_T2; int16_t bmp_dig_T3;
uint16_t bmp_dig_P1; int16_t bmp_dig_P2; int16_t bmp_dig_P3; int16_t bmp_dig_P4; int16_t bmp_dig_P5;
int16_t bmp_dig_P6; int16_t bmp_dig_P7; int16_t bmp_dig_P8; int16_t bmp_dig_P9;

/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
// XBee and AT-probe removed: no prototypes here
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  // Start USART1 receive in interrupt mode for GPS NMEA input
  if (HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart1_rx_byte, 1) != HAL_OK)
  {
    int l = sprintf((char*)serialBuf, "Failed to start USART1 RX\r\n");
    if (l>0) HAL_UART_Transmit(&huart4, serialBuf, l, HAL_MAX_DELAY);
  }
  /* USER CODE BEGIN 2 */

  // Removed XBee AT probing and continuous TX. We'll just use huart1 TX to send JSON payloads.

  // Initialize INA219 on I2C1 (PB6=SCL, PB7=SDA)
  if (INA219_Init(&ina219, &hi2c1, INA219_ADDRESS) == 1)
  {
    ina_present = 1;
    sprintf((char *)serialBuf, "INA219 detected on I2C1\r\n");
    HAL_UART_Transmit(&huart4, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
  }
  else
  {
    ina_present = 0;
    sprintf((char *)serialBuf, "INA219 NOT detected on I2C1\r\n");
    HAL_UART_Transmit(&huart4, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
  }

  // Initialize second INA219 on I2C2 (PB10=SCL, PB11=SDA expected)
  if (INA219_Init(&ina219_2, &hi2c2, INA219_ADDRESS) == 1)
  {
    ina2_present = 1;
    sprintf((char *)serialBuf, "INA219 detected on I2C2\r\n");
    HAL_UART_Transmit(&huart4, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
  }
  else
  {
    ina2_present = 0;
    sprintf((char *)serialBuf, "INA219 NOT detected on I2C2\r\n");
    HAL_UART_Transmit(&huart4, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
  }

  // Initialize and check MPU9250 on I2C4 (PD12/PD13 on board)
  if (MPU_begin(&hi2c4, AD0_LOW, AFSR_4G, GFSR_500DPS, 0.98f, 0.004f) == 1)
  {
    sprintf((char *)serialBuf, "MPU detected\r\n");
    HAL_UART_Transmit(&huart4, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    sprintf((char *)serialBuf, "CALIBRATING GYRO...\r\n");
    HAL_UART_Transmit(&huart4, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
    MPU_calibrateGyro(&hi2c4, 1500);
    sprintf((char *)serialBuf, "CALIBRATION DONE\r\n");
    HAL_UART_Transmit(&huart4, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
    /* Start TIM1 to drive periodic attitude updates */
    HAL_TIM_Base_Start_IT(&htim1);

    // --- Try BMP280 detection at 0x76 (same I2C bus as MPU) ---
    uint8_t bmp_id = 0;
    if (HAL_I2C_Mem_Read(&hi2c4, (uint16_t)(0x76<<1), 0xD0, I2C_MEMADD_SIZE_8BIT, &bmp_id, 1, 200) == HAL_OK)
    {
      int l = sprintf((char*)serialBuf, "BMP280 detected @0x76, chip id=0x%02X\r\n", bmp_id);
      if (l>0) HAL_UART_Transmit(&huart4, serialBuf, l, HAL_MAX_DELAY);
      // read calibration data
      if (bmp280_read_calib(&hi2c4, 0x76) == 0)
      {
        bmp_present = 1;
      }
      else
      {
        bmp_present = 0; // will retry later
        lastBmpTry = HAL_GetTick();
      }
    }
    else
    {
      int l = sprintf((char*)serialBuf, "BMP280 @0x76 not responding\r\n");
      if (l>0) HAL_UART_Transmit(&huart4, serialBuf, l, HAL_MAX_DELAY);
      lastBmpTry = HAL_GetTick();
    }

  }
  else
  {
    sprintf((char *)serialBuf, "MPU NOT detected\r\n");
    HAL_UART_Transmit(&huart4, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
    /* Blink PA8 indefinitely to indicate error instead of calling Error_Handler() */
    while (1)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
      HAL_Delay(250);
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastPrint = 0;
  while (1)
  {
    if (imuTick)
    {
      imuTick = 0;
      // Update integrated attitude and processed sensorData (gx/gy/gz in dps, ax/ay/az in g)
      MPU_calcAttitude(&hi2c4);
    }

    // BMP calibration retry: if not present, try again every bmpRetryIntervalMs
    if (!bmp_present && (HAL_GetTick() - lastBmpTry) >= bmpRetryIntervalMs)
    {
      lastBmpTry = HAL_GetTick();
      if (bmp280_read_calib(&hi2c4, 0x76) == 0)
      {
        bmp_present = 1;
        int l = sprintf((char*)serialBuf, "BMP calibration succeeded on retry\r\n");
        if (l>0) HAL_UART_Transmit(&huart4, serialBuf, l, HAL_MAX_DELAY);
      }
      else
      {
        int l = sprintf((char*)serialBuf, "BMP calibration retry failed\r\n");
        if (l>0) HAL_UART_Transmit(&huart4, serialBuf, l, HAL_MAX_DELAY);
      }
    }

    // Compose and send JSON at 1Hz with MPU (ax/ay/az,gx/gy/gz,roll/pitch/yaw) and INA219 readings
    if ((HAL_GetTick() - lastPrint) >= 1000)
    {
      lastPrint = HAL_GetTick();

      // Ensure latest processed data available (read processed data without modifying attitude)
      MPU_readProcessedData(&hi2c4); // populates sensorData globals

      float ax = sensorData.ax; float ay = sensorData.ay; float az = sensorData.az;
      float gx = sensorData.gx; float gy = sensorData.gy; float gz = sensorData.gz;
      float roll = MPU_getRoll(); float pitch = MPU_getPitch(); float yaw = MPU_getYaw();

      // INA219 readings (I2C1)
      float bus_mV = 0.0f, shunt_mV = 0.0f, current_mA = 0.0f, power_mW = 0.0f;
      if (ina_present)
      {
        bus_mV = INA219_ReadBusVoltage(&ina219);
        shunt_mV = INA219_ReadShuntVolage(&ina219);
        current_mA = INA219_ReadCurrent(&ina219);
        power_mW = INA219_ReadPower(&ina219);
      }

      // INA219 on I2C2 readings
      float bus1_mV = 0.0f, shunt1_mV = 0.0f, current1_mA = 0.0f, power1_mW = 0.0f;
      if (ina2_present)
      {
        bus1_mV = INA219_ReadBusVoltage(&ina219_2);
        shunt1_mV = INA219_ReadShuntVolage(&ina219_2);
        current1_mA = INA219_ReadCurrent(&ina219_2);
        power1_mW = INA219_ReadPower(&ina219_2);
      }

      // BMP readings
      float bt = 0.0f, bp = 0.0f;
      if (bmp_present)
      {
        if (bmp280_read_measurement(&hi2c4, 0x76, &bt, &bp) != 0)
        {
          // read failed; mark as zero/NaN (we'll send 0.0)
          bt = 0.0f; bp = 0.0f;
        }
      }

      // Build compact JSON with one-word keys
      // Example: {"voltage":5000.00,"current":120.00,"voltage1":...,"ax":0.00,...}
      int len = snprintf((char*)serialBuf, sizeof(serialBuf),
        "{\"id\":1003,\"seq\":%lu,\"voltage\":%.2f,\"current\":%.2f,\"voltage1\":%.2f,\"current1\":%.2f,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,\"roll\":%.3f,\"pitch\":%.3f,\"yaw\":%.3f,\"temp\":%.2f,\"pressure\":%.2f,\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.2f,\"spd\":%.2f,\"fq\":%d}\r\n",
        (unsigned long)frame_seq++, bus_mV, current_mA, bus1_mV, current1_mA, ax, ay, az, gx, gy, gz, roll, pitch, yaw, bt, bp,
        gps_lat, gps_lon, gps_alt, gps_speed, gps_fixq);

      if (len < 0)
      {
        len = 0;
      }
      else if (len >= (int)sizeof(serialBuf))
      {
        len = (int)sizeof(serialBuf) - 1;
        serialBuf[len] = '\0';
      }

      if (len > 0)
      {
        // Send JSON to the XBee on USART1 TX (PA9) and also mirror it on debug UART4.
        HAL_UART_Transmit(&huart1, serialBuf, len, 200);
        HAL_UART_Transmit(&huart4, serialBuf, len, HAL_MAX_DELAY);
      }
    }

    /* small sleep to avoid busy-waiting */
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
 RCC_OscInitTypeDef RCC_OscInitStruct = {0};
 RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
 /** Supply configuration update enable
 */
 HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
 /** Configure the main internal regulator output voltage
 */
 __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
 while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
 /** Initializes the RCC Oscillators according to the specified parameters
 * in the RCC_OscInitTypeDef structure.
 */
 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
 RCC_OscInitStruct.HSEState = RCC_HSE_ON;
 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
 RCC_OscInitStruct.PLL.PLLM = 5;
 RCC_OscInitStruct.PLL.PLLN = 192;
 RCC_OscInitStruct.PLL.PLLP = 2;
 RCC_OscInitStruct.PLL.PLLQ = 2;
 RCC_OscInitStruct.PLL.PLLR = 2;
 RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
 RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
 RCC_OscInitStruct.PLL.PLLFRACN = 0;
 if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
 {
   Error_Handler();
 }
 /** Initializes the CPU, AHB and APB buses clocks
 */
 RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                             |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                             |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
 RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
 RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
 RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
 RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
 RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
 RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
 RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
 {
   Error_Handler();
 }
}
/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{
 /* USER CODE BEGIN I2C1_Init 0 */
 /* USER CODE END I2C1_Init 0 */
 /* USER CODE BEGIN I2C1_Init 1 */
 /* USER CODE END I2C1_Init 1 */
 hi2c1.Instance = I2C1;
 hi2c1.Init.Timing = 0x307075B1;
 hi2c1.Init.OwnAddress1 = 0;
 hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
 hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
 hi2c1.Init.OwnAddress2 = 0;
 hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
 hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
 hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
 if (HAL_I2C_Init(&hi2c1) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Analogue filter
 */
 if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Digital filter
 */
 if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN I2C1_Init 2 */
 /* USER CODE END I2C1_Init 2 */
}
/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{
 /* USER CODE BEGIN I2C2_Init 0 */
 /* USER CODE END I2C2_Init 0 */
 /* USER CODE BEGIN I2C2_Init 1 */
 /* USER CODE END I2C2_Init 1 */
 hi2c2.Instance = I2C2;
 hi2c2.Init.Timing = 0x307075B1;
 hi2c2.Init.OwnAddress1 = 0;
 hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
 hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
 hi2c2.Init.OwnAddress2 = 0;
 hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
 hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
 hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
 if (HAL_I2C_Init(&hi2c2) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Analogue filter
 */
 if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Digital filter
 */
 if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN I2C2_Init 2 */
 /* USER CODE END I2C2_Init 2 */
}
/**
 * @brief I2C4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C4_Init(void)
{
 /* USER CODE BEGIN I2C4_Init 0 */
 /* USER CODE END I2C4_Init 0 */
 /* USER CODE BEGIN I2C4_Init 1 */
 /* USER CODE END I2C4_Init 1 */
 hi2c4.Instance = I2C4;
 hi2c4.Init.Timing = 0x307075B1;
 hi2c4.Init.OwnAddress1 = 0;
 hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
 hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
 hi2c4.Init.OwnAddress2 = 0;
 hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
 hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
 hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
 if (HAL_I2C_Init(&hi2c4) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Analogue filter
 */
 if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Digital filter
 */
 if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN I2C4_Init 2 */
 /* USER CODE END I2C4_Init 2 */
}
/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{
 /* USER CODE BEGIN TIM1_Init 0 */
 /* USER CODE END TIM1_Init 0 */
 TIM_ClockConfigTypeDef sClockSourceConfig = {0};
 TIM_MasterConfigTypeDef sMasterConfig = {0};
 /* USER CODE BEGIN TIM1_Init 1 */
 /* USER CODE END TIM1_Init 1 */
 htim1.Instance = TIM1;
 htim1.Init.Prescaler = 2400-1;
 htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
 htim1.Init.Period = 500-1;
 htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 htim1.Init.RepetitionCounter = 0;
 htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
 if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
 {
   Error_Handler();
 }
 sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
 if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
 {
   Error_Handler();
 }
 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
 sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN TIM1_Init 2 */
 /* USER CODE END TIM1_Init 2 */
}
/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{
 /* USER CODE BEGIN TIM3_Init 0 */
 /* USER CODE END TIM3_Init 0 */
 TIM_ClockConfigTypeDef sClockSourceConfig = {0};
 TIM_MasterConfigTypeDef sMasterConfig = {0};
 TIM_OC_InitTypeDef sConfigOC = {0};
 /* USER CODE BEGIN TIM3_Init 1 */
 /* USER CODE END TIM3_Init 1 */
 htim3.Instance = TIM3;
 htim3.Init.Prescaler = 0;
 htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
 htim3.Init.Period = 65535;
 htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
 {
   Error_Handler();
 }
 sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
 if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
 {
   Error_Handler();
 }
 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
 {
   Error_Handler();
 }
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = 0;
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN TIM3_Init 2 */
 /* USER CODE END TIM3_Init 2 */
 HAL_TIM_MspPostInit(&htim3);
}
/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{
 /* USER CODE BEGIN UART4_Init 0 */
 /* USER CODE END UART4_Init 0 */
 /* USER CODE BEGIN UART4_Init 1 */
 /* USER CODE END UART4_Init 1 */
 huart4.Instance = UART4;
 huart4.Init.BaudRate = 115200; // changed from 57600 to 115200
 huart4.Init.WordLength = UART_WORDLENGTH_8B;
 huart4.Init.StopBits = UART_STOPBITS_1;
 huart4.Init.Parity = UART_PARITY_NONE;
 huart4.Init.Mode = UART_MODE_TX_RX;
 huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
 huart4.Init.OverSampling = UART_OVERSAMPLING_16;
 huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
 huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
 huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
 if (HAL_UART_Init(&huart4) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN UART4_Init 2 */
 /* USER CODE END UART4_Init 2 */
}
/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
 /* USER CODE BEGIN USART1_Init 0 */
 /* USER CODE END USART1_Init 0 */
 /* USER CODE BEGIN USART1_Init 1 */
 /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200; // changed to 115200 per user request
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
 if (HAL_UART_Init(&huart1) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN USART1_Init 2 */
 /* USER CODE END USART1_Init 2 */
}
/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
 GPIO_InitTypeDef GPIO_InitStruct = {0};
 /* USER CODE BEGIN MX_GPIO_Init_1 */
 /* USER CODE END MX_GPIO_Init 1 */
 /* GPIO Ports Clock Enable */
 __HAL_RCC_GPIOH_CLK_ENABLE();
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_GPIOB_CLK_ENABLE();
 __HAL_RCC_GPIOD_CLK_ENABLE();
 __HAL_RCC_GPIOC_CLK_ENABLE();
 /*Configure GPIO pin Output Level */
 // Keep PA8 behavior as-is for LED/status. Configure PA11 high (not reset) by default
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
 /*Configure GPIO pin : PA8 */
 GPIO_InitStruct.Pin = GPIO_PIN_8;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 /* Configure GPIO pin : PA11 (XBee RESET, active low) */
 GPIO_InitStruct.Pin = GPIO_PIN_11;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 /* USER CODE BEGIN MX_GPIO_Init_2 */
 /* USER CODE END MX_GPIO_Init 2 */
}
/* USER CODE BEGIN 4 */
/* Callback: timer has rolled over (TIM1 used for IMU sampling) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
    // Do not call I2C/HAL functions here (ISR). Just set a flag and process in main loop.
    imuTick = 1;
  }
}

// Continuous TX for XBee removed: keep callback but do not requeue huart1 transmit automatically
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  // No-op: we perform blocking or single non-blocking sends in main loop. Do not auto-restart any transmit.
  (void)huart;
}

// UART RX callback: accumulate NMEA bytes and parse complete sentences
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    char c = (char)uart1_rx_byte;
    if (c == '\n' || nmea_idx >= (NMEA_BUF_LEN - 2))
    {
      nmea_buf[nmea_idx++] = '\0';
      // parse sentence if checksum ok
      if (minmea_check(nmea_buf, true))
      {
        enum minmea_sentence_id sid = minmea_sentence_id(nmea_buf, true);
        if (sid == MINMEA_SENTENCE_GGA)
        {
          struct minmea_sentence_gga frame;
          if (minmea_parse_gga(&frame, nmea_buf))
          {
            gps_lat = minmea_tocoord(&frame.latitude);
            gps_lon = minmea_tocoord(&frame.longitude);
            gps_alt = minmea_tofloat(&frame.altitude);
            gps_fixq = frame.fix_quality;
          }
        }
        else if (sid == MINMEA_SENTENCE_RMC)
        {
          struct minmea_sentence_rmc rmc;
          if (minmea_parse_rmc(&rmc, nmea_buf))
          {
            gps_lat = minmea_tocoord(&rmc.latitude);
            gps_lon = minmea_tocoord(&rmc.longitude);
            gps_speed = minmea_tofloat(&rmc.speed); // knots
            gps_valid = rmc.valid ? 1 : 0;
          }
        }
      }
      nmea_idx = 0;
    }
    else if (c != '\r')
    {
      nmea_buf[nmea_idx++] = c;
    }
    // re-arm receive for next byte
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart1_rx_byte, 1);
  }
}

/* USER CODE END 4 */
/* MPU Configuration */
void MPU_Config(void)
{
 MPU_Region_InitTypeDef MPU_InitStruct = {0};
 /* Disables the MPU */
 HAL_MPU_Disable();
 /** Initializes and configures the Region and the memory to be protected
 */
 MPU_InitStruct.Enable = MPU_REGION_ENABLE;
 MPU_InitStruct.Number = MPU_REGION_NUMBER0;
 MPU_InitStruct.BaseAddress = 0x0;
 MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
 MPU_InitStruct.SubRegionDisable = 0x87;
 MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
 MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
 MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
 MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
 MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
 MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
 HAL_MPU_ConfigRegion(&MPU_InitStruct);
 /* Enables the MPU */
 HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
 /* USER CODE BEGIN Error_Handler_Debug */
 /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
 /* USER CODE BEGIN 6 */
 /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */