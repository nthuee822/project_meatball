/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_i2c.h"
#include "stm32g4xx_hal_tim.h"

#include "stm32-cordic.h"
#include "as5600.h"
#include "stm32g4xx_hal_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_SIZE 1024
#define OUTPUT_BUFLEN 64
#define ANGLE_WRAP_THRESHOLD 300
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CORDIC_HandleTypeDef hcordic;

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile bool active = false;
volatile bool state_change = false;
volatile bool next_step = false;
volatile bool i2c_dma_ready = false;

uint16_t angle_log[LOG_SIZE];
uint16_t duty_log_x[LOG_SIZE];
uint16_t duty_log_y[LOG_SIZE];
int16_t adc_log_x[LOG_SIZE];
int16_t adc_log_y[LOG_SIZE];
int32_t adc_log_d[LOG_SIZE];
int32_t adc_log_q[LOG_SIZE];

uint8_t output_buffer[OUTPUT_BUFLEN];
int output_length;

AS5600_TypeDef *myAS5600;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
static inline int32_t update_cumulative_angle(int32_t cumulative_angle_prev, uint16_t raw_angle_prev, uint16_t raw_angle);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CORDIC_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  uint32_t adc_dma_dest[2];
  int32_t cordic_results[2];

  int32_t theta_q31 = 0x80000000;   // corresponding to -pi
  int32_t sine_q31 = 0, cosine_q31 = 0;
  uint16_t pwm_level_x = 0, pwm_level_y = 0;
  int16_t adc_x, adc_y;

  int cycle_count = 0;    // 0 -> initiate angle reading, 2 -> Kalman update
  int logging_index = 0;
  uint16_t angle_raw, angle_raw_prev, angle_home;
  int32_t cumulative_angle, angle_dq;
  int32_t sine_dq, cosine_dq;
  int32_t adc_d, adc_q;
  
  myAS5600 = AS5600_New();
  myAS5600->i2cAddr = (0x36 << 1);  // 7-bit address shifted for HAL
  myAS5600->i2cHandle = &hi2c2;
  myAS5600->SlowFilter = AS5600_SLOW_FILTER_2X;
  myAS5600->FastFilterThreshold = AS5600_FAST_FILTER_6LSB;

  AS5600_Init(myAS5600);

  /* Measure ADC DC offset after calibration by averaging a few samples */
  ADC_ChannelConfTypeDef adc_offset_config = {0};

  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }
  adc_offset_config.Channel = ADC_CHANNEL_1;
  adc_offset_config.Rank = ADC_REGULAR_RANK_1;
  adc_offset_config.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  adc_offset_config.SingleDiff = ADC_SINGLE_ENDED;
  adc_offset_config.OffsetNumber = ADC_OFFSET_NONE;
  adc_offset_config.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &adc_offset_config) != HAL_OK) {
    Error_Handler();
  }

  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    Error_Handler();
  }
  adc_offset_config.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &adc_offset_config) != HAL_OK) {
    Error_Handler();
  }

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  const uint8_t adc_offset_samples = 64;
  uint32_t adc1_offset_sum = 0;
  uint32_t adc2_offset_sum = 0;
  uint16_t adc1_offset = 0;
  uint16_t adc2_offset = 0;

  for (uint8_t i = 0; i < adc_offset_samples; i++) {
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
      Error_Handler();
    }
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
      Error_Handler();
    }
    adc1_offset_sum += HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    if (HAL_ADC_Start(&hadc2) != HAL_OK) {
      Error_Handler();
    }
    if (HAL_ADC_PollForConversion(&hadc2, 10) != HAL_OK) {
      Error_Handler();
    }
    adc2_offset_sum += HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);

    HAL_Delay(1);
  }

  adc1_offset = (uint16_t)(adc1_offset_sum / adc_offset_samples);
  adc2_offset = (uint16_t)(adc2_offset_sum / adc_offset_samples);
  // (void)adc1_offset;
  // (void)adc2_offset;

  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(active) {
      if(state_change) {  // first cycle after activation
        state_change = false;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);   // light up LED

        // Start ADC and DMA
        HAL_ADC_Start_DMA(&hadc1, &adc_dma_dest[0], 1);
        __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOC);
        HAL_ADC_Start_DMA(&hadc2, &adc_dma_dest[1], 1);
        // __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_EOC);

        // Start TIMER
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

        // set initial PWM duty cycle for homing
        cordic_q31_cos_sin(theta_q31, cordic_results);
        cosine_q31 = cordic_results[0];
        sine_q31 = cordic_results[1];
        pwm_level_x = (cosine_q31 >> 23) + 512;
        pwm_level_y = (sine_q31 >> 23) + 512;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_level_x);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_level_y);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);

        // delay for rotor to rotate to home position
        HAL_Delay(50);

        // read home position
        AS5600_GetAngle(myAS5600, &angle_home);
        cumulative_angle = -((int32_t)angle_home);   // set home position to 0
      }

      if(next_step) {   // ADC sampling just completed
        next_step = false;    // clear flag

        // update pwm duty cycle
        cordic_q31_cos_sin(theta_q31, cordic_results);
        cosine_q31 = cordic_results[0];
        sine_q31 = cordic_results[1];
        pwm_level_x = (cosine_q31 >> 23) + 512;
        pwm_level_y = (sine_q31 >> 23) + 512;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_level_x);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_level_y);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);

        theta_q31 += 0x2000000;    // let it overflow to loop back and keep spinning

        // perform tasks based on cycle_count
        switch(cycle_count) {
          case 0:
            AS5600_GetAngleDMA(myAS5600);
            cycle_count++;
            break;
          case 1:
            cycle_count++;
            break;
          case 2:
            if(i2c_dma_ready) {   // update angle if AS5600 is ready
              angle_raw_prev = angle_raw;
              angle_raw = (as5600_dma_dest[0] << 8) | as5600_dma_dest[1];
              cumulative_angle = update_cumulative_angle(cumulative_angle, angle_raw_prev, angle_raw);
            }

            adc_x = (adc_dma_dest[0] & 0x0000ffff) - adc1_offset;
            adc_y = (adc_dma_dest[1] & 0x0000ffff) - adc2_offset;

            // compute angle for DQ measurement
            // 12-bit measured angle -> 32-bit cordic angle -> 32-bit cordic sin/cos -> 12-bit sin/cos
            angle_dq = (cumulative_angle * 50) << 20;
            cordic_q31_cos_sin(angle_dq, cordic_results);
            cosine_dq = cordic_results[0] >> 20;
            sine_dq = cordic_results[1] >> 20;
            adc_d = -sine_dq * adc_x + cosine_dq * adc_y;
            adc_q = cosine_dq * adc_x + sine_dq * adc_y;

            // log data
            angle_log[logging_index] = cumulative_angle;
            duty_log_x[logging_index] = pwm_level_x;
            duty_log_y[logging_index] = pwm_level_y;
            adc_log_x[logging_index] = adc_x;
            adc_log_y[logging_index] = adc_y;
            adc_log_d[logging_index] = adc_d;
            adc_log_q[logging_index] = adc_q;

            if(logging_index < LOG_SIZE - 1) {
              logging_index++;
            } else {
              logging_index = 0;
            }

            cycle_count = 0;
            break;
        }
      }

    } else {
      if(state_change) {   // first cycle after stopping
        state_change = false;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);   // turn off LED
      
        // turn off motor current
        HAL_ADC_Stop_DMA(&hadc1);
        __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_EOC);
        HAL_ADC_Stop_DMA(&hadc2);
        // __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_EOC);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

        // print logged data to serial port
        for(int i = 0; i < LOG_SIZE; i++) {
          output_length = snprintf((char *)output_buffer, OUTPUT_BUFLEN, "%d %d %d %d %d %ld %ld\r\n", 
            duty_log_x[logging_index], duty_log_y[logging_index], angle_log[logging_index], 
            adc_log_x[logging_index], adc_log_y[logging_index], adc_log_d[logging_index], adc_log_q[logging_index]);
          // output_length = snprintf((char *)output_buffer, OUTPUT_BUFLEN, "%d %d %d %d %d\r\n", 
          //   duty_log_x[logging_index], duty_log_y[logging_index], angle_log[logging_index], 
          //   adc_log_x[logging_index], adc_log_y[logging_index]);
          
          if(logging_index < LOG_SIZE - 1) {
            logging_index++;
          } else {
            logging_index = 0;
          }
          
          HAL_UART_Transmit(&huart2, output_buffer, output_length, 10);
          HAL_Delay(1);
        }
      }

      // reset variables
      theta_q31 = 0x80000000;   // corresponding to -pi
      sine_q31 = 0;
      cosine_q31 = 0;
      pwm_level_x = 0;
      pwm_level_y = 0;

      cycle_count = 0;
      logging_index = 0;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 28;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

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
  hi2c2.Init.Timing = 0x10C31027;
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

  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C2);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1023;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 7;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin == GPIO_PIN_13) {
    active = !active;
    state_change = true;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if(hadc == &hadc1) {
    next_step = true;
  }
}

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//   if(htim == &htim1) {
//     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
//   }
// }

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if(hi2c == &hi2c2) {
    i2c_dma_ready = true;
  }
}

static inline int32_t update_cumulative_angle(int32_t cumulative_angle_prev, uint16_t raw_angle_prev, uint16_t raw_angle) {
  int32_t cumulative_angle;
  if(raw_angle_prev > 4095 - ANGLE_WRAP_THRESHOLD && raw_angle < ANGLE_WRAP_THRESHOLD) {
    // forward wrap around
    cumulative_angle = cumulative_angle_prev + (4096 - raw_angle_prev + raw_angle);
  } else if(raw_angle_prev < ANGLE_WRAP_THRESHOLD && raw_angle > 4095 - ANGLE_WRAP_THRESHOLD) {
    // backward wrap around
    cumulative_angle = cumulative_angle_prev - (4096 - raw_angle + raw_angle_prev);
  } else {
    // no wrap around
    cumulative_angle = cumulative_angle_prev + (raw_angle - raw_angle_prev);
  }

  return cumulative_angle;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
