/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>

#include "envelope.h"
#include "btn.h"
#include "led_rgb.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Definition of ADCx conversions data table size */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  4)   /* Size of array aADCxConvertedData[] */

/* For PWM */
#define  PULSE1_VALUE       (uint32_t)(TIM1_PER/2)
#define  PULSE2_VALUE       (uint32_t)(TIM1_PER/2)
#define  PULSE3_VALUE       (uint32_t)(TIM1_PER/2)

#define TOGGLE_COUNT   1000   

#define BLINK_RATE     750

//#define SH_TIME (uint32_t)7 // give the dac 1us and the LF398 6us

#define GATE_HIGH GPIO_PIN_RESET
#define GATE_LOW  GPIO_PIN_SET

#define SEG_NO_CHANGE 0xFFFF
#define CHANGE_THRESH (ADC_FS / 8)

//#define TOGGLE ; // Debug toggle of LD3 on the nucleo
//#define SEEENVS ; // Debug make the leds flash like an envelope


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Variable containing ADC conversions data */
static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

static GPIO_PinState  trigger_pins[3];

//some of these could become flags for efficiency?
#ifdef TOGGLE
static uint16_t	  toggle;
#endif
static uint16_t	  tick, last_tick, btn_tick;
static uint16_t	  dac;
static uint16_t	  sh_timer;
static uint16_t	  i, j;
static uint16_t	  seg_change; // 0xFFFF means no param changed, otherwise 0 -> 3 map to ADSR
static enum curves new_seg;

static env_gen_t envelopes[NUM_ENVS];
static btn_t	 btn;
static led_rgb_t led;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  if (HAL_ADC_Start_DMA(&hadc1,
                        (uint32_t *)aADCxConvertedData,
                        ADC_CONVERTED_DATA_BUFFER_SIZE
                       ) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /*## Start PWM signals generation #######################################*/
  /* Start channel 1 */
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 2 */
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 3 */
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }

#ifdef TOGGLE
  toggle = 0;
#endif
  tick = 0;
  last_tick = 0;
  btn_tick = 1;
  dac = 0;
  sh_timer = 0;
  seg_change = SEG_NO_CHANGE;

  init_btn(&btn);
  init_led_rgb(&led, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3);


  for (i = 0; i < NUM_ENVS; i++) {
    init_env(&envelopes[i]);
  }
  envelopes[1].acurve = EXP;
  envelopes[1].dcurve = EXP;
  envelopes[1].rcurve = EXP;
  envelopes[2].acurve = LOG;
  envelopes[2].dcurve = LOG;
  envelopes[2].rcurve = LOG;

  i = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (tick != last_tick) {

#ifdef TOGGLE
      // take care of flashing the LD3 to show signs of life
      toggle++;
      if (toggle >= TOGGLE_COUNT) {
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	toggle = 0;
      }
#endif

      // Check the TRIG inputs
      trigger_pins[0] = HAL_GPIO_ReadPin(GPIOA, TRIG1_Pin);
      trigger_pins[1] = HAL_GPIO_ReadPin(GPIOB, TRIG2_Pin);
      trigger_pins[2] = HAL_GPIO_ReadPin(GPIOB, TRIG3_Pin);

      // Check the envelope pots
      // If we're doing a long press we shouldn't
      if (btn.click_state == LONG) {
	//don't do anything here - we're going to check if they're different below and 
	//update segment shapes choices based on these numbers
	//Basically hold the button and move the pot of the parameter that you want to change shape
      } else {
	envelopes[i].adsr[ATTACK]   = aADCxConvertedData[ATTACK];
      	envelopes[i].adsr[DECAY]    = aADCxConvertedData[DECAY];
      	envelopes[i].adsr[SUSTAIN]  = aADCxConvertedData[SUSTAIN];
      	envelopes[i].adsr[RELEASE]  = aADCxConvertedData[RELEASE];
      }

      // Process the envelopes
      process_env(&envelopes[0], (trigger_pins[0] == GATE_HIGH) ? 1 : 0, envelopes[2].end);
      process_env(&envelopes[1], (trigger_pins[1] == GATE_HIGH) ? 1 : 0, envelopes[0].end);
      process_env(&envelopes[2], (trigger_pins[2] == GATE_HIGH) ? 1 : 0, envelopes[1].end);

      // Step through the envs and set the DAC accordingly, give it 1uS to stablelize plus 6uS of Sample time
      if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ENV_INV(&envelopes[0])) != HAL_OK)
      {
        /* Setting value Error */
        Error_Handler();
      }
      // Assume 6uS sampling time once DAC is stable
      HAL_GPIO_WritePin(SH1_GPIO_Port, SH1_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_AUTORELOAD(&htim7, SH_TIME);
      HAL_TIM_Base_Start_IT(&htim7);
      while (!sh_timer) {
	//spin on the timer
      }
      sh_timer = 0;
      HAL_GPIO_WritePin(SH1_GPIO_Port, SH1_Pin, GPIO_PIN_RESET);

      if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ENV_INV(&envelopes[1])) != HAL_OK)
      {
        /* Setting value Error */
        Error_Handler();
      }
      HAL_GPIO_WritePin(SH2_GPIO_Port, SH2_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_AUTORELOAD(&htim7, SH_TIME);
      HAL_TIM_Base_Start_IT(&htim7);
      while (!sh_timer) {
	//spin on the timer
      }
      sh_timer = 0;
      HAL_GPIO_WritePin(SH2_GPIO_Port, SH2_Pin, GPIO_PIN_RESET);

      if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ENV_INV(&envelopes[2])) != HAL_OK)
      {
        /* Setting value Error */
        Error_Handler();
      }
      HAL_GPIO_WritePin(SH3_GPIO_Port, SH3_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_AUTORELOAD(&htim7, SH_TIME);
      HAL_TIM_Base_Start_IT(&htim7);
      while (!sh_timer) {
	//spin on the timer
      }
      sh_timer = 0;
      HAL_GPIO_WritePin(SH3_GPIO_Port, SH3_Pin, GPIO_PIN_RESET);


      // Let's do the button and led every ten ticks which should give us ms resolution
      if ((btn_tick % BTN_TICK) == 0) {

	process_btn(&btn, (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_SET) ? 1 : 0);

      	switch (btn.click_state) {
      	  case NONE:
	    seg_change = SEG_NO_CHANGE;
      	    break;
      	  case SINGLE:
	    //cycle through the envelopes
      	    i = (i + 1) % NUM_ENVS;
      	    break;
      	  case DOUBLE:
	    //double click toggles invert of current envelope
      	    envelopes[i].mode = (envelopes[i].mode + 1) % NUM_MODES;
      	    break;
	  case LONG:
	    //This is where I want to check the ADC channels and pick segment shapes based on each third of the range  
	    //Might need to do something to stop doing continuous sampling or continuous update of the envelopes directly
	    //For now only allow adjustment of one segment per long press
	    if (seg_change == SEG_NO_CHANGE) {
	      for (j = 0; j < 4; j++) {
	        if (abs(envelopes[i].adsr[j] - aADCxConvertedData[j]) > CHANGE_THRESH) {
	          seg_change = j;
	        }
	      }
	      led.blink = 0xFF;
	      led.blink_duty = 0x00;
	    } else {

	      if (aADCxConvertedData[seg_change] < BTM_THRD) {
		new_seg = LIN;
	      } else if (aADCxConvertedData[seg_change] < MID_THRD) {
		new_seg = EXP;
	      } else {
		new_seg = LOG;
	      }

	      switch (seg_change) {
		case ATTACK:
		  envelopes[i].acurve = new_seg;
		  break;
		case DECAY:
		  envelopes[i].dcurve = new_seg;
		  break;
		case SUSTAIN:
		  //easter eggs
		  if (new_seg == LOG) {
		    envelopes[i].mode = BOUNCY;
		  } else {
		    envelopes[i].mode = ONESHOT;
		  }
		  break;
		case RELEASE:
		  envelopes[i].rcurve = new_seg;
		  break;
		default:
		  break;
	      }
	      led.blink = 0xFF;
	      led.blink_duty = 0xFF;
	    }
	    break;

      	  default:
      	    break;
      	}

	switch ((btn.click_state == LONG) ? new_seg : i) {
      	  case 0:
      	    led.r = ADC_FS;
      	    led.g = 0;
      	    led.b = TRIG_MODE(&envelopes[i]) ? ADC_FS : 0;
      	    break;
      	  case 1:
      	    led.r = TRIG_MODE(&envelopes[i]) ? ADC_FS : 0;
      	    led.g = ADC_FS;
      	    led.b = 0;
      	    break;
      	  case 2:
      	    led.r = 0;
      	    led.g = TRIG_MODE(&envelopes[i]) ? ADC_FS : 0;
      	    led.b = ADC_FS;
      	    break;
      	  default:
      	    led.r = ADC_FS;
      	    led.g = ADC_FS;
      	    led.b = ADC_FS;
      	}

	if (btn.click_state != LONG) {
	  switch (envelopes[i].mode) {
	    case ONESHOT:
	      led.blink = 0xFF;
	      led.blink_duty = 0xFF;
	      break;
	    case INV_ONESHOT:
	      led.blink = BLINK_RATE;
      	      led.blink_duty = 0.5 * led.blink;
	      break;
	    case TRIG:
	      led.blink = 0xFF;
	      led.blink_duty = 0xFF;
	      break;
	    case INV_TRIG:
	      led.blink = BLINK_RATE;
      	      led.blink_duty = 0.5 * led.blink;
	      break;
	    case LOOP:
	      led.blink = BLINK_RATE;
      	      led.blink_duty = 0.75 * led.blink;
	      break;
	    case INV_LOOP:
	      led.blink = BLINK_RATE;
      	      led.blink_duty = 0.25 * led.blink;
	      break;
	    case BOUNCY:
	      led.blink = 0xFF;
	      led.blink_duty = 0xFF;
	      led.r = envelopes[i].val;
      	      led.g = envelopes[i].val;
      	      led.b = envelopes[i].val;
	      break;
      	  }
	}

#ifdef SEEENVS	
	/*
      	 * Testing - visualize the envs
      	 */
	if (GATE(&envelopes[0], (trigger_pins[0] == GATE_HIGH) ? 1 : 0) || \
	    GATE(&envelopes[1], (trigger_pins[1] == GATE_HIGH) ? 1 : 0) || \
	    GATE(&envelopes[2], (trigger_pins[2] == GATE_HIGH) ? 1 : 0)) { 
	  led.r = ENV_INV(&envelopes[0]);
      	  led.g = ENV_INV(&envelopes[1]);
      	  led.b = ENV_INV(&envelopes[2]);
	}
#endif
      	
      	process_led(&led);

	btn_tick = 1;
      } else {
	btn_tick++;
      }

      tick++;
      last_tick = tick;
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) !=  HAL_OK)
  {
    Error_Handler();
  }
  
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = TIM1_PSC;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = TIM1_PER;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

   /* TIM6 is clocked by APB1 Timer clocks, currently configured at 80MHz
    * With the Prescaler set to 363 the counter clock should be running at something like 220.385k
    * Divide that by 22 and we're around 10kHz for the triggers
    * See TIM6_PSC and TIM6_PER in main.h
    */ 

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = TIM6_PSC;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = TIM6_PER;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  // This should be a time base with a period of 1uS - for driving the Sample and Hold signals

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = TIM7_PSC;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = SH_TIME;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SH2_Pin|SH3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|SH1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TRIG1_Pin */
  GPIO_InitStruct.Pin = TRIG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TRIG1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG2_Pin TRIG3_Pin */
  GPIO_InitStruct.Pin = TRIG2_Pin|TRIG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SH2_Pin SH3_Pin */
  GPIO_InitStruct.Pin = SH2_Pin|SH3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin SH1_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|SH1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS)) {
    tick = 1;
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7) {
    HAL_TIM_Base_Stop_IT(htim);
    sh_timer++;
  }
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
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
