/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h> // Needed for the sprintf function
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDR 0x27 // I2C address of the PCF8574
#define RS_BIT 0      // Register select bit
#define EN_BIT 2      // Enable bit
#define BL_BIT 3      // Back_light bit
#define D4_BIT 4      // Data 4 bit
#define D5_BIT 5      // Data 5 bit
#define D6_BIT 6      // Data 6 bit
#define D7_BIT 7      // Data 7 bit
#define LCD_ROWS 2    // Number of rows on the LCD
#define LCD_COLS 16   // Number of columns on the LCD

#define BLINKLEN 30        // Amount of time each LED will shine and buzzer will sound
#define DEBOUNCE_TIME 300  // Amount of time before a button press will be recognized again
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint16_t timeSig = 2;         // Time signature (2 = 2/4, 3 = 3/4, 4 = 4/4)
uint16_t bpm = 100;           // Beats per minute
uint32_t bpmms = 1666;        // Beats per minute in milliseconds
uint16_t go = 1;              // Start and stop variable controlled by GO button
uint8_t backlight_state = 1;  // Back_light is automatically turned on
uint32_t lastTime = 0;        // Last time the GO button was pressed
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void getBPM(); // Prototype to get the bpm from the potentiometer
void time24(); // Prototype to flash lights and sound buzzer in a 2/4 pattern
void time34(); // Prototype to flash lights and sound buzzer in a 3/4 pattern
void time44(); // Prototype to flash lights and sound buzzer in a 4/4 pattern
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Sends individual nibbles of information to the LDC to be printed (displayed)
void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
	uint8_t data = nibble << D4_BIT;
	data |= rs << RS_BIT;
	data |= backlight_state << BL_BIT; // Include backlight state in data
	data |= 1 << EN_BIT;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
	HAL_Delay(1);
	data &= ~(1 << EN_BIT);
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
}

// Sends a command to the display
void lcd_send_cmd(uint8_t cmd) {
	uint8_t upper_nibble = cmd >> 4;
	uint8_t lower_nibble = cmd & 0x0F;
	lcd_write_nibble(upper_nibble, 0);
	lcd_write_nibble(lower_nibble, 0);
	if (cmd == 0x01 || cmd == 0x02) {
		HAL_Delay(2);
	}
}

// Splits data from bytes to nibbles
void lcd_send_data(uint8_t data) {
	uint8_t upper_nibble = data >> 4;
	uint8_t lower_nibble = data & 0x0F;
	lcd_write_nibble(upper_nibble, 1);
	lcd_write_nibble(lower_nibble, 1);
}

// Initializes the display
void lcd_init() {
	HAL_Delay(50);
	lcd_write_nibble(0x03, 0);
	HAL_Delay(5);
	lcd_write_nibble(0x03, 0);
	HAL_Delay(1);
	lcd_write_nibble(0x03, 0);
	HAL_Delay(1);
	lcd_write_nibble(0x02, 0);
	lcd_send_cmd(0x28);
	lcd_send_cmd(0x0C);
	lcd_send_cmd(0x06);
	lcd_send_cmd(0x01);
	HAL_Delay(2);
}

// Prints a string to the display (in chars)
void lcd_write_string(char *str) {
	while (*str) {
		lcd_send_data(*str++);
	}
}

// Moves cursor on display
void lcd_set_cursor(uint8_t row, uint8_t column) {
	uint8_t address;
	switch (row) {
		case 0:
			address = 0x00;
			break;
		case 1:
			address = 0x40;
			break;
		default:
			address = 0x00;
	}
	address += column;
	lcd_send_cmd(0x80 | address);
}

// Clear display
void lcd_clear(void) {
	lcd_send_cmd(0x01);
	HAL_Delay(2);
}

// Enable or disable back_light
void lcd_backlight(uint8_t state) {
	if (state) {
		backlight_state = 1;
	} else {
		backlight_state = 0;
	}
}
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // I2C pull-up resistors
  GPIOB->PUPDR |= 0b01 << (8*2);
  GPIOB->PUPDR |= 0b01 << (9*2);

  // Initialize the LCD
  lcd_init();
  lcd_backlight(1); // Turn on backlight

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (go == 1) { // If system is on
		  getBPM();  // Get the bpm value from the potentiometer

		  lcd_clear();                    // Clear the screen
		  char pbpm[15];                  // Create an empty string to display bpm
		  sprintf(pbpm, "BPM: %d", bpm);  // Insert the actual bpm to the string
		  lcd_write_string(pbpm);         // Write the string to the LCD

		  // Check for the time signature to call the correct function
		  if(timeSig == 2) {
			  time24(); // Run 2/4 time function
		  } else if(timeSig == 3) {
			  time34(); // Run 3/4 time function
		  } else if(timeSig == 4) {
			  time44(); // Run 4/4 time function
		  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
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
  hi2c1.Init.Timing = 0x10D19CE4;
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
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED3_Pin|LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GO_Pin T24_Pin */
  GPIO_InitStruct.Pin = GO_Pin|T24_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED7_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : T34_Pin */
  GPIO_InitStruct.Pin = T34_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(T34_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED5_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : T44_Pin */
  GPIO_InitStruct.Pin = T44_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(T44_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == T24_Pin) {        // Check if it was 2/4 time
		timeSig = 2; // Set the time signature to 2/4
	} else if (GPIO_Pin == T34_Pin) { // Check if it was 3/4 time
		timeSig = 3; // Set the time signature to 3/4
	} else if (GPIO_Pin == T44_Pin) { // Check if it was 4/4 time
		timeSig = 4; // Set the time signature to 4/4
	} else if (GPIO_Pin == GO_Pin) {  // Check if it was on/off button

		// Prevent multiple presses from button bouncing
		uint32_t currentTime = HAL_GetTick(); // Get the current time
		if((currentTime-lastTime) > DEBOUNCE_TIME) { // If the current time is within the DEBOUNCE_TIME from the last time the button was pressed
			// Toggle the on/off setting "go"
			if(go == 1) {
				go = 0;
			} else {
				go = 1;
			}

			// Set the last time to the current time
			lastTime = currentTime;
		}
	}
}

void getBPM() {
	  int ADC_RANGE = 4095;  // Range provided by ADC from the potentiometer
	  HAL_ADC_Start(&hadc1); // Start ADC Conversion
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for ADC conversion to complete
	  uint16_t bpmMeasurement = HAL_ADC_GetValue(&hadc1); // Read ADC value

	  // Convert from ADC range to a range of 0 to 300
	  float unitBPM = ((float) bpmMeasurement) / ADC_RANGE;
	  bpm = (uint16_t) (unitBPM * 300);

	  // Prevent the bpm from being lower than 20 (desired range is 20 to 300)
	  if(bpm < 20){
		  bpm = 20;
	  }

	  // Sets the bpm variable in milliseconds for the delay
	  bpmms = (uint32_t) ((60 * 1000) / bpm);
}

void time24 () {
	TIM3->ARR = 22; // Sets the high pitch for the first note
	TIM3->CCR2 = 11; // Sets the off-value for PWM to half of the max to simulate a perfect sine wave
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Starts the timer (enables the buzzer)
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // Turns on LED1
	HAL_Delay(BLINKLEN); // Waits before turning off LED and buzzer
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); // Turns off timer (buzzer)
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); // Turns off LED1
	HAL_Delay(bpmms/2-BLINKLEN); // Waits for half the length of the beats per minute (minus the time the light was on)

	TIM3->ARR = 44; // Sets the low pitch for the next note
	TIM3->CCR2 = 22; // Sets the off-value for PWM to half of the max to simulate a perfect sine wave
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Starts the timer (enables the buzzer)
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); // Turns on LED3
	HAL_Delay(BLINKLEN); // Waits before turning off LED and buzzer
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); // Turns off timer (buzzer)
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); // Turns off LED3
	HAL_Delay(bpmms/2-BLINKLEN); // Waits for half the length of the beats per minute (minus the time the light was on)
}

void time34 () {
	TIM3->ARR = 22; // Sets the high pitch for the first note
	TIM3->CCR2 = 11; // Sets the off-value for PWM to half of the max to simulate a perfect sine wave
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Starts the timer (enables the buzzer)
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // Turns on LED1
	HAL_Delay(BLINKLEN); // Waits before turning off LED and buzzer
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); // Turns off timer (buzzer)
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); // Turns off LED1
	HAL_Delay(bpmms/3-BLINKLEN); // Waits for a third of the length of the beats per minute (minus the time the light was on)

	TIM3->ARR = 44; // Sets the low pitch for the last notes
	TIM3->CCR2 = 22; // Sets the off-value for PWM to half of the max to simulate a perfect sine wave
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Starts the timer (enables the buzzer)
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); // Turns on LED3
	HAL_Delay(BLINKLEN); // Waits before turning off LED and buzzer
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); // Turns off timer (buzzer)
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); // Turns off LED3
	HAL_Delay(bpmms/3-BLINKLEN); // Waits for a third of the length of the beats per minute (minus the time the light was on)

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Starts the timer (enables the buzzer)
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET); // Turns on LED5
	HAL_Delay(BLINKLEN); // Waits before turning off LED and buzzer
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); // Turns off timer (buzzer)
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET); // Turns off LED5
	HAL_Delay(bpmms/3-BLINKLEN); // Waits for a third of the length of the beats per minute (minus the time the light was on)
}

void time44 () {
	TIM3->ARR = 22; // Sets the high pitch for the first note
	TIM3->CCR2 = 11; // Sets the off-value for PWM to half of the max to simulate a perfect sine wave
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Starts the timer (enables the buzzer)
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // Turns on LED1
	HAL_Delay(BLINKLEN); // Waits before turning off LED and buzzer
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); // Turns off timer (buzzer)
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); // Turns off LED1
	HAL_Delay(bpmms/4-BLINKLEN); // Waits for a fourth of the length of the beats per minute (minus the time the light was on)

	TIM3->ARR = 44; // Sets the low pitch for the last notes
	TIM3->CCR2 = 22; // Sets the off-value for PWM to half of the max to simulate a perfect sine wave
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Starts the timer (enables the buzzer)
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); // Turns on LED3
	HAL_Delay(BLINKLEN); // Waits before turning off LED and buzzer
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); // Turns off timer (buzzer)
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); // Turns off LED3
	HAL_Delay(bpmms/4-BLINKLEN); // Waits for a fourth of the length of the beats per minute (minus the time the light was on)

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Starts the timer (enables the buzzer)
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET); // Turns on LED5
	HAL_Delay(BLINKLEN); // Waits before turning off LED and buzzer
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); // Turns off timer (buzzer)
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET); // Turns off LED5
	HAL_Delay(bpmms/4-BLINKLEN); // Waits for a fourth of the length of the beats per minute (minus the time the light was on)

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Starts the timer (enables the buzzer)
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET); // Turns on LED7
	HAL_Delay(BLINKLEN); // Waits before turning off LED and buzzer
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); // Turns off timer (buzzer)
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET); // Turns off LED7
	HAL_Delay(bpmms/4-BLINKLEN); // Waits for a fourth of the length of the beats per minute (minus the time the light was on)
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
