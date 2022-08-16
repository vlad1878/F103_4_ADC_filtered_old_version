/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "lcd1602_i2c_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define adc_buf_len 6
#define r1 6616.0f
#define r2 3300.0f
#define u_ref 3.30f
#define adc_max 4095.0f
#define koef_volt 0.17f
#define zero_off_volt 0.17f
#define middle_off_volt 0.02f
#define middle_volt_threashold 4.80f
#define low_threashold_battery 7.60f
#define high_threashold_battery 8.50f
#define medium_threashold_battery 7.80f
#define min_pressure_adc_1 (float)-100.0f
#define max_pressure_adc_1 (float)500.0f
#define min_pressure_adc_2 (float)-100.0f
#define max_pressure_adc_2 (float)500.0f
#define min_pressure_adc_3 (float)-100.0f
#define max_pressure_adc_3 (float)500.0f
#define min_pressure_adc_4 (float)-100.0f
#define max_pressure_adc_4 (float)500.0f
#define koeficient_adc_1 (float)4.0f
#define koeficient_adc_2 (float)4.0f
#define koeficient_adc_3 (float)4.0f
#define koeficient_adc_4 (float)4.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern char lcd1602_tx_buffer[40];
uint8_t procent = 0;
uint8_t mode_lcd = 0;
unsigned long t_lcd = 0;

uint32_t adc_buffer[6];
uint32_t adc_value_1 = 0;
uint32_t adc_value_2 = 0;
uint32_t adc_value_3 = 0;
uint32_t adc_value_4 = 0;
uint32_t adc_value_5 = 0;
uint32_t adc_value_6 = 0;

float adc_value_1_filt = 0;
float adc_value_2_filt = 0;
float adc_value_3_filt = 0;
float adc_value_4_filt = 0;
float adc_value_5_filt = 0;
float adc_value_6_filt = 0;

float adc_volt_1 = 0.0f;
float adc_volt_2 = 0.0f;
float adc_volt_3 = 0.0f;
float adc_volt_4 = 0.0f;

float filtered_buf_1[100] = { 0, };
uint8_t filtered_buf_1_index = 0;
float summ_1 = 0.0f;
float filtered_buf_2[100] = { 0, };
uint8_t filtered_buf_2_index = 0;
float summ_2 = 0.0f;
float filtered_buf_5[100] = { 0, };
uint8_t filtered_buf_5_index = 0;
float summ_5 = 0.0f;
float filtered_buf_3[100] = { 0, };
uint8_t filtered_buf_3_index = 0;
float summ_3 = 0.0f;
float filtered_buf_4[100] = { 0, };
uint8_t filtered_buf_4_index = 0;
float summ_4 = 0.0f;
bool adc_flag = 0;
bool clear_flag = 0;

uint8_t filtration_step = 0;

float voltage = 0.0f;
float volt_var = 0.0f;

volatile bool button_flag = 0;
unsigned long t_led = 0;
unsigned long t_clear = 0;

float PR201 = 0.0f;
float PR202 = 0.0f;
float PR203 = 0.0f;
float PR204 = 0.0f;

uint8_t led_duty = 0;
unsigned long t_led_duty = 0;
bool mode_led = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void get_adc(void);
void filter_adc(void);
void analog_pin_voltage(void);
void print_lcd(void);
void battery_voltage(void);
float map(float x, float in_min, float in_max, float out_min, float out_max);
void adc_1_to_val(void);
void adc_2_to_val(void);
void adc_3_to_val(void);
void adc_4_to_val(void);
void battery_capacity(void);
void disp_clear(void);
void led_duty_f(void);
void print_all_adc(void);
void print_first_adc(void);
void print_second_adc(void);
void print_third_adc(void);
void print_fourth_adc(void);
void print_fifth_adc(void);

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
	t_lcd = HAL_GetTick();
	t_led = HAL_GetTick();
	t_clear = HAL_GetTick();
	t_led_duty = HAL_GetTick();

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_Delay(100);
	lcd1602_Init();
	while (1) {
		lcd1602_SetCursor(0, 0);
		sprintf(lcd1602_tx_buffer, "   Initialization");
		lcd1602_Print_text(lcd1602_tx_buffer);
		lcd1602_SetCursor(0, 1);
		sprintf(lcd1602_tx_buffer, " --- %d percent ---", procent);
		lcd1602_Print_text(lcd1602_tx_buffer);
		HAL_Delay(10);
		procent++;
		if (procent == 100) {
			lcd1602_SetCursor(0, 0);
			sprintf(lcd1602_tx_buffer, "   Initialization");
			lcd1602_Print_text(lcd1602_tx_buffer);
			lcd1602_SetCursor(0, 1);
			sprintf(lcd1602_tx_buffer, " --- %d percent ---", procent);
			lcd1602_Print_text(lcd1602_tx_buffer);
			lcd1602_SetCursor(0, 2);
			sprintf(lcd1602_tx_buffer, "       Ready     ");
			lcd1602_Print_text(lcd1602_tx_buffer);
			lcd1602_Clean_Text();
			break;
		}
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		get_adc();
		filter_adc();
		adc_1_to_val();
		adc_2_to_val();
		adc_3_to_val();
		adc_4_to_val();
		battery_voltage();
		analog_pin_voltage();
		print_lcd();
		disp_clear();
		battery_capacity();
		led_duty_f();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void get_adc() {
	HAL_ADC_Start_DMA(&hadc1, adc_buffer, adc_buf_len);
	adc_value_1 = adc_buffer[0];
	adc_value_2 = adc_buffer[1];
	adc_value_3 = adc_buffer[2];
	adc_value_4 = adc_buffer[3];
	adc_value_5 = adc_buffer[4];
	adc_value_6 = adc_buffer[5];
	adc_flag = 1;
}

void filter_adc() {
	if (adc_flag) {
		adc_flag = 0;
		if (filtered_buf_1_index >= 0 && filtration_step == 0) {
			filtered_buf_1[filtered_buf_1_index] = (float) adc_value_1;
			if (filtered_buf_1_index == 99) {
				for (int i = 0; i <= 99; i++) {
					summ_1 = summ_1 + filtered_buf_1[i];

				}
				adc_value_1_filt = summ_1 / 100;
				summ_1 = 0.0f;
				filtered_buf_1_index = 0;
			} else {
				filtered_buf_1_index++;
			}
		}
		if (filtered_buf_2_index >= 0 && filtration_step == 0) {
			filtered_buf_2[filtered_buf_2_index] = (float) adc_value_2;
			if (filtered_buf_2_index == 99) {
				for (int i = 0; i <= 99; i++) {
					summ_2 = summ_2 + filtered_buf_2[i];

				}
				adc_value_2_filt = summ_2 / 100;
				summ_2 = 0.0f;
				filtered_buf_2_index = 0;
			} else {
				filtered_buf_2_index++;
			}
		}
		if (filtered_buf_5_index >= 0 && filtration_step == 0) {
			filtered_buf_5[filtered_buf_5_index] = (float) adc_value_5;
			if (filtered_buf_5_index == 99) {
				for (int i = 0; i <= 99; i++) {
					summ_5 = summ_5 + filtered_buf_5[i];

				}
				adc_value_5_filt = summ_5 / 100;
				summ_5 = 0.0f;
				filtered_buf_5_index = 0;
			} else {
				filtered_buf_5_index++;
			}
		}
		if (filtered_buf_3_index >= 0 && filtration_step == 0) {
			filtered_buf_3[filtered_buf_3_index] = (float) adc_value_3;
			if (filtered_buf_3_index == 99) {
				for (int i = 0; i <= 99; i++) {
					summ_3 = summ_3 + filtered_buf_3[i];

				}
				adc_value_3_filt = summ_3 / 100;
				summ_3 = 0.0f;
				filtered_buf_3_index = 0;
			} else {
				filtered_buf_3_index++;
			}
		}
		if (filtered_buf_4_index >= 0 && filtration_step == 0) {
			filtered_buf_4[filtered_buf_4_index] = (float) adc_value_4;
			if (filtered_buf_4_index == 99) {
				for (int i = 0; i <= 99; i++) {
					summ_4 = summ_4 + filtered_buf_4[i];

				}
				adc_value_4_filt = summ_4 / 100;
				summ_4 = 0.0f;
				filtered_buf_4_index = 0;
			} else {
				filtered_buf_4_index++;
			}
		}
	}
}

void print_lcd() {
	if (HAL_GetTick() - t_lcd > 300) {
		switch (mode_lcd) {
		case 0:
			print_all_adc();
			break;
		case 1:
			print_first_adc();
			break;
		case 2:
			print_second_adc();
			break;
		case 3:
			print_third_adc();
			break;
		case 4:
			print_fourth_adc();
			break;
		case 5:
			print_fifth_adc();
			break;
		}
	}

}

void battery_voltage() {
	volt_var = ((float) adc_value_5_filt * u_ref) / adc_max;
	voltage = (volt_var / (r2 / (r1 + r2))) + koef_volt;
	if (voltage <= zero_off_volt) {
		voltage = 0.0f;
	} else if (voltage >= middle_volt_threashold) {
		voltage = voltage + middle_off_volt;
	}
}

void analog_pin_voltage() {
	adc_volt_1 = (float) (adc_value_1_filt * u_ref) / adc_max;
	adc_volt_2 = (float) (adc_value_2_filt * u_ref) / adc_max;
	adc_volt_3 = (float) (adc_value_3_filt * u_ref) / adc_max;
	adc_volt_4 = (float) (adc_value_4_filt * u_ref) / adc_max;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		if (mode_lcd < 5) {
			mode_lcd = mode_lcd + 1;
			clear_flag = 1;
		} else {
			mode_lcd = 0;
			clear_flag = 1;
		}

	}
}

void disp_clear() {
	if (clear_flag) {
		lcd1602_Clean_Text();
		clear_flag = 0;
	}
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void adc_1_to_val() {
	PR201 = map(adc_value_1_filt, 0, adc_max, min_pressure_adc_1,
	max_pressure_adc_1);
}

void adc_2_to_val() {
	PR202 = map(adc_value_2_filt, 0, adc_max, min_pressure_adc_2,
	max_pressure_adc_2);
}

void adc_3_to_val() {
	PR203 = map(adc_value_3_filt, 0, adc_max, min_pressure_adc_3,
	max_pressure_adc_3);
}

void adc_4_to_val() {
	PR204 = map(adc_value_4_filt, 0, adc_max, min_pressure_adc_4,
	max_pressure_adc_4);
}

void battery_capacity() {
	if ((voltage < high_threashold_battery)
			&& (voltage > medium_threashold_battery)) {
		GPIOC->BSRR = led_green_Pin;
		GPIOC->BSRR = led_yelow_Pin;
		GPIOC->BSRR = led_red_Pin;

	}

	else if ((voltage < medium_threashold_battery)
			&& (voltage > low_threashold_battery)) {
		GPIOC->BSRR = (uint32_t) led_green_Pin << 16;
		GPIOC->BSRR = led_yelow_Pin;
		GPIOC->BSRR = led_red_Pin;
	} else if (voltage < low_threashold_battery) {
		if (HAL_GetTick() - t_led > 100) {
			t_led = HAL_GetTick();
			GPIOC->BSRR = (uint32_t) led_green_Pin << 16;
			GPIOC->BSRR = (uint32_t) led_yelow_Pin << 16;
			HAL_GPIO_TogglePin(GPIOC, led_red_Pin);
		}
	}
}

void led_duty_f() {
	if ((HAL_GetTick() - t_led_duty > 25) && (mode_led == 0)) {
		t_led_duty = HAL_GetTick();
		TIM2->CCR2 = led_duty;
		led_duty = led_duty + 1;
		if (led_duty == 99) {
			mode_led = 1;
		}
	} else if ((HAL_GetTick() - t_led_duty > 25) && (mode_led == 1)) {
		t_led_duty = HAL_GetTick();
		TIM2->CCR2 = led_duty;
		led_duty = led_duty - 1;
		if (led_duty == 0) {
			mode_led = 0;
		}

	}
}

void print_all_adc() {
	t_lcd = HAL_GetTick();
	if((int32_t)PR201 > (-10))
	{
		lcd1602_SetCursor(0, 0);
		sprintf(lcd1602_tx_buffer, "PR201 %.0f mBar     ", (PR201 + koeficient_adc_1));
		lcd1602_Print_text(lcd1602_tx_buffer);
	}
	else
	{
		lcd1602_SetCursor(0, 0);
		sprintf(lcd1602_tx_buffer, "PR201 %.0f mBar     ", PR201);
		lcd1602_Print_text(lcd1602_tx_buffer);
	}
	if((int32_t)PR202 > (-10))
	{
		lcd1602_SetCursor(0, 1);
		sprintf(lcd1602_tx_buffer, "PR202 %.0f mBar     ", (PR202 + koeficient_adc_2));
		lcd1602_Print_text(lcd1602_tx_buffer);
	}
	else
	{
		lcd1602_SetCursor(0, 1);
		sprintf(lcd1602_tx_buffer, "PR202 %.0f mBar     ", PR202);
		lcd1602_Print_text(lcd1602_tx_buffer);
	}
	if((int32_t)PR203 > (-10))
	{
		lcd1602_SetCursor(0, 2);
		sprintf(lcd1602_tx_buffer, "PR203 %.0f mBar     ", (PR203 + koeficient_adc_3));
		lcd1602_Print_text(lcd1602_tx_buffer);
	}
	else
	{
		lcd1602_SetCursor(0, 2);
		sprintf(lcd1602_tx_buffer, "PR203 %.0f mBar     ", PR203);
		lcd1602_Print_text(lcd1602_tx_buffer);
	}
	if((int32_t)PR204 > (-10))
	{
		lcd1602_SetCursor(0, 3);
		sprintf(lcd1602_tx_buffer, "PR204 %.0f mBar    ", (PR204 + koeficient_adc_4));
		lcd1602_Print_text(lcd1602_tx_buffer);
	}
	else
	{
		lcd1602_SetCursor(0, 3);
		sprintf(lcd1602_tx_buffer, "PR204 %.0f mBar    ", PR204);
		lcd1602_Print_text(lcd1602_tx_buffer);
	}

}

void print_first_adc() {
	t_lcd = HAL_GetTick();
	lcd1602_SetCursor(0, 0);
	sprintf(lcd1602_tx_buffer, "adc 1 is %.2f      ", adc_value_1_filt);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 1);
	sprintf(lcd1602_tx_buffer, "adc 1 volt is %.2f ", adc_volt_1);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 2);
	sprintf(lcd1602_tx_buffer, "PR201  %.2f kPa    ", PR201);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 3);
	sprintf(lcd1602_tx_buffer, "                   ");
	lcd1602_Print_text(lcd1602_tx_buffer);
}

void print_second_adc() {
	t_lcd = HAL_GetTick();
	lcd1602_SetCursor(0, 0);
	sprintf(lcd1602_tx_buffer, "adc 2 is %.2f       ", adc_value_2_filt);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 1);
	sprintf(lcd1602_tx_buffer, "adc 2 volt is %.2f  ", adc_volt_2);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 2);
	sprintf(lcd1602_tx_buffer, "PR202 %.2f kPa      ", PR202);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 3);
	sprintf(lcd1602_tx_buffer, "                   ");
	lcd1602_Print_text(lcd1602_tx_buffer);
}

void print_third_adc() {
	t_lcd = HAL_GetTick();
	lcd1602_SetCursor(0, 0);
	sprintf(lcd1602_tx_buffer, "adc 3 is %.2f       ", adc_value_3_filt);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 1);
	sprintf(lcd1602_tx_buffer, "adc 3 volt is %.2f  ", adc_volt_3);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 2);
	sprintf(lcd1602_tx_buffer, "PR203 %.2f kPa      ", PR203);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 3);
	sprintf(lcd1602_tx_buffer, "                    ");
	lcd1602_Print_text(lcd1602_tx_buffer);
}

void print_fourth_adc() {
	t_lcd = HAL_GetTick();
	lcd1602_SetCursor(0, 0);
	sprintf(lcd1602_tx_buffer, "adc 4 is %.2f       ", adc_value_4_filt);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 1);
	sprintf(lcd1602_tx_buffer, "adc 4 volt is %.2f  ", adc_volt_4);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 2);
	sprintf(lcd1602_tx_buffer, "TR101 %.1f celsius  ", PR204);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 3);
	sprintf(lcd1602_tx_buffer, "                   ");
	lcd1602_Print_text(lcd1602_tx_buffer);
}

void print_fifth_adc() {
	t_lcd = HAL_GetTick();
	lcd1602_SetCursor(0, 0);
	sprintf(lcd1602_tx_buffer, "adc 5 is %.2f      ", adc_value_5_filt);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 1);
	sprintf(lcd1602_tx_buffer, "Battery volt  %.2f", voltage);
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 2);
	sprintf(lcd1602_tx_buffer, "                  ");
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 3);
	sprintf(lcd1602_tx_buffer, "                  ");
	lcd1602_Print_text(lcd1602_tx_buffer);
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
	while (1) {
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
