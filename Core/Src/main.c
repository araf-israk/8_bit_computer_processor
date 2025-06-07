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
#include <stdio.h>
#include <stdlib.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "eeprom_rw.h"

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

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

uint8_t read_io_data = 0;
uint16_t read_io_addr = 0;


char dataText[] = "FFFF FF FF FF FF";
char welcomeText[] = "EEPROM BURNER";
char arafText[] = "ARAF";
char retVal;
uint16_t display_addr = 0;;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum control_bit_list {
	J,
	PCO,
	PCE,
	OE,
	BO,
	BI,
	SUB,
	EO,
	AO,
	AI,
	II,
	IO,
	RO,
	RI,
	MI,
	HLT
}control_bit_list_t;



typedef struct instruction{
	char 	mnemonic[3];
	uint8_t id;
	uint8_t step;
	uint8_t step_bit_count_list[10];
	control_bit_list_t control_bits[10][10];
}instruction_t;

typedef struct control_unit_lut{
	uint8_t addresses[200];
	uint16_t set_control_bits[200];
}control_unit_lut_t;

void instruction_struct_setup(instruction_t temp, instruction_t *target){
	  for(int i = 0; i < sizeof(temp.mnemonic); i++){
		  target->mnemonic[i] = temp.mnemonic[i];
	  }
	  target->id = temp.id;
	  target->step = temp.step;
	  for(int i = 0; i < temp.step; i++){
		  target->step_bit_count_list[i] = temp.step_bit_count_list[i];
		  for(int j = 0; j < temp.step_bit_count_list[i]; j++){
			  target->control_bits[i][j] = temp.control_bits[i][j];
		  }
	  }
}


uint8_t generate_eeprom_ISA_CU_table(instruction_t **instruction_list, control_unit_lut_t *output_CU_LUT, uint8_t total_instr){
	uint8_t _lines = 0;
	for(uint8_t i = 0; i < total_instr; i++){
		for(uint8_t j = 0; j < instruction_list[i]->step; j++){
			uint8_t _addr = 0;
			uint16_t _cbits = 0;
			_addr |= (instruction_list[i]->id << 3) | ((uint8_t)j);

			for(uint8_t k = 0; k < instruction_list[i]->step_bit_count_list[j]; k++){
				_cbits |= (1 << (instruction_list[i]->control_bits[j][k]));
			}
			output_CU_LUT->addresses[_lines] = _addr;
			output_CU_LUT->set_control_bits[_lines] = _cbits;
			_lines++;
		}
	}
	return _lines;
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
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */




  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(15, 2);
  ssd1306_WriteString(welcomeText, Font_7x10, White);
  ssd1306_SetCursor(40, 14);
  ssd1306_WriteString(arafText, Font_7x10, White);
  ssd1306_UpdateScreen();
  HAL_Delay(500);


  HAL_TIM_Base_Start(&htim1);
  printf("  ARAF  EEPROM  BURNER  \n\n");

  //SDP_Disable();

  //erase_eeprom();

  uint8_t data[] = {0x01, 0x4f, 0x12, 0x06, 0x4c, 0x24, 0x20, 0x0f, 0x00, 0x04, 0x08, 0x60, 0x31, 0x42, 0x30, 0x38};
  uint8_t data[] = {0x7e, 0x30, 0x6d, 0x79, 0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b, 0x77, 0x1f, 0x4e, 0x3d, 0x4f, 0x47};
  int _temp = 0;
  uint8_t digits[] = {0x7e, 0x30, 0x6d, 0x79, 0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b};

  printf("\nProgramming ones place.\n\n");
  for(int value = 0; value <= 255; value += 1){
	  _temp += write_eeprom_addr(value, digits[value % 10]);
  }
  printf("\nProgramming tens place.\n\n");
  for(int value = 0; value <= 255; value += 1){
	  _temp += write_eeprom_addr(value + 256, digits[(value / 10) % 10]);
  }
  printf("\nProgramming hundreds place.\n\n");
  for(int value = 0; value <= 255; value += 1){
	  _temp += write_eeprom_addr(value + 512, digits[(value / 100) % 10]);
  }
  printf("\nProgramming sign place.\n\n");
  for(int value = 0; value <= 255; value += 1){
	  _temp += write_eeprom_addr(value + 768, 0);
  }
  printf("\nProgramming ones place (twos complement).\n\n");
  for(int value = -128; value <= 127; value += 1){
	  _temp += write_eeprom_addr((uint8_t)value + 1024, digits[abs(value) % 10]);
  }
  printf("\nProgramming tens place (twos complement).\n\n");
  for(int value = -128; value <= 127; value += 1){
	  _temp += write_eeprom_addr((uint8_t)value + 1280, digits[abs(value / 10) % 10]);
  }
  printf("\nProgramming hundreds place (twos complement).\n\n");
  for(int value = -128; value <= 127; value += 1){
	  _temp += write_eeprom_addr((uint8_t)value + 1536, digits[abs(value / 100) % 10]);
  }
  printf("\nProgramming sign place (twos complement).\n\n");
  for(int value = -128; value <= 127; value += 1){
	  if(value < 0){
		  _temp += write_eeprom_addr((uint8_t)value + 1792, 0x01);
	  } else {
		  _temp += write_eeprom_addr((uint8_t)value + 1792, 0);
	  }
  }


#define PROGRAM_EEPROM EEPROM_LSB


  printf("  Control Unit Decoder  \n\n");
  /* | HLT | MI | RI | RO | IO | II | AI | AO | EO | SUB | BI | BO | OE | PCE | PCO | J | */
  instruction_t *zro = (instruction_t*)malloc(sizeof(instruction_t));
  instruction_t temp1 = {"zro", 0b0000, 2, {2, 3}, {{MI, PCO}, {RO, II, PCE}}};
  instruction_struct_setup(temp1, zro);


  instruction_t *lda = (instruction_t*)malloc(sizeof(instruction_t));
  instruction_t temp2 = {"lda", 0b0001, 5, {2, 3, 2, 2, 0}, {{MI, PCO}, {RO, II, PCE}, {MI, IO}, {RO, AI}, {0}}};
  instruction_struct_setup(temp2, lda);


  instruction_t *add = (instruction_t*)malloc(sizeof(instruction_t));
  instruction_t temp3 = {"add", 0b0010, 5, {2, 3, 2, 2, 2}, {{MI, PCO}, {RO, II, PCE}, {MI, IO}, {RO, BI}, {AI, EO}}};
  instruction_struct_setup(temp3, add);


  instruction_t *out = (instruction_t*)malloc(sizeof(instruction_t));
  instruction_t temp4 = {"out", 0b1110, 5, {2, 3, 2, 0, 0}, {{MI, PCO}, {RO, II, PCE}, {AO, OE}, {0}, {0}}};
  instruction_struct_setup(temp4, out);


  instruction_t *hlt = (instruction_t*)malloc(sizeof(instruction_t));
  instruction_t temp5 = {"hlt", 0b1111, 5, {2, 3, 1, 0, 0}, {{MI, PCO}, {RO, II, PCE}, {HLT}, {0}, {0}}};
  instruction_struct_setup(temp5, hlt);

  instruction_t *sub = (instruction_t*)malloc(sizeof(instruction_t));
  instruction_t temp6 = {"sub", 0b0011, 5, {2, 3, 2, 2, 3}, {{MI, PCO}, {RO, II, PCE}, {MI, IO}, {RO, BI}, {AI, EO, SUB}}};
  instruction_struct_setup(temp6, sub);

  instruction_t *sta = (instruction_t*)malloc(sizeof(instruction_t));
  instruction_t temp7 = {"sta", 0b0100, 5, {2, 3, 2, 2, 0}, {{MI, PCO}, {RO, II, PCE}, {MI, IO}, {AO, RI}, {0}}};
  instruction_struct_setup(temp7, sta);

  instruction_t *ldi = (instruction_t*)malloc(sizeof(instruction_t));
  instruction_t temp8 = {"ldi", 0b0101, 5, {2, 3, 2, 0, 0}, {{MI, PCO}, {RO, II, PCE}, {AI, IO}, {0}, {0}}};
  instruction_struct_setup(temp8, ldi);

  instruction_t *jmp = (instruction_t*)malloc(sizeof(instruction_t));
  instruction_t temp9 = {"jmp", 0b0110, 5, {2, 3, 2, 0, 0}, {{MI, PCO}, {RO, II, PCE}, {J, IO}, {0}, {0}}};
  instruction_struct_setup(temp9, jmp);

  uint8_t intr_list = 9;
  instruction_t **list = (instruction_t**)malloc(intr_list*sizeof(instruction_t*));
  list[0] = zro;
  list[1] = lda;
  list[2] = add;
  list[3] = out;
  list[4] = hlt;
  list[5] = sub;
  list[6] = sta;
  list[7] = ldi;
  list[8] = jmp;


  control_unit_lut_t output_CU_LUT = {0};

  uint8_t ISA_CU_Lines = generate_eeprom_ISA_CU_table(list, &output_CU_LUT, intr_list);

  int _temp = 0;
  for(int i = 0; i < ISA_CU_Lines; i++){
	  _temp += write_eeprom_addr(output_CU_LUT.addresses[i], eeprom_control_unit_converter(output_CU_LUT.set_control_bits[i], PROGRAM_EEPROM));
  }
  printf("\nWrote %d/%d bytes to EEPROM.\n\n", _temp, ISA_CU_Lines);

  printContents();

  display_addr = 0;
//  ssd1306_Fill(Black);
//  sprintf(dataText, "%04x %02x %02x %02x %02x", display_addr, read_eeprom_addr(display_addr), read_eeprom_addr(display_addr + 1), read_eeprom_addr(display_addr + 2), read_eeprom_addr(display_addr + 3));
//  ssd1306_SetCursor(5, 1);
//  retVal = ssd1306_WriteString(dataText, Font_7x10, White);
//
//  display_addr += 4;
//  sprintf(dataText, "%04x %02x %02x %02x %02x", display_addr, read_eeprom_addr(display_addr), read_eeprom_addr(display_addr + 1), read_eeprom_addr(display_addr + 2), read_eeprom_addr(display_addr + 3));
//  ssd1306_SetCursor(5, 15);
//  retVal = ssd1306_WriteString(dataText, Font_7x10, White);
//  ssd1306_UpdateScreen();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin) == GPIO_PIN_SET){
		  display_addr += 4;
		  HAL_Delay(50);
	  }
	  if(HAL_GPIO_ReadPin(IN2_GPIO_Port, IN2_Pin) == GPIO_PIN_SET){
		  display_addr -= 4;
		  HAL_Delay(50);
	  }

	  ssd1306_Fill(Black);
	  sprintf(dataText, "%04x %02x %02x %02x %02x", display_addr, read_eeprom_addr(display_addr), read_eeprom_addr(display_addr + 1), read_eeprom_addr(display_addr + 2), read_eeprom_addr(display_addr + 3));
	  ssd1306_SetCursor(5, 1);
	  retVal = ssd1306_WriteString(dataText, Font_7x10, White);

	  sprintf(dataText, "%04x %02x %02x %02x %02x", (display_addr + 4), read_eeprom_addr(display_addr + 4), read_eeprom_addr(display_addr + 5), read_eeprom_addr(display_addr + 6), read_eeprom_addr(display_addr + 7));
	  ssd1306_SetCursor(5, 15);
	  retVal = ssd1306_WriteString(dataText, Font_7x10, White);
	  ssd1306_UpdateScreen();


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GREEN_Pin|WRITE_LED_Pin|READ_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AD0_Pin|AD1_Pin|AD2_Pin|AD3_Pin
                          |AD4_Pin|AD5_Pin|AD6_Pin|AD7_Pin
                          |AD8_Pin|AD9_Pin|AD10_Pin|AD11_Pin
                          |AD12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, WE_Pin|OE_Pin|CE_Pin|IO4_Pin
                          |IO5_Pin|IO6_Pin|IO7_Pin|IO0_Pin
                          |IO1_Pin|IO2_Pin|IO3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GREEN_Pin WRITE_LED_Pin READ_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin|WRITE_LED_Pin|READ_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AD0_Pin AD1_Pin AD2_Pin AD3_Pin
                           AD4_Pin AD5_Pin AD6_Pin AD7_Pin
                           AD8_Pin AD9_Pin AD10_Pin AD11_Pin
                           AD12_Pin */
  GPIO_InitStruct.Pin = AD0_Pin|AD1_Pin|AD2_Pin|AD3_Pin
                          |AD4_Pin|AD5_Pin|AD6_Pin|AD7_Pin
                          |AD8_Pin|AD9_Pin|AD10_Pin|AD11_Pin
                          |AD12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : WE_Pin OE_Pin CE_Pin IO4_Pin
                           IO5_Pin IO6_Pin IO7_Pin IO0_Pin
                           IO1_Pin IO2_Pin IO3_Pin */
  GPIO_InitStruct.Pin = WE_Pin|OE_Pin|CE_Pin|IO4_Pin
                          |IO5_Pin|IO6_Pin|IO7_Pin|IO0_Pin
                          |IO1_Pin|IO2_Pin|IO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IN1_Pin */
  GPIO_InitStruct.Pin = IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN2_Pin */
  GPIO_InitStruct.Pin = IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IN2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
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
