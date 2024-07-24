/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include<stdio.h>






int main(void)
{

	GPIO_TypeDef  GPIO_Config;
	GPIO_Config.MODER  = 00;
	GPIO_Config.OTYPER = 01;

}


/*
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;




  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_USART2_UART_Init(void)
{

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


}

static void MX_USART3_UART_Init(void)
{


  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }


}




static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};


  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin|LCD_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin
                            |LCD_D7_Pin|GPIO_PIN_2, GPIO_PIN_RESET);


    GPIO_InitStruct.Pin  = LCD_RS_Pin|LCD_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LCD_D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LCD_D7_GPIO_Port, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}




void INIT_SI8900_ADC1(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);

	for(int  i =0 ;i<2000;i++)
	{
		HAL_UART_Transmit(&huart2,&cmd_INIT, 1, 10);
		HAL_UART_Receive(&huart2, &RXbyte1, 1, 10);

		if(RXbyte1==0x55)
		{

			HAL_UART_Receive(&huart2, &RXbyte1, 1, 10);
			HAL_UART_Receive(&huart2, &RXbyte1, 1, 10);

			break;
		}

	}
}


void INIT_SI8900_ADC2(void)
{
	uint8_t Code_receive, Code_confirm;
	uint16_t unCounter;
	Code_receive = 0; // Correct code receive = 0
	Code_confirm = 0; // Confirm code receiving = 0
	unCounter = 0;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);
//
//	for(int  i =0 ;i<2000;i++)
//	{
//		HAL_UART_Transmit(&huart2,&cmd_INIT, 1, 10);
//		HAL_UART_Receive(&huart2, &RXbyte2, 1, 10);
//
//		if(RXbyte2==0x55)
//		{
//			HAL_UART_Receive(&huart2, &RXbyte2, 1, 10);
//			HAL_UART_Receive(&huart2, &RXbyte2, 1, 10);
//
//			break;
//		}
//
//	}

		HAL_UART_Transmit(&huart2,&cmd_INIT, 1, 10);
		while((Code_receive==0) || (Code_confirm == 0))
		{
			unCounter++;
			HAL_UART_Receive(&huart2, &RXbyte2, 1, 10);

			if(RXbyte2==0x55)
			{
				if (Code_receive == 1)
				{
					Code_confirm = 1; // Confirm communication established
				}
				Code_receive = 1;     // Initial communication established

			}
			else
			{
				Code_receive = 0;
				Code_confirm = 0;
			}
			HAL_UART_Transmit(&huart2,&cmd_INIT, 1, 10);

			if(unCounter >=500)
			{
				break;
			}
		}


//SBUF = 0xAA; // Send the first timing sample to UART
//// transmission buffer SBUF
//Code_receive = 0; // Correct code receive = 0
//Code_confirm = 0; // Confirm code receiving = 0
//	while ((Code receive == 0)||( Code_confirm == 0))
//	{ // Establish correct communication and confirm
//		while ((SCON & 0x01) != 0x01); // Response received?
//		SCON &= 0xFE; // Clear receiving indicator
//	if ( SBUF == 0x55 ) // Check two continuous "55" receiving
//	{
//		if (Code_receive == 1)
//		{
//			Code_confirm = 1; // Confirm communication established
//		}
//		Code_receive = 1; // Initial communication established
//	}
//	else
//	{
//		Code_receive = 0; // If receive one in-correctly, clear both
//		Code_confirm = 0; // Code_receive & Code_confirm
//	}
//	SBUF = 0xAA; // If not right, keep trying
//}


}


void Read_ADC_1(void)
{

	//ADC CH0
	HAL_UART_Transmit(&huart2, &cmd_AIN0, 1, 10);
	HAL_UART_Receive(&huart2, RXbuff1, 3,10);
	Data_Buff_1_CH0[0]=0;
	Data_Buff_1_CH0[1]=0;
	Data_Buff_1_CH0[2]=0;
	Data_Buff_1_CH0[0]  = RXbuff1[1]&(0x0F);
	Data_Buff_1_CH0[1]  = (RXbuff1[2]&(0x7E))>>1;
	Data_Buff_1_CH0[2] |= (Data_Buff_1_CH0[0]<<6);
	Data_Buff_1_CH0[2] |= (Data_Buff_1_CH0[1]);
	ADC1_CH0_Data = Data_Buff_1_CH0[2];
	//ADC CH1
	HAL_UART_Transmit(&huart2, &cmd_AIN1, 1, 10);
	HAL_UART_Receive(&huart2, RXbuff1, 3,10);
	Data_Buff_1_CH1[0]=0;
	Data_Buff_1_CH1[1]=0;
	Data_Buff_1_CH1[2]=0;
	Data_Buff_1_CH1[0]=   RXbuff1[1]&(0x0F);
	Data_Buff_1_CH1[1]  = (RXbuff1[2]&(0x7E))>>1;
	Data_Buff_1_CH1[2] |= (Data_Buff_1_CH1[0]<<6);
	Data_Buff_1_CH1[2] |= (Data_Buff_1_CH1[1]);
	ADC1_CH1_Data = Data_Buff_1_CH1[2];

	//ADC CH2
	HAL_UART_Transmit(&huart2, &cmd_AIN2, 1, 10);
	HAL_UART_Receive(&huart2, RXbuff1, 3,10);
	Data_Buff_1_CH2[0]=0;
	Data_Buff_1_CH2[1]=0;
	Data_Buff_1_CH2[2]=0;
	Data_Buff_1_CH2[0]=   RXbuff1[1]&(0x0F);
	Data_Buff_1_CH2[1]  = (RXbuff1[2]&(0x7E))>>1;
	Data_Buff_1_CH2[2] |= (Data_Buff_1_CH2[0]<<6);
	Data_Buff_1_CH2[2] |= (Data_Buff_1_CH2[1]);
	ADC1_CH2_Data = Data_Buff_1_CH2[2];

	Len0 = sprintf(ADC1_CH0_Data_Buff,"%d", ADC1_CH0_Data);
	Len1 = sprintf(ADC1_CH1_Data_Buff,"%d", ADC1_CH1_Data);
	Len2 = sprintf(ADC1_CH2_Data_Buff,"%d", ADC1_CH2_Data);

	HAL_Delay(500);
	lcd_gotoxy(0,0);//down line
	lcd_printstr("ADC CH0:",8);
	HAL_Delay(500);
	lcd_gotoxy(1,0);
	lcd_printstr(ADC1_CH0_Data_Buff,Len0);

	lcd_gotoxy(0,0);//down line
	lcd_printstr("ADC CH1:",8);
	HAL_Delay(500);
	lcd_gotoxy(1,0);
	lcd_printstr(ADC1_CH1_Data_Buff,Len1);

	lcd_gotoxy(0,0);//down line
	lcd_printstr("ADC CH2:",8);
	HAL_Delay(500);
	lcd_gotoxy(1,0);
	lcd_printstr(ADC1_CH2_Data_Buff,Len2);
}


void Read_ADC_2(void)
{
	//ADC CH0
	HAL_UART_Transmit(&huart2, &cmd_AIN0, 1, 10);
	HAL_UART_Receive(&huart2, RXbuff2, 3,10);
	Data_Buff_2_CH0[0]=0;
	Data_Buff_2_CH0[1]=0;
	Data_Buff_2_CH0[2]=0;
	Data_Buff_2_CH0[0]  = RXbuff2[1]&(0x0F);
	Data_Buff_2_CH0[1]  = (RXbuff2[2]&(0x7E))>>1;
	Data_Buff_2_CH0[2] |= (Data_Buff_2_CH0[0]<<6);
	Data_Buff_2_CH0[2] |= (Data_Buff_2_CH0[1]);
	ADC2_CH0_Data = Data_Buff_2_CH0[2];
	//ADC CH1
	HAL_UART_Transmit(&huart2, &cmd_AIN1, 1, 10);
	HAL_UART_Receive(&huart2, RXbuff2, 3,10);
	Data_Buff_2_CH1[0]=0;
	Data_Buff_2_CH1[1]=0;
	Data_Buff_2_CH1[2]=0;
	Data_Buff_2_CH1[0]=   RXbuff2[1]&(0x0F);
	Data_Buff_2_CH1[1]  = (RXbuff2[2]&(0x7E))>>1;
	Data_Buff_2_CH1[2] |= (Data_Buff_2_CH1[0]<<6);
	Data_Buff_2_CH1[2] |= (Data_Buff_2_CH1[1]);
	ADC2_CH1_Data = Data_Buff_2_CH1[2];

	//ADC CH2
	HAL_UART_Transmit(&huart2, &cmd_AIN2, 1, 10);
	HAL_UART_Receive(&huart2, RXbuff2, 3,10);
	Data_Buff_2_CH2[0]=0;
	Data_Buff_2_CH2[1]=0;
	Data_Buff_2_CH2[2]=0;
	Data_Buff_2_CH2[0]=   RXbuff2[1]&(0x0F);
	Data_Buff_2_CH2[1]  = (RXbuff2[2]&(0x7E))>>1;
	Data_Buff_2_CH2[2] |= (Data_Buff_2_CH2[0]<<6);
	Data_Buff_2_CH2[2] |= (Data_Buff_2_CH2[1]);
	ADC2_CH2_Data = Data_Buff_2_CH2[2];

	A2_Len0 = sprintf(ADC2_CH0_Data_Buff,"%d", ADC2_CH0_Data);
	A2_Len1 = sprintf(ADC2_CH1_Data_Buff,"%d", ADC2_CH1_Data);
	A2_Len2 = sprintf(ADC2_CH2_Data_Buff,"%d", ADC2_CH2_Data);
}






void lcd_init()
{
	HAL_Delay(100);
	LCD_sendCmd(0x33); // Initialize controller
	LCD_sendCmd(0x32); // Set 4-bit mode
	LCD_sendCmd(0x28); // 4 bit, 2 line, 5x7
	LCD_sendCmd(0x06); // Cursor direction -> right
	LCD_sendCmd(0x0C); // Display on, cursor off
	LCD_sendCmd(0x01); // Clear display
	HAL_Delay(100);
}



void lcd_write_nibble(uint8_t rs, uint8_t data)// eg. 0x65 for 'U' so val =6 for hi_nibble and 5 for low_nibble
{
//	uint32_t v = data;
	// rs=0: for cmd	and rs=1: for data
	if(rs==LCD_CMD)
		 HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);


    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, !!(data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, !!(data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, !!(data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, !!(data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);

}

void lcd_write(uint8_t rs, uint8_t val)
{
	uint8_t high = val >> 4, low = val & 0x0F;
	lcd_write_nibble(rs, high);
	lcd_write_nibble(rs, low);
	//lcd_busywait();
	HAL_Delay(10);
}


int lcd_gotoxy(uint8_t row, uint8_t col)
{
	if(row>1 || col>15)
		return 0; // false => failed
	if(row==0)	//row 0: ADDR = 0x00 :: 0x00 + 0x80 ==0x80 coz 7th bit always high...
		lcd_write(LCD_CMD, 0x80+col);
	else		//row 1: ADDR = 0x40 :: 0x40 + 0x80 ==0xC0 coz 7th bit always high..
		lcd_write(LCD_CMD, 0xC0+col);
	return 1; // true => success
}

void lcd_putchar(char ch)
{
	lcd_write(LCD_DATA, ch);
}

void lcd_putstr(const char *str)
{
	uint8_t cnt;
	//lcd_gotoxy(row, 0);
	for(cnt=0; *str!='\0' && cnt<16; cnt++)
		lcd_putchar(*str++);
}

void lcd_printstr(const char *str,uint8_t len)
{
	uint8_t i=0;
	for(i=0; i<len && i<16; i++)
	{
		lcd_putchar(str[i]);
	}
}

void lcd_clrscr(void)
{

    lcd_write(LCD_CMD, 0x01); //Clear screen
}


void lcd_home(void)
{
    lcd_command(1<<LCD_HOME);
}


void lcd_command(uint8_t cmd)
{
   // lcd_busywait();
    lcd_write(LCD_CMD, cmd);
}


void lcd_pattern(unsigned char location, unsigned char *ptr)
{
	unsigned char i;
	if(location<8)
	{
		lcd_command(0x40+(location*8));
		for(i=0;i<8;i++)
		lcd_putchar(ptr[ i ]);
	}
}

void LCD_sendCmd(uint8_t data) {
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    LCD_sendByte(data);
}

void LCD_sendChar(uint8_t data) {
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
    LCD_sendByte(data);
}

void LCD_sendNibble(uint8_t data) {
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, !!(data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, !!(data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, !!(data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, !!(data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
}

void LCD_sendByte(uint8_t data) {
    LCD_sendNibble(data >> 4); // High order bit
    LCD_sendNibble(data); // Low order bit
}

void LCD_puts(char * data)
	{
    while (data[0] != '\0')
			{
        LCD_sendChar(data[0]);
        data++;
		}
}





*/
