/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

#include <usbd_hid.h>
extern USBD_HandleTypeDef hUsbDeviceFS;
extern float gx, gy, gz, ax, ay, az;
#include <math.h>	
#include <MadgwickAHRS.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
	unsigned int accx, accy, accz;
	int gyro_x, gyro_y, gyro_z;
	long acc_x, acc_y, acc_z, acc_total_vector;
	int temperature;
	long gyro_x_cal, gyro_y_cal, gyro_z_cal;
	long loop_timer;
	float angle_pitch, angle_roll;
	int angle_pitch_buffer, angle_roll_buffer;
  float angle_roll_acc, angle_pitch_acc;
  float angle_pitch_output, angle_roll_output;

	void set_MPU6050_registers(void);
	void read_mpu_6050_data(void);
	
	uint8_t buffer[4];   			// for usb hid mouse //

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
	
	buffer[0] = 0;				// mouse button //
	buffer[1] = 0;				// mouse x 	//
	buffer[2] = 0;				// mouse y //
  buffer[3] = 0;				// mouse wheel //

 	set_MPU6050_registers();

	for (int cal_int = 0; cal_int < 65 ; cal_int ++)										//Run this code 65 times
{                  
	if(cal_int % 125 == 0)HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);    //Print a dot on the LCD every 125 readings
	read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
	gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
	gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
	gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
	HAL_Delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
}
	gyro_x_cal /= 65;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
	gyro_y_cal /= 65;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
	gyro_z_cal /= 65;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	read_mpu_6050_data();
		
	gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

 // converting acc to G's //
	 ax = ((float)acc_x) / 8192.0f;
	 ay = ((float)acc_y) / 8192.0f;
	 az = ((float)acc_z) / 8192.0f;	
	// converting gyro to rad per sec //
		
	 gx = ((float)gyro_x) / 939.650784f;
	 gy = ((float)gyro_y) / 939.650784f;
	 gz = ((float)gyro_z) / 939.650784f;	
	
	/*Implementing madgwick filter function*/
	
  MadgwickAHRSupdateIMU(gx, gy ,gz, ax, ay, az);

	/*calculate the pitch and roll*/
	
 angle_pitch = ((acosf(q0 / sqrt(q0*q0 + q2*q2)) * 2.0f) - 1.570796327f) * (180 / 3.142);
 
 angle_roll = ((acosf(q1 / sqrt(q1*q1 + q2*q2)))-1.570796327f )* (180 / 3.142); 

//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians

 angle_pitch += angle_roll * sin(gz * 0.000001066);               //If the IMU has yawed, transfer the roll angle to the pitch angle
 
 angle_roll -= angle_pitch * sin(gz * 0.000001066);               //If the IMU has yawed, transfer the pitch angle to the roll angle

 //To dampen the pitch and roll angles a complementary filter is used
  
 angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value

 angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
	/*	
// mahony filter algorithm//
	ax = (float)acc_x;
	ay = (float)acc_y;
	az = (float)acc_z;
	gx = (float)gyro_x;
	gy = (float)gyro_x;
	gz = (float)gyro_x;	
	MahonyAHRSupdate(gx, gy, gz, ax, ay, az, NULL, NULL, NULL );
  MahonyAHRSupdateIMU(gx, gy ,gx, ax, ay, az);
	
		//Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gx * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gy * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gz * 0.000001066);               //If the IMU has yawed, transfer the roll angle to the pitch angle
  angle_roll -= angle_pitch * sin(gz * 0.000001066);               //If the IMU has yawed, transfer the pitch angle to the roll angle
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((ax*ax)+(ay*ay)+(az*az));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)ay/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)ax/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

    //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
 
	
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
	 */
	
		HAL_Delay(100);
/*		
	// USB HID MOUSE //	
		if(angle_pitch_output > 155) //&& angle_pitch_output < 155//)
		{
			buffer[0] = 0;				// mouse button //
			buffer[1] = 10;       // mouse x 	//
			buffer[2] = 0;				// mouse y //
			buffer[3] = 0;				// mouse wheel //

			USBD_HID_SendReport(&hUsbDeviceFS, buffer, 4);
			HAL_Delay(10);
		}
		else if(angle_pitch_output < -220) //&& angle_pitch_output > -280)
		{
			buffer[0] = 0;				// mouse button //
			buffer[1] = -10;				// mouse x 	//
			buffer[2] = 0;				// mouse y //
			buffer[3] = 0;				// mouse wheel //

			USBD_HID_SendReport(&hUsbDeviceFS, buffer, 4);
			HAL_Delay(10);
		}
		else if(angle_roll_output > 195) //&& angle_roll_output < 255)
		{
			buffer[0] = 0;				// mouse button //
			buffer[1] = 0;				// mouse x 	//
			buffer[2] = 10;				// mouse y //
			buffer[3] = 0;				// mouse wheel //

			USBD_HID_SendReport(&hUsbDeviceFS, buffer, 4);
			HAL_Delay(10);
		}
		else if(angle_roll_output < 5 )//&& angle_roll_output > -55)
		{
			buffer[0] = 0;				// mouse button //
			buffer[1] = 0;				// mouse x 	//
			buffer[2] = -10;			// mouse y //
			buffer[3] = 0;				// mouse wheel //

			USBD_HID_SendReport(&hUsbDeviceFS, buffer, 4);
			HAL_Delay(10);
		}
		else
		{
			buffer[0] = 0;				// mouse button //
			buffer[1] = 0;				// mouse x 	//
			buffer[2] = 0;				// mouse y //
			buffer[3] = 0;				// mouse wheel //

			USBD_HID_SendReport(&hUsbDeviceFS, buffer, 4);
			HAL_Delay(10);

		}
*/

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
//Subroutine for reading the raw gyro and accelerometer data
void read_mpu_6050_data(){       
	
  uint8_t tempBuffer[14];
	
	tempBuffer[0] = 0x3B;
	HAL_I2C_Master_Transmit(&hi2c1,0x68<<1, tempBuffer, 1, 100);				 //Request the data burst
  

	HAL_I2C_Master_Receive(&hi2c1,0x68<<1, tempBuffer, 14, 100);				 //Request 14 bytes from the MPU-6050	
 
  while( HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_RX);           //Wait until all the bytes are received
	
  acc_x = tempBuffer[0]<<8|tempBuffer[1];                                  //Add the low and high byte to the acc_x variable
  acc_y = tempBuffer[2]<<8|tempBuffer[3];                                  //Add the low and high byte to the acc_y variable
  acc_z = tempBuffer[4]<<8|tempBuffer[5];                                  //Add the low and high byte to the acc_z variable
  temperature = tempBuffer[6]<<8|tempBuffer[7];                            //Add the low and high byte to the temperature variable
  gyro_x = tempBuffer[8]<<8|tempBuffer[9];                                 //Add the low and high byte to the gyro_x variable
  gyro_y = tempBuffer[10]<<8|tempBuffer[11];                                 //Add the low and high byte to the gyro_y variable
  gyro_z = tempBuffer[12]<<8|tempBuffer[13];                                 //Add the low and high byte to the gyro_z variable

}


void set_MPU6050_registers()
{
	unsigned char tempBuffer1[2]; 
	unsigned char tempBuffer2[2]; 
	unsigned char tempBuffer3[2]; 
	tempBuffer1[0]=0x6B;	//Activate the MPU-6050
	tempBuffer1[1]=0x00;
	
	tempBuffer2[0]=0x1C;	//Configure the accelerometer (+/-8g)
	tempBuffer2[1]=0x10;
	
	tempBuffer3[0]=0x1B;	//Configure the gyro (500dps full scale)
	tempBuffer3[1]=0x08;
	
	HAL_I2C_Master_Transmit(&hi2c1,0x68<<1, tempBuffer1, 2, 100);  
  HAL_Delay(20);
	HAL_I2C_Master_Transmit(&hi2c1,0x68<<1, tempBuffer2, 2, 100);
	HAL_Delay(20);
	HAL_I2C_Master_Transmit(&hi2c1,0x68<<1, tempBuffer3, 2, 100);
	HAL_Delay(20);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
