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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MPU9250_ADDR		0xD0


#define XG_OFFSET_H			0X13
#define XG_OFFSET_L			0X14
#define YG_OFFSET_H			0X15
#define YG_OFFSET_L			0X16
#define ZG_OFFSET_H			0X17
#define ZG_OFFSET_L			0X18

#define SMPLRT_DIV			0x19

#define MPU9250_CONFIG		0X1A
#define GYRO_CONFIG			0X1B
#define ACCEL_CONFIG		0X1C
#define ACCEL_CONFIG_2		0X1D

#define ACCEL_XOUT_H		0X3B
#define ACCEL_XOUT_L		0X3C
#define ACCEL_YOUT_H		0X3D
#define ACCEL_YOUT_L		0X3E
#define ACCEL_ZOUT_H		0X3F
#define ACCEL_ZOUT_L		0X40

#define TEMP_OUT_H			0X41
#define TEMP_OUT_L			0x42

#define GYRO_XOUT_H			0X43
#define GYRO_XOUT_L			0X44
#define GYRO_YOUT_H			0X45
#define GYRO_YOUT_L			0X46
#define GYRO_ZOUT_H			0X47
#define GYRO_ZOUT_L			0X48

#define PWR_MGMT_1			0x6B
#define WHO_AM_I_REG 		0x75

#define XA_OFFSET_H			0X77
#define XA_OFFSET_L			0X78
#define YA_OFFSET_H			0X7A
#define YA_OFFSET_L			0X7B
#define ZA_OFFSET_H			0X7D
#define ZA_OFFSET_L			0X7E




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


uint8_t GYRO_DLPF_CFG		= 0X02; 	// Set digital low-pass filter 0x02 to get bandwidth 92Hz
uint8_t ACCEL_DLPF_CFG		= 0X02;		// Set digital low-pass filter 0x02 to get bandwidth 99Hz

uint8_t GYRO_FS				= 0x08;		// +500DPS
uint8_t ACCEL_FS			= 0X10;		// +-8G

uint8_t PWR_CONIG			= 0X00;		// Auto select the best available clock source
uint8_t SMPLRT_CONFIG		= 0X07;		// Sample_Rate = Internal_Sample_Rate/(1+SMPLRT_CONFIG)
										// Sample_Rate = 8Khz/(1+8) = 1Khz
uint8_t MPU9250_RESPONSE_OK	= 0x71;		// Check WHO_AM_I if it is OK it will 0x71 or 113



int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;

long gyro_x_cal, gyro_y_cal, gyro_z_cal;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
bool set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;


uint8_t MPU9250_Check		= 0;
uint32_t Current_Time		= 0;

uint8_t uart_buff;
uint8_t buf[20] = "Hello\n";
uint8_t MPU_Is_Active		= 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void MPU9250_Init(void){
	 if (HAL_I2C_Mem_Read (&hi2c1, MPU9250_ADDR,WHO_AM_I_REG,1, &MPU9250_Check, 1, 1000) == HAL_OK ){
		 if ( MPU9250_Check == MPU9250_RESPONSE_OK ){

			 HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, PWR_MGMT_1, 1, &PWR_CONIG, 1, 100);				// Auto select best available clock source
			 HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, SMPLRT_DIV, 1, &SMPLRT_CONFIG, 1, 100);			// 1Khz sample rate

			 HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, MPU9250_CONFIG, 1, &GYRO_DLPF_CFG, 1, 100);		// Set digital low-pass filter 0x02 to get bandwidth 92Hz
			 HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, ACCEL_CONFIG_2, 1, &ACCEL_DLPF_CFG, 1, 100);		// Set digital low-pass filter 0x02 to get bandwidth 99Hz

			 HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, GYRO_CONFIG, 1, &GYRO_FS, 1, 100);					// Set GYRO to full scale +250DPS
			 HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, GYRO_CONFIG, 1, &ACCEL_FS, 1, 100);				// Set ACCEL to full scale +-2G


		 }

	 }
}

void MPU9250_Read_Data	( void ){

	uint8_t Accel_Val_Raw[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, ACCEL_XOUT_H, 1, Accel_Val_Raw, 6, 1000);

	acc_x = (int16_t) (Accel_Val_Raw[0] << 8 | Accel_Val_Raw [1]);
	acc_y = (int16_t) (Accel_Val_Raw[2] << 8 | Accel_Val_Raw [3]);
	acc_z = (int16_t) (Accel_Val_Raw[4] << 8 | Accel_Val_Raw [5]);

	uint8_t Gyro_Val_Raw[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, GYRO_XOUT_H, 1, Gyro_Val_Raw, 6, 1000);

	gyro_x = (int16_t) (Gyro_Val_Raw[0] << 8 | Gyro_Val_Raw [1]);
	gyro_y = (int16_t) (Gyro_Val_Raw[2] << 8 | Gyro_Val_Raw [3]);
	gyro_z = (int16_t) (Gyro_Val_Raw[4] << 8 | Gyro_Val_Raw [5]);


}

void debug( float val ){
	  int len = sprintf((char*) buf,"%f\n",val);
	  HAL_UART_Transmit(&huart4, buf, len, 100);
}

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
  MX_I2C1_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  MPU9250_Init();
  for ( uint16_t i =0 ; i < 2000 ; i++ ){

		  MPU9250_Read_Data();
		  gyro_x_cal += gyro_x;
		  gyro_y_cal += gyro_y;
		  gyro_z_cal += gyro_z;
		  HAL_Delay(3);

  }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;

  HAL_TIM_Base_Start(&htim10);
  HAL_TIM_Base_Start_IT(&htim11);
  Current_Time = __HAL_TIM_GET_COUNTER(&htim10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



//	  debug(Angle_Pitch_Output);
	  HAL_GPIO_TogglePin(led_pc13_GPIO_Port, led_pc13_Pin);
	  HAL_Delay(100);

//	  while( __HAL_TIM_GET_COUNTER(&htim10) - Current_Time < 10000 );
//	  Current_Time = __HAL_TIM_GET_COUNTER(&htim10);




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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if ( htim == &htim11 ){


		  MPU9250_Read_Data();

		  gyro_x -= gyro_x_cal;
		  gyro_y -= gyro_y_cal;
		  gyro_z -= gyro_z_cal;

		  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
		  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
		  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
		  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
		  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

		  //Accelerometer angle calculations
		  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
		  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
		  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
		  angle_roll_acc = asin((float)acc_x/acc_total_vector)* 57.296;       //Calculate the roll angle

		  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
		  angle_pitch_acc -= 2.1;                                               //Accelerometer calibration value for pitch
		  angle_roll_acc -= 4.1;                                               //Accelerometer calibration value for roll

		  if(set_gyro_angles){                                                 //If the IMU is already started
		    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
		    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
		  }
		  else{                                                                //At first start
		    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
		    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
		    set_gyro_angles = true;                                            //Set the IMU started flag
		  }

		  //To dampen the pitch and roll angles a complementary filter is used
		  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
		  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value



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
