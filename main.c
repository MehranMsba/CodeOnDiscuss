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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "iicb_interface.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C<<1
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL2     0x0B
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define MPU9250_ADDRESS  0x68<<1

#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define MPU9250_USER_CTRL                        0x6a 

#define DATA_READY_MASK 0x01
#define MAGIC_OVERFLOW_MASK 0x8

#define MPU9250_CLKSRC_AUTO      1


#define MPU9250_CLKSRC_LOC       0
#define MPU9250_GYRO_FS_LOC      3
#define MPU9250_ACCEL_FS_LOC     3


#define MPU9250_PWR_MGMT_1                       0x6b
#define MPU9250_PWR_MGMT_2                       0x6c

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float MagX;
float MagY;
float MagZ;

/* USER CODE BEGIN PV */
enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x02;

const uint16_t i2c_timeoutB = 1000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return(ch);
}

float MagnetoX;
float MagnetoY;
float MagnetoZ;

HAL_StatusTypeDef halStat;
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
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
 printf("**************************** \r\n");
    printf("MPU9250 STM32 Implementation \r\n");
    printf("**************************** \r\n");

  //pre-def. vars
    uint8_t readData,readData2;
    uint8_t writeData;

    // Check MPU and Mag---------------------------------------------------------------------------------------------------
    //read MPU9255 WHOAMI
    HAL_I2C_Mem_Read(&hi2c2, MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, &readData, 1, i2c_timeoutB);
    printf("MPU WHO AM I is (Must return 113): %d\r\n", readData);

    writeData = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, MPU9250_USER_CTRL, 1, &writeData, 1, i2c_timeoutB);
		
    //enable Mag bypass
    //writeData = 0x22;
		writeData = 0x02;
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, INT_PIN_CFG, 1, &writeData, 1, i2c_timeoutB);


    //read AK8963 WHOAMI
    HAL_I2C_Mem_Read(&hi2c2, AK8963_ADDRESS, AK8963_WHO_AM_I, 1, &readData, 1, i2c_timeoutB);
    printf("MAG WHO AM I is (Must return 72): %d\r\n", readData);

    printf("------------------------------------------------\r\n");




    //Init Mag-------------------------------------------------------------------------------------------------------------
    //Power down magnetometer
    writeData = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeoutB);
    HAL_Delay(500);

    //Enter Fuse ROM access mode
    writeData = 0x0F;
    HAL_I2C_Mem_Write(&hi2c2, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeoutB);
    HAL_Delay(500);

    //Read the x-, y-, and z-axis calibration values
    uint8_t rawMagCalData[3];
    HAL_I2C_Mem_Read(&hi2c2, AK8963_ADDRESS, AK8963_ASAX, 1, &rawMagCalData[0], 3, i2c_timeoutB);
    float calMagX =  (float)(rawMagCalData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
    float calMagY =  (float)(rawMagCalData[1] - 128)/256. + 1.;
    float calMagZ =  (float)(rawMagCalData[2] - 128)/256. + 1.;

    printf("Mag cal off X: %f\r\n", calMagX);
    printf("Mag cal off Y: %f\r\n", calMagY);
    printf("Mag cal off Z: %f\r\n", calMagZ);
    HAL_Delay(100);

    //Power down magnetometer
    writeData = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeoutB);
    HAL_Delay(2500);//2500mSecond ->2.5S


    //Set magnetometer data resolution and sample ODR
    writeData = (0 << 4) | 0x02;
    HAL_I2C_Mem_Write(&hi2c2, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeoutB);

    printf("------------------------------------------------\r\n");
		HAL_Delay(2500);
		
		readData=0;
    halStat=HAL_I2C_Mem_Read(&hi2c2, AK8963_ADDRESS, AK8963_CNTL, 1, &readData, 1, i2c_timeoutB);
		if(halStat==HAL_OK)
		{
    printf("Readback writeData: %d\r\n", readData);
		}
  
    //Read Mag-------------------------------------------------------------------------------------------------------------
    //Read Mag data
//    uint8_t rawMagData[6];
//    HAL_I2C_Mem_Read(&hi2c2, AK8963_ADDRESS, AK8963_XOUT_L, 1, &rawMagData[0], 6, i2c_timeoutB);
//    MagX = ((int16_t)rawMagData[1] << 8) | rawMagData[0];
//    MagY = ((int16_t)rawMagData[3] << 8) | rawMagData[2];
//    MagZ = ((int16_t)rawMagData[5] << 8) | rawMagData[4];

//    printf("Mag X: %f\r\n", MagX);
//    printf("Mag Y: %f\r\n", MagY);
//    printf("Mag Z: %f\r\n", MagZ);
//    HAL_Delay(100);

//    printf("------------------------------------------------\r\n");
    

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint8_t ff[10];

	 HAL_I2C_Mem_Read(&hi2c2, AK8963_ADDRESS, AK8963_ST1, 1, ff, 10, i2c_timeoutB);
					printf("Read Data: %d\r\n", readData);					
					if( (ff[0] & 0x01) == 0x01 )
					{

							//Read Mag data
							uint8_t rawMagData[7];
							halStat=HAL_I2C_Mem_Read(&hi2c2, AK8963_ADDRESS, AK8963_XOUT_L, 1, &rawMagData[0], 7, i2c_timeoutB);
							uint8_t c = rawMagData[6];

							if(!(c & 0x08)) {
									MagX = ((int16_t)rawMagData[1] << 8) | rawMagData[0];
									MagY = ((int16_t)rawMagData[3] << 8) | rawMagData[2];
									MagZ = ((int16_t)rawMagData[5] << 8) | rawMagData[4];
									
									printf("Mag X: %f\r\n", MagX);
									printf("Mag Y: %f\r\n", MagY);
									printf("Mag Z: %f\r\n", MagZ);
									printf("------------------------------------------------\r\n");
							}

					}
					else
					{
							printf("No Data? \r\n");
							printf("------------------------------------------------\r\n");
					}
        HAL_Delay(500);
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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

