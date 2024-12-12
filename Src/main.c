/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Kalmanfilter.h"


#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG	0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C  //  CONGIF ACEL
#define GYRO_CONFIG_REG 0x1B   //	 ConFIG GYRO
#define MPU6050_ADDR 0xD0 // b1101000x


// DATA RED
// Accelerometer Registers (Thanh ghi cam bien gia toc)
#define ACCEL_XOUT_H_REG        0x3B   // Thanh ghi du lieu gia toc truc X (bit cao)
#define ACCEL_XOUT_L_REG        0x3C   // Thanh ghi du lieu gia toc truc X (bit thap)
#define ACCEL_YOUT_H_REG        0x3D   // Thanh ghi du lieu gia toc truc Y (bit cao)
#define ACCEL_YOUT_L_REG        0x3E   // Thanh ghi du lieu gia toc truc Y (bit thap)
#define ACCEL_ZOUT_H_REG        0x3F   // Thanh ghi du lieu gia toc truc Z (bit cao)
#define ACCEL_ZOUT_L_REG        0x40   // Thanh ghi du lieu gia toc truc Z (bit thap)

// Temperature Registers (Thanh ghi cam bien nhiet do)
#define TEMP_OUT_H_REG          0x41   // Thanh ghi du lieu nhiet do (bit cao)
#define TEMP_OUT_L_REG          0x42   // Thanh ghi du lieu nhiet do (bit thap)

// Gyroscope Registers (Thanh ghi cam bien con quay hoi chuyen)
#define GYRO_XOUT_H_REG         0x43   // Thanh ghi du lieu con quay truc X (bit cao)
#define GYRO_XOUT_L_REG         0x44   // Thanh ghi du lieu con quay truc X (bit thap)
#define GYRO_YOUT_H_REG         0x45   // Thanh ghi du lieu con quay truc Y (bit cao)
#define GYRO_YOUT_L_REG         0x46   // Thanh ghi du lieu con quay truc Y (bit thap)
#define GYRO_ZOUT_H_REG         0x47   // Thanh ghi du lieu con quay truc Z (bit cao)
#define GYRO_ZOUT_L_REG         0x48   // Thanh ghi du lieu con quay truc Z (bit thap)

// External Sensor Data Registers (Thanh ghi du lieu cam bien ngoai)
#define EXT_SENS_DATA_00_REG    0x73   // Du lieu cam bien ngoai (byte 0)
#define EXT_SENS_DATA_01_REG    0x74   // Du lieu cam bien ngoai (byte 1)
#define EXT_SENS_DATA_02_REG    0x75   // Du lieu cam bien ngoai (byte 2)
#define EXT_SENS_DATA_03_REG    0x76   // Du lieu cam bien ngoai (byte 3)
#define EXT_SENS_DATA_04_REG    0x77   // Du lieu cam bien ngoai (byte 4)
#define EXT_SENS_DATA_05_REG    0x78   // Du lieu cam bien ngoai (byte 5)
#define EXT_SENS_DATA_06_REG    0x79   // Du lieu cam bien ngoai (byte 6)
#define EXT_SENS_DATA_07_REG    0x80   // Du lieu cam bien ngoai (byte 7)
#define EXT_SENS_DATA_08_REG    0x81   // Du lieu cam bien ngoai (byte 8)
#define EXT_SENS_DATA_09_REG    0x82   // Du lieu cam bien ngoai (byte 9)
#define EXT_SENS_DATA_10_REG    0x83   // Du lieu cam bien ngoai (byte 10)
#define EXT_SENS_DATA_11_REG    0x84   // Du lieu cam bien ngoai (byte 11)
#define EXT_SENS_DATA_12_REG    0x85   // Du lieu cam bien ngoai (byte 12)
#define EXT_SENS_DATA_13_REG    0x86   // Du lieu cam bien ngoai (byte 13)
#define EXT_SENS_DATA_14_REG    0x87   // Du lieu cam bien ngoai (byte 14)
#define EXT_SENS_DATA_15_REG    0x88   // Du lieu cam bien ngoai (byte 15)
#define EXT_SENS_DATA_16_REG    0x89   // Du lieu cam bien ngoai (byte 16)
#define EXT_SENS_DATA_17_REG    0x90   // Du lieu cam bien ngoai (byte 17)
#define EXT_SENS_DATA_18_REG    0x91   // Du lieu cam bien ngoai (byte 18)
#define EXT_SENS_DATA_19_REG    0x92   // Du lieu cam bien ngoai (byte 19)
#define EXT_SENS_DATA_20_REG    0x93   // Du lieu cam bien ngoai (byte 20)
#define EXT_SENS_DATA_21_REG    0x94   // Du lieu cam bien ngoai (byte 21)
#define EXT_SENS_DATA_22_REG    0x95   // Du lieu cam bien ngoai (byte 22)
#define EXT_SENS_DATA_23_REG    0x96   // Du lieu cam bien ngoai (byte 23)

// I2C Slave Data Output Registers (Thanh ghi xuat du lieu tu I2C slave)
#define I2C_SLV0_DO_REG         0x99   // Du lieu xuat tu I2C slave 0
#define I2C_SLV1_DO_REG         0x100  // Du lieu xuat tu I2C slave 1
#define I2C_SLV2_DO_REG         0x101  // Du lieu xuat tu I2C slave 2
#define I2C_SLV3_DO_REG         0x102  // Du lieu xuat tu I2C slave 3

const uint16_t i2c_timeout = 100;
 double Accel_Z_corrector = 14418.0; // offset Accel z


double vel_z  = 0;
double angle_z = 0;
uint32_t time_sample = 0;

typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;
    double De_Gx; // T?ng l?y m?u 400 l?n / 400
    double De_Gy;
    double De_Gz;

    double De_Ax;
    double De_Ay;
    double De_Az;

    double Num_Z[400]; // Ch?a 400 l?n l?y m?u Az
    double Num_X[400];
    double Num_Y[400];

    double Num_Gz[400];

    double DeltaGyro_Z[3];

    double Gyro_Z[3]; 

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
    double KalmanAngleZ;
} MPU6050_t;

MPU6050_t MPU6050;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern SimpleKalmanFilter Filter_Ax;
extern SimpleKalmanFilter Filter_Ay;
extern SimpleKalmanFilter Filter_Az;
extern SimpleKalmanFilter Filter_Gz;
extern SimpleKalmanFilter Filter_Gy;
extern SimpleKalmanFilter Filter_Gx;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR +1 , WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 115)  // 0x68 Check address from slave return
    {
				Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
        /*
				The Sample Rate is generated by dividing the gyroscope output rate by SMPLRT_DIV:  
				Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) 
				where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz 
				when the DLPF is enabled (see Register 26). 
				*/
        Data = 0x03; // 1KHZ
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        /*Bits in the register:
				Bit 7-5: Reserved.
				Bit 4 (XA_ST): Enables the X-axis auto-test of the accelerometer.
				Bit 3 (YA_ST): Enables the Y-axis auto-test of the accelerometer.
				Bit 2 (ZA_ST): Enables the Z-axis auto-test of the accelerometer.
				Bit 1-0 (AFS_SEL[1:0]): Selects the measurement range of the accelerometer. Available values:
				0: ±2g
				1: ±4g
				2: ±8g
				3: ±16g
				*/
        Data = 0x01; // +-4G
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);
				/* Bit 3 vs Bit 4 for setting range of the GYRO
				0: 250 
				1: 500
				2: 1000
				3: 2000 reg/s
				*/
        Data = 0x18; // b00010000 <=> 1000reg/s
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);


    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);


    DataStruct->Gx = DataStruct->Gyro_X_RAW / 32.8;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 32.8;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 16.4;
}


double offset_vel = 0;
double sum_offset = 0;
uint32_t time_offset = 0;
int circle_offset = 2000;
void off_set(){
	while(circle_offset > 0){
		if(HAL_GetTick() - time_offset >= 1){
			MPU6050_Read_Gyro(&hi2c1, &MPU6050);
			vel_z = SimpleKalmanFilter_UpdateEstimate(&Filter_Gz,MPU6050.Gz);
			sum_offset += vel_z; 
			circle_offset--;
			time_offset = HAL_GetTick();
		}
	}
	offset_vel = sum_offset/2000;
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


	while (MPU6050_Init(&hi2c1) == 1);
	off_set();
	
	HAL_TIM_Base_Start_IT(&htim2);
	
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

	MPU6050_Read_Gyro(&hi2c1, &MPU6050);
	vel_z = SimpleKalmanFilter_UpdateEstimate(&Filter_Gz,MPU6050.Gz);
	angle_z += (vel_z - offset_vel )*0.00025 ;//*0.6923*1.125 ;
	HAL_GPIO_TogglePin(LED_PA7_GPIO_Port, LED_PA7_Pin);
	
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
