/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dcache.h"
#include "flash.h"
#include "gpdma.h"
#include "i2c.h"
#include "icache.h"
#include "memorymap.h"
#include "tim.h"
#include "ucpd.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ioif_mpu6050.h"
#include "ioif_mpu6050_second.h"
#include "imu_algorithms.h"

#include "ioif_bmp280.h"
//#include "ioif_vl53l0x.h"

#include "VL53L0X.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

static void InitValueSetting(IMU_FuzzyData_t* imuFuzzyData, IMU_NormData_t* imuNormData, IMU_GaitData_t* imuGaitData, IMU_ThresData_t* imuThresData);
static void SetInitialAngle(IMU_AngleData_t* imuAngleData, IMU_NormData_t* imuNormData, float initialAngle);
static void GetInitialAngle_IMU(IOIF_6AxisData_t* imu6AxisData, IMU_SensorData_t* imuSensorData, IMU_AngleData_t* imuAngleData, IMU_NormData_t* imuNormData, IMU_AttachCase_t imuAttachCase);
static void GetInitialAngle_IMU2(IOIF_6AxisData2_t* imu6AxisData, IMU_SensorData_t* imuSensorData, IMU_AngleData_t* imuAngleData, IMU_NormData_t* imuNormData, IMU_AttachCase_t imuAttachCase);
static void ResetDataObj(void);
static void UpdateSensorRawData(IMU_SensorData_t* imuSensorData, IOIF_6AxisData_t* imu6AxisData, IMU_AttachCase_t imuAttachCase);
static void UpdateSensorRawData2(IMU_SensorData_t* imuSensorData, IOIF_6AxisData2_t* imu6AxisData, IMU_AttachCase_t imuAttachCase);
static void RunTvcfFilter(IMU_SensorData_t* imuSensorData, IMU_AngleData_t* imuAngleData, IMU_FuzzyData_t* imuFuzzyData, float samplingPeriod, IMU_AttachCase_t imuAttachCase);
static void SetUsedDegVel(IMU_AngleData_t* imuAngleData);
static int RunIMU(void);
static int RunTotalIMUAlgorithm(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
IOIF_6AxisData_t 	mpu6050DataObj1;
IOIF_6AxisData2_t 	mpu6050DataObj2;

uint32_t codeTime = 0;				// microSecond

uint8_t err1 = 0;
uint8_t err2 = 0;
uint32_t errSum1 = 0;
uint32_t errSum2 = 0;
uint16_t msTime = 0;
uint32_t totalElapsedTime = 0;		// Second


/* For IMU Algorithms */
// For 1st IMU - I2C2 //
IMU_GaitData_t		imuGaitDataObj1;
IMU_AngleData_t 	imuAngleDataObj1;

static IMU_AttachCase_t ATTACH_CASE_SEL_1;

static IMU_SensorData_t 	imuSensorDataObj1;
static IMU_FuzzyData_t		imuFuzzyDataObj1;
static IMU_NormData_t 		imuNormDataObj1;
static IMU_ThresData_t		imuThresDataObj1;

static float wcDebug1 = 0.0;

static uint32_t breakRT = 0;
// For 2nd IMU - I2C3 //
IMU_GaitData_t		imuGaitDataObj2;
IMU_AngleData_t 	imuAngleDataObj2;

static IMU_AttachCase_t ATTACH_CASE_SEL_2;

static IMU_SensorData_t 	imuSensorDataObj2;
static IMU_FuzzyData_t		imuFuzzyDataObj2;
static IMU_NormData_t 		imuNormDataObj2;
static IMU_ThresData_t		imuThresDataObj2;

static float wcDebug2 = 0.0;



/* For BPS */
IOIF_BPSData_t bmp280DataObj;


/* For TOF */
//IOIF_TOFData_t vl53l0xDataObj;
uint16_t distance;

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the System Power */
  SystemPower_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_UCPD1_Init();
  MX_ADC1_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DCACHE1_Init();
  MX_FLASH_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;

  /* Initialize the IMUs */
//  IOIF_Init6Axis(&hi2c2);
//  IOIF_Init6Axis2(&hi2c3);
//
//  /* For IMU Algorithms */
//  /* Initial Setting for the IMU */
//  ATTACH_CASE_SEL_1 = IMU_LEFT_SAGITAL;
//  ATTACH_CASE_SEL_2 = IMU_LEFT_SAGITAL;
//
//  /* Reset and Initial setting */
//  ResetDataObj();
//  InitValueSetting(&imuFuzzyDataObj1, &imuNormDataObj1, &imuGaitDataObj1, &imuThresDataObj1);
//  InitValueSetting(&imuFuzzyDataObj2, &imuNormDataObj2, &imuGaitDataObj2, &imuThresDataObj2);
//
//  /* Get initial angle at Standing state */
//  GetInitialAngle_IMU(&mpu6050DataObj1, &imuSensorDataObj1, &imuAngleDataObj1, &imuNormDataObj1, ATTACH_CASE_SEL_1);
//  GetInitialAngle_IMU2(&mpu6050DataObj2, &imuSensorDataObj2, &imuAngleDataObj2, &imuNormDataObj2, ATTACH_CASE_SEL_2);
  /* ------------------------------------------------------------------------------------------------------------------ */

  /* Initialize the BPS */
//  IOIF_InitBPS(&hi2c1);

  /* Initialize the TOF */
//  IOIF_InitTOF(&hi2c1);



  /* Using Open Source of TOF */
/*
   // Initialise the VL53L0X
   statInfo_t_VL53L0X distanceStr;
   initVL53L0X(1, &hi2c1);

	// Configure the sensor
	setSignalRateLimit(200);
	setVcselPulsePeriod(VcselPeriodPreRange, 10);
	setVcselPulsePeriod(VcselPeriodFinalRange, 14);
	setMeasurementTimingBudget(300 * 1000UL);
*/

  /* Start the TIMER INTERRUPT */
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		distance = readRangeSingleMillimeters(&distanceStr);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  HAL_PWREx_EnableVddIO2();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim1.Instance){

		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;


		/* Choose the Function you want to RUN */
//		RunIMU();
//		RunTotalIMUAlgorithm();

		/* For BPS */
//		IOIF_GetBPSValue(&bmp280DataObj);

		/* For TOF */
//		IOIF_GetTOFValue(&vl53l0xDataObj);

		/* ----------------------------------- */

		if (msTime == 1000){
			msTime = 0;
			totalElapsedTime++;

//			IOIF_GetTOFValue(&vl53l0xDataObj);
		}

		/* Code End */
		codeTime = DWT->CYCCNT/160;
		msTime++;

		if (codeTime > 1000){
			breakRT++;
		}
	}
}



/* Private functions */

/* Setting for initial parameters */
static void InitValueSetting(IMU_FuzzyData_t* imuFuzzyData, IMU_NormData_t* imuNormData, IMU_GaitData_t* imuGaitData, IMU_ThresData_t* imuThresData)
{
	imuFuzzyData->wl 				= 0.5;
	imuFuzzyData->wh 				= 10.0;
	imuFuzzyData->var[0] 			= 8.0;
	imuFuzzyData->var[1] 			= 30.0;
	imuFuzzyData->var[2] 			= 5.8;
	imuFuzzyData->var[3] 			= 320.0;

	imuNormData->ampDeg 			= 30.0; 	//30
	imuNormData->ampDegLPF[0]		= 30.0;
	imuNormData->ampDegLPF[1]		= 30.0;

	imuNormData->ampVel 			= 400.0; 	//400
	imuNormData->ampVelLPF[0]		= 400.0;
	imuNormData->ampVelLPF[1]		= 400.0;

	imuGaitData->gaitPeriod 		= 1000;
	imuGaitData->gaitPhase 			= -100.0;
	imuGaitData->gaitPhasePre   	= -100.0;

	imuThresData->degThStart		= 12.0;
	imuThresData->velThStart		= 20.0;
	imuThresData->degThStop 		= 3.0;
	imuThresData->velThStop 		= 12.0;
}

/* Setting for Initial values of "angle" variables after get initial thigh angle */
static void SetInitialAngle(IMU_AngleData_t* imuAngleData, IMU_NormData_t* imuNormData, float initialAngle)
{
	imuAngleData->degAccFiltered 	= initialAngle;
	imuAngleData->degGyrFiltered	= 0.0;

	imuAngleData->degLPF1st[0] 		= initialAngle;
	imuAngleData->degLPF1st[1] 		= initialAngle;
	imuAngleData->degLPF2nd[0] 		= initialAngle;
	imuAngleData->degLPF2nd[1] 		= initialAngle;
	imuNormData->degOri 	 		= initialAngle;
}

/*
 *Function to calculate the initial thigh angle - IMU case
*/
static void GetInitialAngle_IMU(IOIF_6AxisData_t* imu6AxisData, IMU_SensorData_t* imuSensorData, IMU_AngleData_t* imuAngleData, IMU_NormData_t* imuNormData, IMU_AttachCase_t imuAttachCase)
{
	uint8_t tTotalSamples 	= 100;
	uint8_t tDataCheck_IMU	= 0;
	uint8_t tRealSamples	= 0;
	float tAccumulatedAngle = 0.0;
	float tInitThighAngle	= 0.0;
	float tImuAngle			= 0.0;

	for (uint8_t i = 1; i <= tTotalSamples; i++) {
        for (uint16_t j = 0; j < 30000; j++) {
        	// For delay of DMA reading
        }

        tDataCheck_IMU = IOIF_Get6AxisValue(imu6AxisData);

        if (tDataCheck_IMU == 0){
        	imuSensorData->accX[0] = imu6AxisData->accX;
        	imuSensorData->accY[0] = imu6AxisData->accY;
        	imuSensorData->gyrZ[0] = imu6AxisData->gyrZ;

            tImuAngle = IMU_AttachCaseSetting(imuSensorData, imuAttachCase);
            tAccumulatedAngle += tImuAngle;

            tRealSamples++;
        }
    }

	tInitThighAngle = tAccumulatedAngle / tRealSamples;
	imuAngleData->initAngle = tInitThighAngle;
    SetInitialAngle(imuAngleData, imuNormData, tInitThighAngle);
}

/*
 *Function to calculate the initial thigh angle - IMU case
*/
static void GetInitialAngle_IMU2(IOIF_6AxisData2_t* imu6AxisData, IMU_SensorData_t* imuSensorData, IMU_AngleData_t* imuAngleData, IMU_NormData_t* imuNormData, IMU_AttachCase_t imuAttachCase)
{
	uint8_t tTotalSamples 	= 100;
	uint8_t tDataCheck_IMU	= 0;
	uint8_t tRealSamples	= 0;
	float tAccumulatedAngle = 0.0;
	float tInitThighAngle	= 0.0;
	float tImuAngle			= 0.0;

	for (uint8_t i = 1; i <= tTotalSamples; i++) {
        for (uint16_t j = 0; j < 30000; j++) {
        	// For delay of DMA reading
        }

        tDataCheck_IMU = IOIF_Get6AxisValue2(imu6AxisData);

        if (tDataCheck_IMU == 0){
        	imuSensorData->accX[0] = imu6AxisData->accX;
        	imuSensorData->accY[0] = imu6AxisData->accY;
        	imuSensorData->gyrZ[0] = imu6AxisData->gyrZ;

            tImuAngle = IMU_AttachCaseSetting(imuSensorData, imuAttachCase);
            tAccumulatedAngle += tImuAngle;

            tRealSamples++;
        }
    }

	tInitThighAngle = tAccumulatedAngle / tRealSamples;
	imuAngleData->initAngle = tInitThighAngle;
    SetInitialAngle(imuAngleData, imuNormData, tInitThighAngle);
}

static void ResetDataObj(void)
{
	imuSensorDataObj1 		= 	(IMU_SensorData_t){0};
	imuFuzzyDataObj1 		= 	(IMU_FuzzyData_t){0};
	imuAngleDataObj1		=   (IMU_AngleData_t){0};
	imuNormDataObj1 		= 	(IMU_NormData_t){0};
	imuGaitDataObj1 		= 	(IMU_GaitData_t){0};
	imuThresDataObj1 		= 	(IMU_ThresData_t){0};
	wcDebug1					= 	0.0;


	imuSensorDataObj2 		= 	(IMU_SensorData_t){0};
	imuFuzzyDataObj2 		= 	(IMU_FuzzyData_t){0};
	imuAngleDataObj2		=   (IMU_AngleData_t){0};
	imuNormDataObj2 		= 	(IMU_NormData_t){0};
	imuGaitDataObj2 		= 	(IMU_GaitData_t){0};
	imuThresDataObj2 		= 	(IMU_ThresData_t){0};
	wcDebug2					= 	0.0;
}

/*
*The function UpdateSensorRawData updates the IMU raw values.
*/
static void UpdateSensorRawData(IMU_SensorData_t* imuSensorData, IOIF_6AxisData_t* imu6AxisData, IMU_AttachCase_t imuAttachCase)
{
	imuSensorData->accX[0] = imu6AxisData->accX;
	imuSensorData->accY[0] = imu6AxisData->accY;
	imuSensorData->gyrZ[0] = imu6AxisData->gyrZ;

	if (imuAttachCase == IMU_LEFT_SAGITAL){
		imuSensorData->gyrZ[0] = (-1) * (imu6AxisData->gyrZ); 			// For Negative Gyro case (Maybe LEFT case)
	}
	else if (imuAttachCase == IMU_RIGHT_SAGITAL){
		imuSensorData->gyrZ[0] = imu6AxisData->gyrZ; 					// For Positive Gyro case (Maybe RIGHT case)
	}
}

/*
*The function UpdateSensorRawData updates the IMU raw values.
*/
static void UpdateSensorRawData2(IMU_SensorData_t* imuSensorData, IOIF_6AxisData2_t* imu6AxisData, IMU_AttachCase_t imuAttachCase)
{
	imuSensorData->accX[0] = imu6AxisData->accX;
	imuSensorData->accY[0] = imu6AxisData->accY;
	imuSensorData->gyrZ[0] = imu6AxisData->gyrZ;

	if (imuAttachCase == IMU_LEFT_SAGITAL){
		imuSensorData->gyrZ[0] = (-1) * (imu6AxisData->gyrZ); 			// For Negative Gyro case (Maybe LEFT case)
	}
	else if (imuAttachCase == IMU_RIGHT_SAGITAL){
		imuSensorData->gyrZ[0] = imu6AxisData->gyrZ; 					// For Positive Gyro case (Maybe RIGHT case)
	}
}

/*
 *Function to execute the time-varying complementary filter (with Fuzzy Logic - wc)
*/
static void RunTvcfFilter(IMU_SensorData_t* imuSensorData, IMU_AngleData_t* imuAngleData, IMU_FuzzyData_t* imuFuzzyData, float samplingPeriod, IMU_AttachCase_t imuAttachCase)
{
	/* Apply time-varying complementary filter on the sensor data using fuzzy logic(wc) and update the thigh angle parameters */
	IMU_RunTVCF(imuSensorData, imuAngleData, imuFuzzyData->wc, samplingPeriod, imuAttachCase);

	/* Update the unfiltered thigh angle to be the same as the filtered thigh angle */
	imuAngleData->degTvcf[0] = imuAngleData->degTvcfFiltered;
	imuAngleData->velRaw[0] = (imuAngleData->degTvcf[0] - imuAngleData->degTvcf[1]) / IMU_CONTROL_PERIOD;
}

/*
 *Function to select finally used deg&vel value before filtering
*/
static void SetUsedDegVel(IMU_AngleData_t* imuAngleData)
{
	imuAngleData->degFinal = imuAngleData->degTvcf[0] - imuAngleData->initAngle;
	imuAngleData->velFinal = imuAngleData->velRaw[0];
}

/* Total Algorithm Function */
static int RunIMU(void)
{
	err1 = IOIF_Get6AxisValue(&mpu6050DataObj1);
	err2 = IOIF_Get6AxisValue2(&mpu6050DataObj2);

	if (err1 != 0){
		errSum1++;
	}
	if (err2 != 0){
		errSum2++;
	}

	return 0;
}

static int RunTotalIMUAlgorithm(void)
{
	IMU_UpdateBuffer(&imuSensorDataObj1, &imuAngleDataObj1, &imuGaitDataObj1, &imuNormDataObj1);
	IMU_UpdateBuffer(&imuSensorDataObj2, &imuAngleDataObj2, &imuGaitDataObj2, &imuNormDataObj2);

	err1 = IOIF_Get6AxisValue(&mpu6050DataObj1);
	err2 = IOIF_Get6AxisValue2(&mpu6050DataObj2);
	if (err1 != 0){
		errSum1++;
	}
	if (err2 != 0){
		errSum2++;
	}

	UpdateSensorRawData(&imuSensorDataObj1, &mpu6050DataObj1, ATTACH_CASE_SEL_1);
	UpdateSensorRawData2(&imuSensorDataObj2, &mpu6050DataObj2, ATTACH_CASE_SEL_2);

	IMU_CalculateFuzzyInput(&imuSensorDataObj1, &imuFuzzyDataObj1);
	IMU_CalculateFuzzyInput(&imuSensorDataObj2, &imuFuzzyDataObj2);

	wcDebug1 = IMU_CalculateFuzzyWc(&imuFuzzyDataObj1);
	wcDebug2 = IMU_CalculateFuzzyWc(&imuFuzzyDataObj2);

	RunTvcfFilter(&imuSensorDataObj1, &imuAngleDataObj1, &imuFuzzyDataObj1, IMU_CONTROL_PERIOD, ATTACH_CASE_SEL_1);
	RunTvcfFilter(&imuSensorDataObj2, &imuAngleDataObj2, &imuFuzzyDataObj2, IMU_CONTROL_PERIOD, ATTACH_CASE_SEL_2);

	SetUsedDegVel(&imuAngleDataObj1);
	SetUsedDegVel(&imuAngleDataObj2);

	return 0;
}

/* End of Private Functions */
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  MPU_Attributes_InitTypeDef MPU_AttributesInit = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x28000000;
  MPU_InitStruct.LimitAddress = 0x28003fff;
  MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER1;
  MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RW;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  MPU_AttributesInit.Number = MPU_REGION_NUMBER0;
  MPU_AttributesInit.Attributes = MPU_DEVICE_nGnRnE | MPU_WRITE_THROUGH
                              | MPU_TRANSIENT | MPU_NO_ALLOCATE;

  HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
