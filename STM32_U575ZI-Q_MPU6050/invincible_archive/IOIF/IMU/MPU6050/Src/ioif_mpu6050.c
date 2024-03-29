/**
 *-----------------------------------------------------------
 *                 6AXIS ACC & GYRO IMU DRIVER
 *-----------------------------------------------------------
 * @file ioif_mpu6050.c
 * @date Created on: Mar 28, 2023
 * @author INVINCIBLE 1NE
 * @brief Driver code for the MPU6050 accelerometer and gyroscope.
 *
 * This source file provides functionality to interface
 * with the MPU6050 accelerometer and gyroscope, including initialization,
 * data retrieval, and control register configurations.
 * 
 * Refer to the MPU6050 datasheet and related documents for more information.
 *
 * @ref MPU6050 Datasheet
 */

#include "ioif_mpu6050.h"

/** @defgroup I2C I2C
  * @brief I2C MPU6050 module driver
  * @{
  */

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */




/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

/* DMA region setting */
static uint8_t mpu6050Dma1RxBuff[IOIF_MPU6050_BUFF_SIZE] __attribute__((section(".i2c2RxBuff"))) = {0};


static MPU6050Obj_t mpu6050Obj;
static MPU6050IOctx_t mpu6050IOctx;
static I2C_HandleTypeDef* i2cHandle;

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static uint8_t IsDevReady(uint16_t devAddr);
static uint8_t ReadReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size);
static uint8_t WriteReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
 * @brief Initialize the 6-axis IMU sensor.
 * @param i2c Enumeration representing the I2C channel to use.
 * @return Status of the initialization operation.
 */
IOIF_6AxisState_t IOIF_Init6Axis(I2C_HandleTypeDef* hi2c)
{
    mpu6050IOctx.IsDevReady = IsDevReady;
    mpu6050IOctx.ReadReg = ReadReg;
    mpu6050IOctx.WriteReg = WriteReg;
    
    i2cHandle = hi2c;
    // for DMA Read
    mpu6050Obj.dataBuff = mpu6050Dma1RxBuff;

    uint8_t status = MPU6050_SetIoctx(&mpu6050Obj, &mpu6050IOctx);
    if (status != MPU6050_STATUS_OK) {
        return status; // Error handling
    }

    // Initialize the MPU6050 object
    // You should have a function for initialization
	status = MPU6050_Init(&mpu6050Obj);
    if (status != MPU6050_STATUS_OK) {
        return status; // Error handling
    }

    return status;
}

/**
 * @brief Retrieve the current values from the 6-axis IMU sensor.
 * @param imuData Pointer to a structure to store the retrieved data.
 * @return Status of the data retrieval operation.
 */
IOIF_6AxisState_t IOIF_Get6AxisValue(IOIF_6AxisData_t* imuData)
{
    // Check for NULL pointer
    if (imuData == NULL) {
        return IOIF_IMU6AXIS_STATUS_ERROR;
    }

    // Initialize acceleration and gyroscope data
	memset(&mpu6050Obj.IMUData, 0, sizeof(mpu6050Obj.IMUData));

    // Get the value from the hardware object and check for errors
    uint8_t status = MPU6050_GetValue(&mpu6050Obj);
    if (status != IOIF_IMU6AXIS_STATUS_OK) {
        return status;
    }

    memcpy(imuData, &mpu6050Obj.IMUData, sizeof(mpu6050Obj.IMUData));

    return status;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

// Checks if the IMU 6-Axis device is ready
static uint8_t IsDevReady(uint16_t devAddr)
{
    return HAL_I2C_IsDeviceReady(i2cHandle, devAddr, IOIF_MPU6050_TRIALS, IOIF_MPU6050_TIMEOUT);
}

// Reads a register from the IMU 6-Axis device using DMA
static uint8_t ReadReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size)
{
    return HAL_I2C_Mem_Read_DMA(i2cHandle, devAddr, regAddr, MPU6050_CONTROL_SIZE, pData, size);
}

// Writes to a register on the IMU 6-Axis device using blocking method
static uint8_t WriteReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size)
{
	return HAL_I2C_Mem_Write(i2cHandle, devAddr, regAddr, MPU6050_CONTROL_SIZE, pData, size, IOIF_MPU6050_TIMEOUT);
}

