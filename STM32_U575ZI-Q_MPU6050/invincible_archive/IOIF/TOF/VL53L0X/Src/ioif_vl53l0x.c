/*
 * ioif_vl53l0x.c
 *
 *  Created on: Jun 26, 2024
 *      Author: INVINCIBLE
 */


#include "ioif_vl53l0x.h"

/** @defgroup I2C I2C
  * @brief I2C VL53L0X module driver
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
static uint8_t vl53l0xDma1RxBuff[IOIF_VL53L0X_BUFF_SIZE] __attribute__((section(".i2c1RxBuff"))) = {0};


static VL53L0XObj_t vl53l0xObj;
static VL53L0XIOctx_t vl53l0xIOctx;
static I2C_HandleTypeDef* i2cHandle_tof;

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static uint8_t IsDevReady_TOF(uint16_t devAddr);
static uint8_t ReadReg_TOF(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size);
static uint8_t WriteReg_TOF(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size);

static uint8_t ReadRegNoDMA_TOF(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size);


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
IOIF_TOFState_t IOIF_InitTOF(I2C_HandleTypeDef* hi2c)
{
    vl53l0xIOctx.IsDevReady = IsDevReady_TOF;
    vl53l0xIOctx.ReadReg = ReadReg_TOF;
    vl53l0xIOctx.WriteReg = WriteReg_TOF;
    vl53l0xIOctx.ReadRegNoDMA = ReadRegNoDMA_TOF;

    i2cHandle_tof = hi2c;
    // for DMA Read
    vl53l0xObj.dataBuff = vl53l0xDma1RxBuff;

    uint8_t status = VL53L0X_SetIoctx(&vl53l0xObj, &vl53l0xIOctx);
    if (status != VL53L0X_STATUS_OK) {
        return status; // Error handling
    }

    // Initialize the VL53L0X object
    // You should have a function for initialization
	status = VL53L0X_Init(&vl53l0xObj);
    if (status != VL53L0X_STATUS_OK) {
        return status; // Error handling
    }

    return status;
}

/**
 * @brief Retrieve the current values from the 6-axis IMU sensor.
 * @param imuData Pointer to a structure to store the retrieved data.
 * @return Status of the data retrieval operation.
 */
IOIF_TOFState_t IOIF_GetTOFValue(IOIF_TOFData_t* tofData)
{
    // Check for NULL pointer
    if (tofData == NULL) {
        return IOIF_TOF_STATUS_ERROR;
    }

    // Initialize acceleration and gyroscope data
	memset(&vl53l0xObj.TOFData, 0, sizeof(vl53l0xObj.TOFData));

    // Get the value from the hardware object and check for errors
    uint8_t status = VL53L0X_GetValue(&vl53l0xObj);
    if (status != IOIF_TOF_STATUS_OK) {
        return status;
    }

    memcpy(tofData, &vl53l0xObj.TOFData, sizeof(vl53l0xObj.TOFData));

    return status;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

// Checks if the IMU 6-Axis device is ready
static uint8_t IsDevReady_TOF(uint16_t devAddr)
{
    return HAL_I2C_IsDeviceReady(i2cHandle_tof, devAddr, IOIF_VL53L0X_TRIALS, IOIF_VL53L0X_TIMEOUT);
}

// Reads a register from the IMU 6-Axis device using DMA
static uint8_t ReadReg_TOF(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size)
{
    return HAL_I2C_Mem_Read_DMA(i2cHandle_tof, devAddr, regAddr, VL53L0X_CONTROL_SIZE, pData, size);
}

// Writes to a register on the IMU 6-Axis device using blocking method
static uint8_t WriteReg_TOF(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size)
{
	return HAL_I2C_Mem_Write(i2cHandle_tof, devAddr, regAddr, VL53L0X_CONTROL_SIZE, pData, size, IOIF_VL53L0X_TIMEOUT);
}


// Reads a register from the IMU 6-Axis device using DMA
static uint8_t ReadRegNoDMA_TOF(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size)
{
    return HAL_I2C_Mem_Read(i2cHandle_tof, devAddr, regAddr, VL53L0X_CONTROL_SIZE, pData, size, IOIF_VL53L0X_TIMEOUT);
}

