/*
 * ioif_bmp280.c
 *
 *  Created on: Jun 25, 2024
 *      Author: INVINCIBLE
 */


#include "ioif_bmp280.h"

/** @defgroup I2C I2C
  * @brief I2C BMP280 module driver
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
static uint8_t bmp280Dma1RxBuff[IOIF_BMP280_BUFF_SIZE] __attribute__((section(".i2c4RxBuff"))) = {0};


static BMP280Obj_t bmp280Obj;
static BMP280IOctx_t bmp280IOctx;
static I2C_HandleTypeDef* i2cHandle_bps;

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static uint8_t IsDevReady_BPS(uint16_t devAddr);
static uint8_t ReadReg_BPS(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size);
static uint8_t WriteReg_BPS(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size);


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
IOIF_BPSState_t IOIF_InitBPS(I2C_HandleTypeDef* hi2c)
{
    bmp280IOctx.IsDevReady = IsDevReady_BPS;
    bmp280IOctx.ReadReg = ReadReg_BPS;
    bmp280IOctx.WriteReg = WriteReg_BPS;

    i2cHandle_bps = hi2c;
    // for DMA Read
    bmp280Obj.dataBuff = bmp280Dma1RxBuff;

    uint8_t status = BMP280_SetIoctx(&bmp280Obj, &bmp280IOctx);
    if (status != BMP280_STATUS_OK) {
        return status; // Error handling
    }

    // Initialize the BMP280 object
    // You should have a function for initialization
	status = BMP280_Init(&bmp280Obj);
    if (status != BMP280_STATUS_OK) {
        return status; // Error handling
    }

    return status;
}

/**
 * @brief Retrieve the current values from the 6-axis IMU sensor.
 * @param imuData Pointer to a structure to store the retrieved data.
 * @return Status of the data retrieval operation.
 */
IOIF_BPSState_t IOIF_GetBPSValue(IOIF_BPSData_t* bpsData)
{
    // Check for NULL pointer
    if (bpsData == NULL) {
        return IOIF_BPS_STATUS_ERROR;
    }

    // Initialize acceleration and gyroscope data
	memset(&bmp280Obj.BPSData, 0, sizeof(bmp280Obj.BPSData));

    // Get the value from the hardware object and check for errors
    uint8_t status = BMP280_GetValue(&bmp280Obj);
    if (status != IOIF_BPS_STATUS_OK) {
        return status;
    }

    memcpy(bpsData, &bmp280Obj.BPSData, sizeof(bmp280Obj.BPSData));

    return status;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

// Checks if the IMU 6-Axis device is ready
static uint8_t IsDevReady_BPS(uint16_t devAddr)
{
    return HAL_I2C_IsDeviceReady(i2cHandle_bps, devAddr, IOIF_BMP280_TRIALS, IOIF_BMP280_TIMEOUT);
}

// Reads a register from the IMU 6-Axis device using DMA
static uint8_t ReadReg_BPS(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size)
{
    return HAL_I2C_Mem_Read_DMA(i2cHandle_bps, devAddr, regAddr, BMP280_CONTROL_SIZE, pData, size);
}

// Writes to a register on the IMU 6-Axis device using blocking method
static uint8_t WriteReg_BPS(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size)
{
	return HAL_I2C_Mem_Write(i2cHandle_bps, devAddr, regAddr, BMP280_CONTROL_SIZE, pData, size, IOIF_BMP280_TIMEOUT);
}



