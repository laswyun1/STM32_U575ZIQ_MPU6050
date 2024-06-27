/*
 * vl53l0x.c
 *
 *  Created on: Jun 26, 2024
 *      Author: INVINCIBLE
 */



#include "vl53l0x.h"

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




/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

// static uint8_t ReadReg(VL53L0XObj_t* vl53l0xObj, uint16_t reg, uint8_t* pData, uint32_t size);
static uint8_t WriteReg(VL53L0XObj_t* vl53l0xObj, uint16_t reg, uint8_t* pData, uint32_t size);
static uint8_t SetReg(VL53L0XObj_t* vl53l0xObj, uint16_t reg, uint8_t* pData, uint32_t size);
static uint8_t GetReg(VL53L0XObj_t* vl53l0xObj, uint16_t reg, uint8_t* pData, uint32_t size);

static void CalValues(VL53L0XObj_t* vl53l0xObj);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
 * @brief Set the I/O context for the VL53L0X object.
 * @param vl53l0xObj Pointer to the VL53L0X object.
 * @param vl53l0xIOctx Pointer to the I/O context structure.
 * @return VL53L0X status code.
 */
uint8_t VL53L0X_SetIoctx(VL53L0XObj_t* vl53l0xObj, VL53L0XIOctx_t* vl53l0xIOctx)
{
    if (!vl53l0xObj || !vl53l0xIOctx->IsDevReady || !vl53l0xIOctx->ReadReg || !vl53l0xIOctx->WriteReg) {
        return VL53L0X_STATUS_ERROR; // Return error if any pointer is NULL or essential functions are missing
    }

    // Assign IO context functions
    vl53l0xObj->io.Init       = vl53l0xIOctx->Init;
    vl53l0xObj->io.DeInit     = vl53l0xIOctx->DeInit;
    vl53l0xObj->io.IsDevReady = vl53l0xIOctx->IsDevReady;
    vl53l0xObj->io.ReadReg    = vl53l0xIOctx->ReadReg;
    vl53l0xObj->io.WriteReg   = vl53l0xIOctx->WriteReg;
    vl53l0xObj->io.Wait       = vl53l0xIOctx->Wait;
    vl53l0xObj->io.ReadRegNoDMA    = vl53l0xIOctx->ReadRegNoDMA;

    return VL53L0X_STATUS_OK;
}

/**
 * @brief Initialize the VL53L0X object by setting control registers.
 * @param vl53l0xObj Pointer to the VL53L0X object.
 * @return VL53L0X status code.
 */
uint8_t VL53L0X_Init(VL53L0XObj_t* vl53l0xObj)
{
    if (!vl53l0xObj) {
        return VL53L0X_STATUS_ERROR; // Return error if any pointer is NULL or essential functions are missing
    }

    uint8_t status = VL53L0X_STATUS_ERROR;
    vl53l0xObj->devAddr = VL53L0X_DEV_ADDR_WRITE;

    // Check device readiness
    if (vl53l0xObj->io.IsDevReady(vl53l0xObj->devAddr) == 0){
        status = VL53L0X_STATUS_OK;
    }

    // Configure control registers if the device is ready
    if (status == VL53L0X_STATUS_OK){

    	/* (1) Switch to 2V8 mode (Default : 1V8 mode) */
    	uint8_t tempArr1[1] = {0};
    	status = GetReg(vl53l0xObj, 0x89, tempArr1, VL53L0X_CONTROL_SIZE);

    	uint8_t conf_0 = (tempArr1[0] | 0x01);
    	status = SetReg(vl53l0xObj, 0x89, &conf_0, VL53L0X_CONTROL_SIZE);


    	/* (2) Set I2C Standard mode */
        uint8_t conf_1 = 0x00;
        status = SetReg(vl53l0xObj, 0x88, &conf_1, VL53L0X_CONTROL_SIZE);

        uint8_t conf_2 = 0x01;
        status = SetReg(vl53l0xObj, 0x80, &conf_2, VL53L0X_CONTROL_SIZE);
        uint8_t conf_3 = 0x01;
        status = SetReg(vl53l0xObj, 0xFF, &conf_3, VL53L0X_CONTROL_SIZE);
        uint8_t conf_4 = 0x00;
        status = SetReg(vl53l0xObj, 0x00, &conf_4, VL53L0X_CONTROL_SIZE);


        uint8_t tempArr2[1] = {0};
        status = GetReg(vl53l0xObj, 0x91, tempArr2, VL53L0X_CONTROL_SIZE);


        uint8_t conf_5 = 0x01;
        status = SetReg(vl53l0xObj, 0x00, &conf_5, VL53L0X_CONTROL_SIZE);
        uint8_t conf_6 = 0x00;
        status = SetReg(vl53l0xObj, 0xFF, &conf_6, VL53L0X_CONTROL_SIZE);
        uint8_t conf_7 = 0x00;
        status = SetReg(vl53l0xObj, 0x80, &conf_7, VL53L0X_CONTROL_SIZE);


        /* (3) Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks */
    	uint8_t tempArr3[1] = {0};
    	status = GetReg(vl53l0xObj, 0x60, tempArr3, VL53L0X_CONTROL_SIZE);

    	uint8_t conf_8 = (tempArr3[0] | 0x12);
    	status = SetReg(vl53l0xObj, 0x60, &conf_8, VL53L0X_CONTROL_SIZE);
    }

    /* Change the device address to read */
    vl53l0xObj->devAddr = VL53L0X_DEV_ADDR_READ;

    return status;
}

/**
 * @brief Retrieves the raw data from the VL53L0X sensor and converts it into scaled acceleration and gyroscope values.
 *
 * This function reads the raw data bytes from the VL53L0X sensor using the provided ReadReg function pointer.
 * The raw data is then passed to the CalValues function to be converted into scaled values for acceleration
 * and gyroscope, which are assigned to the corresponding members of the VL53L0X object.
 *
 * @param vl53l0xObj Pointer to the VL53L0X object containing the configuration, IO context, and data structure.
 * @return VL53L0X_STATUS_OK if the operation is successful; VL53L0X_STATUS_ERROR otherwise.
 */
uint8_t VL53L0X_GetValue(VL53L0XObj_t* vl53l0xObj)
{
    // Check for NULL pointer
    if (!vl53l0xObj) {
        return VL53L0X_STATUS_ERROR; // Return error if any pointer is NULL or essential functions are missing
    }

    // Read the raw data from the sensor
    uint8_t status = vl53l0xObj->io.ReadReg(vl53l0xObj->devAddr, VL53L0X_DISTANCE_H, vl53l0xObj->dataBuff, VL53L0X_READ_DATA_SIZE);

    // If the read operation was successful, scale and assign the data
    if (status == VL53L0X_STATUS_OK) {
        CalValues(vl53l0xObj);
    }

    return status;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief Read data from the specified register of the VL53L0X device.
 * @param vl53l0xObj Pointer to the VL53L0X object.
 * @param reg Register devAddress to read.
 * @param data Pointer to store the read data.
 * @param size Size of the data to be read.
 * @return VL53L0X status code.
 */
// static uint8_t ReadReg(VL53L0XObj_t* vl53l0xObj, uint16_t reg, uint8_t* pData, uint32_t size)
// {
//    return vl53l0xObj->io.ReadReg(vl53l0xObj->devAddr, reg, pData, size);
// }

/**
 * @brief Write data to the specified register of the VL53L0X device.
 * @param vl53l0xObj Pointer to the VL53L0X object.
 * @param reg Register devAddress to write.
 * @param data Pointer to the data to write.
 * @param size Size of the data to be written.
 * @return VL53L0X status code.
 */
static uint8_t WriteReg(VL53L0XObj_t* vl53l0xObj, uint16_t reg, uint8_t* pData, uint32_t size)
{
    return vl53l0xObj->io.WriteReg(vl53l0xObj->devAddr, reg, pData, size);
}

static uint8_t ReadRegNoDMA(VL53L0XObj_t* vl53l0xObj, uint16_t reg, uint8_t* pData, uint32_t size)
{
    return vl53l0xObj->io.ReadRegNoDMA(vl53l0xObj->devAddr, reg, pData, size);
}

/**
 * @brief Set and verify the specified register of the VL53L0X device.
 * @param vl53l0xObj Pointer to the VL53L0X object.
 * @param reg Register devAddress to set.
 * @param data Pointer to the data to set.
 * @param size Size of the data to be set.
 * @return VL53L0X status code.
 */
static uint8_t SetReg(VL53L0XObj_t* vl53l0xObj, uint16_t reg, uint8_t* pData, uint32_t size)
{
	uint8_t status = VL53L0X_STATUS_OK;

    status = WriteReg(vl53l0xObj, reg, pData, size);

    return status;
}

static uint8_t GetReg(VL53L0XObj_t* vl53l0xObj, uint16_t reg, uint8_t* pData, uint32_t size)
{
	uint8_t status = VL53L0X_STATUS_OK;

    status = ReadRegNoDMA(vl53l0xObj, reg, pData, size);

    return status;
}


/**
 * @brief Scales the raw data from the VL53L0X sensor and assigns it to the appropriate structure members.
 *
 * This function takes the raw data bytes from the VL53L0X sensor, converts them to 16-bit signed integers,
 * scales them according to the defined scale factors for acceleration and gyroscope, and assigns them
 * to the corresponding members of the VL53L0X object.
 *
 * @param vl53l0xObj Pointer to the VL53L0X object containing the data structure to be updated.
 * @param dataBuff Pointer to the buffer containing the raw data bytes from the sensor.
 */
static void CalValues(VL53L0XObj_t* vl53l0xObj)
{
    // Array to hold the raw 16-bit values for each sensor reading
    uint16_t rawValues[VL53L0X_TOTAL_SENSORS] = {0};

    // Convert the raw bytes to 16-bit signed integers
    for (int i = 0; i < VL53L0X_TOTAL_SENSORS; i++) {
        rawValues[i] = (uint16_t)(vl53l0xObj->dataBuff[i * 2] << 8 | vl53l0xObj->dataBuff[i * 2 + 1]);
    }

    // Scale the raw values and assign them to the appropriate structure members
    vl53l0xObj->TOFData.distance = (float)(rawValues[0]);
}


