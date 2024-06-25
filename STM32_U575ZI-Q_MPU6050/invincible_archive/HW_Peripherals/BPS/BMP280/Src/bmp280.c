/*
 * bmp280.c
 *
 *  Created on: Jun 25, 2024
 *      Author: INVINCIBLE
 */


#include "bmp280.h"

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

// static uint8_t ReadReg(BMP280Obj_t* bmp280Obj, uint16_t reg, uint8_t* pData, uint32_t size);
static uint8_t WriteReg(BMP280Obj_t* bmp280Obj, uint16_t reg, uint8_t* pData, uint32_t size);
static uint8_t SetReg(BMP280Obj_t* bmp280Obj, uint16_t reg, uint8_t* pData, uint32_t size);

static void CalValues(BMP280Obj_t* bmp280Obj);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
 * @brief Set the I/O context for the BMP280 object.
 * @param bmp280Obj Pointer to the BMP280 object.
 * @param bmp280IOctx Pointer to the I/O context structure.
 * @return BMP280 status code.
 */
uint8_t BMP280_SetIoctx(BMP280Obj_t* bmp280Obj, BMP280IOctx_t* bmp280IOctx)
{
    if (!bmp280Obj || !bmp280IOctx->IsDevReady || !bmp280IOctx->ReadReg || !bmp280IOctx->WriteReg) {
        return BMP280_STATUS_ERROR; // Return error if any pointer is NULL or essential functions are missing
    }

    // Assign IO context functions
    bmp280Obj->io.Init       = bmp280IOctx->Init;
    bmp280Obj->io.DeInit     = bmp280IOctx->DeInit;
    bmp280Obj->io.IsDevReady = bmp280IOctx->IsDevReady;
    bmp280Obj->io.ReadReg    = bmp280IOctx->ReadReg;
    bmp280Obj->io.WriteReg   = bmp280IOctx->WriteReg;
    bmp280Obj->io.Wait       = bmp280IOctx->Wait;

    return BMP280_STATUS_OK;
}

/**
 * @brief Initialize the BMP280 object by setting control registers.
 * @param bmp280Obj Pointer to the BMP280 object.
 * @return BMP280 status code.
 */
uint8_t BMP280_Init(BMP280Obj_t* bmp280Obj)
{
    if (!bmp280Obj) {
        return BMP280_STATUS_ERROR; // Return error if any pointer is NULL or essential functions are missing
    }

    uint8_t status = BMP280_STATUS_ERROR;
    bmp280Obj->devAddr = BMP280_DEV_ADDR_WRITE;

    // Check device readiness
    if (bmp280Obj->io.IsDevReady(bmp280Obj->devAddr) == 0){
        status = BMP280_STATUS_OK;
    }

    // Configure control registers if the device is ready
    if (status == BMP280_STATUS_OK){
        uint8_t conf_1 = BMP280_REG_CONFIG_VALUE;
        status = SetReg(bmp280Obj, BMP280_REG_CONFIG, &conf_1, BMP280_CONTROL_SIZE);
        uint8_t conf_2 = BMP280_REG_CTRL_MEAS_VALUE;
        status = SetReg(bmp280Obj, BMP280_REG_CTRL_MEAS, &conf_2, BMP280_CONTROL_SIZE);
    }

    /* Change the device address to read */
    bmp280Obj->devAddr = BMP280_DEV_ADDR_READ;

    return status;
}

/**
 * @brief Retrieves the raw data from the BMP280 sensor and converts it into scaled acceleration and gyroscope values.
 *
 * This function reads the raw data bytes from the BMP280 sensor using the provided ReadReg function pointer.
 * The raw data is then passed to the CalValues function to be converted into scaled values for acceleration
 * and gyroscope, which are assigned to the corresponding members of the BMP280 object.
 *
 * @param bmp280Obj Pointer to the BMP280 object containing the configuration, IO context, and data structure.
 * @return BMP280_STATUS_OK if the operation is successful; BMP280_STATUS_ERROR otherwise.
 */
uint8_t BMP280_GetValue(BMP280Obj_t* bmp280Obj)
{
    // Check for NULL pointer
    if (!bmp280Obj) {
        return BMP280_STATUS_ERROR; // Return error if any pointer is NULL or essential functions are missing
    }

    // Read the raw data from the sensor
    uint8_t status = bmp280Obj->io.ReadReg(bmp280Obj->devAddr, BMP280_REG_PRESS_MSB, bmp280Obj->dataBuff, BMP280_READ_DATA_SIZE);

    // If the read operation was successful, scale and assign the data
    if (status == BMP280_STATUS_OK) {
        CalValues(bmp280Obj);
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
 * @brief Read data from the specified register of the BMP280 device.
 * @param bmp280Obj Pointer to the BMP280 object.
 * @param reg Register devAddress to read.
 * @param data Pointer to store the read data.
 * @param size Size of the data to be read.
 * @return BMP280 status code.
 */
// static uint8_t ReadReg(BMP280Obj_t* bmp280Obj, uint16_t reg, uint8_t* pData, uint32_t size)
// {
//    return bmp280Obj->io.ReadReg(bmp280Obj->devAddr, reg, pData, size);
// }

/**
 * @brief Write data to the specified register of the BMP280 device.
 * @param bmp280Obj Pointer to the BMP280 object.
 * @param reg Register devAddress to write.
 * @param data Pointer to the data to write.
 * @param size Size of the data to be written.
 * @return BMP280 status code.
 */
static uint8_t WriteReg(BMP280Obj_t* bmp280Obj, uint16_t reg, uint8_t* pData, uint32_t size)
{
    return bmp280Obj->io.WriteReg(bmp280Obj->devAddr, reg, pData, size);
}

/**
 * @brief Set and verify the specified register of the BMP280 device.
 * @param bmp280Obj Pointer to the BMP280 object.
 * @param reg Register devAddress to set.
 * @param data Pointer to the data to set.
 * @param size Size of the data to be set.
 * @return BMP280 status code.
 */
static uint8_t SetReg(BMP280Obj_t* bmp280Obj, uint16_t reg, uint8_t* pData, uint32_t size)
{
	uint8_t status = BMP280_STATUS_OK;

    status = WriteReg(bmp280Obj, reg, pData, size);

    return status;
}

/**
 * @brief Scales the raw data from the BMP280 sensor and assigns it to the appropriate structure members.
 *
 * This function takes the raw data bytes from the BMP280 sensor, converts them to 16-bit signed integers,
 * scales them according to the defined scale factors for acceleration and gyroscope, and assigns them
 * to the corresponding members of the BMP280 object.
 *
 * @param bmp280Obj Pointer to the BMP280 object containing the data structure to be updated.
 * @param dataBuff Pointer to the buffer containing the raw data bytes from the sensor.
 */
static void CalValues(BMP280Obj_t* bmp280Obj)
{
    // Array to hold the raw 16-bit values for each sensor reading
    uint16_t rawValues[BMP280_TOTAL_SENSORS] = {0};

    // Convert the raw bytes to 16-bit signed integers
    for (int i = 0; i < BMP280_TOTAL_SENSORS; i++) {
        rawValues[i] = (uint16_t)(bmp280Obj->dataBuff[i * 2] << 8 | bmp280Obj->dataBuff[i * 2 + 1]);
    }

    // Scale the raw values and assign them to the appropriate structure members
    bmp280Obj->BPSData.pressure = (float)(rawValues[0]);
}


