/*
 * ioif_bmp280.h
 *
 *  Created on: Jun 25, 2024
 *      Author: INVINCIBLE
 */

#ifndef BMP280_INC_IOIF_BMP280_H_
#define BMP280_INC_IOIF_BMP280_H_


#include <string.h>

#include "bmp280.h"
#include "main.h"
#include "i2c.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_BMP280_BUFF_SIZE        4

#define IOIF_BMP280_TRIALS           10
#define IOIF_BMP280_START_UP_DELAY   10
#define IOIF_BMP280_TIMEOUT          1


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 * @brief Enumeration to describe the state of the 6-axis IMU.
 */
typedef enum _IOIF_BPSState_t {
    IOIF_BPS_STATUS_OK = 0,
    IOIF_BPS_STATUS_ERROR,
    IOIF_BPS_STATUS_BUSY,
    IOIF_BPS_STATUS_TIMEOUT,
} IOIF_BPSState_t;

/**
 * @brief Structure to hold the data from the 6-axis IMU.
 */
typedef struct _IOIF_BPSData_t {
	float pressure;
    float temp;
} IOIF_BPSData_t;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */




/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

IOIF_BPSState_t IOIF_InitBPS(I2C_HandleTypeDef* i2c);
IOIF_BPSState_t IOIF_GetBPSValue(IOIF_BPSData_t* bpsData);


#endif /* BMP280_INC_IOIF_BMP280_H_ */
