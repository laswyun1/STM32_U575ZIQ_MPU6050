/*
 * ioif_vl53l0x.h
 *
 *  Created on: Jun 26, 2024
 *      Author: INVINCIBLE
 */

#ifndef VL53L0X_INC_IOIF_VL53L0X_H_
#define VL53L0X_INC_IOIF_VL53L0X_H_

#include <string.h>

#include "vl53l0x.h"
#include "main.h"
#include "i2c.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_VL53L0X_BUFF_SIZE        4

#define IOIF_VL53L0X_TRIALS           10
#define IOIF_VL53L0X_START_UP_DELAY   10
#define IOIF_VL53L0X_TIMEOUT          1


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 * @brief Enumeration to describe the state of the 6-axis IMU.
 */
typedef enum _IOIF_TOFState_t {
    IOIF_TOF_STATUS_OK = 0,
    IOIF_TOF_STATUS_ERROR,
    IOIF_TOF_STATUS_BUSY,
    IOIF_TOF_STATUS_TIMEOUT,
} IOIF_TOFState_t;

/**
 * @brief Structure to hold the data from the 6-axis IMU.
 */
typedef struct _IOIF_TOFData_t {
	float distance;
} IOIF_TOFData_t;


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

IOIF_TOFState_t IOIF_InitTOF(I2C_HandleTypeDef* i2c);
IOIF_TOFState_t IOIF_GetTOFValue(IOIF_TOFData_t* tofData);


#endif /* VL53L0X_INC_IOIF_VL53L0X_H_ */
