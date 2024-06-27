/*
 * vl53l0x.h
 *
 *  Created on: Jun 26, 2024
 *      Author: INVINCIBLE
 */

#ifndef TOF_VL53L0X_INC_VL53L0X_H_
#define TOF_VL53L0X_INC_VL53L0X_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "vl53l0x_regmap.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define VL53L0X_DATA_BUFF_SIZE    4
#define VL53L0X_READ_DATA_SIZE    2

#define VL53L0X_TOTAL_SENSORS     1

#define VL53L0X_TRIALS            10
#define VL53L0X_START_UP_DELAY    10
#define VL53L0X_TIMEOUT           1


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

// Function pointer types for initialization, de-initialization, readiness, etc.
typedef uint8_t (*VL53L0X_InitFunc)       (void);
typedef uint8_t (*VL53L0X_DeInitFunc)     (void);
typedef uint8_t (*VL53L0X_IsDevReadyFunc) (uint16_t);                                  // DevAddr
typedef uint8_t (*VL53L0X_ReadRegFunc)    (uint16_t, uint16_t, uint8_t*, uint16_t);    // DevAddr, RegAddr, Data, DataSize
typedef uint8_t (*VL53L0X_WriteRegFunc)   (uint16_t, uint16_t, uint8_t*, uint16_t);    // DevAddr, RegAddr, Data, DataSize
typedef uint8_t (*VL53L0X_WaitFunc)       (uint32_t);			                      // ms to Wait

typedef uint8_t (*VL53L0X_ReadRegNoDMAFunc)    (uint16_t, uint16_t, uint8_t*, uint16_t);    // DevAddr, RegAddr, Data, DataSize

// Enumeration for VL53L0X status codes.
typedef enum _VL53L0XState_t {
    VL53L0X_STATUS_OK = 0,
    VL53L0X_STATUS_ERROR,
    VL53L0X_STATUS_BUSY,
    VL53L0X_STATUS_TIMEOUT,
} VL53L0XState_t;

// Structure for IO context
typedef struct _VL53L0XIOctx_t {
    VL53L0X_InitFunc       Init;
    VL53L0X_DeInitFunc     DeInit;
    VL53L0X_IsDevReadyFunc IsDevReady;
    VL53L0X_ReadRegFunc    ReadReg;
    VL53L0X_WriteRegFunc   WriteReg;
    VL53L0X_WaitFunc       Wait;

    VL53L0X_ReadRegNoDMAFunc   ReadRegNoDMA;
} VL53L0XIOctx_t;

// Structure for sensor values
typedef struct _VL53L0XData_t {
    float distance;

} VL53L0XData_t;

// Object handle structure
typedef struct _VL53L0XObj_t {
    uint16_t devAddr;               // Device address
    uint8_t isInit;                 // Initialization status
    uint8_t* dataBuff;              // Data Buffer
    VL53L0XData_t TOFData;          // Distance data
    VL53L0XIOctx_t io;              // IO context
} VL53L0XObj_t;


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

// Set the I/O context for the VL53L0X object.
uint8_t VL53L0X_Init(VL53L0XObj_t* vl53l0x_obj);

// Initialize the VL53L0X object.
uint8_t VL53L0X_SetIoctx(VL53L0XObj_t* vl53l0x_obj, VL53L0XIOctx_t* vl53l0x_ioctx);

// Retrieve the acclerometer and gyroscope data values from the VL53L0X object.
uint8_t VL53L0X_GetValue(VL53L0XObj_t* vl53l0x_obj);



#endif /* TOF_VL53L0X_INC_VL53L0X_H_ */
