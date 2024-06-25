/*
 * bmp280.h
 *
 *  Created on: Jun 25, 2024
 *      Author: INVINCIBLE
 */

#ifndef BMP280_INC_BMP280_H_
#define BMP280_INC_BMP280_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "bmp280_regmap.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define BMP280_DATA_BUFF_SIZE    4
#define BMP280_READ_DATA_SIZE    2

#define BMP280_TOTAL_SENSORS     1

#define BMP280_TRIALS            10
#define BMP280_START_UP_DELAY    10
#define BMP280_TIMEOUT           1


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

// Function pointer types for initialization, de-initialization, readiness, etc.
typedef uint8_t (*BMP280_InitFunc)       (void);
typedef uint8_t (*BMP280_DeInitFunc)     (void);
typedef uint8_t (*BMP280_IsDevReadyFunc) (uint16_t);                                  // DevAddr
typedef uint8_t (*BMP280_ReadRegFunc)    (uint16_t, uint16_t, uint8_t*, uint16_t);    // DevAddr, RegAddr, Data, DataSize
typedef uint8_t (*BMP280_WriteRegFunc)   (uint16_t, uint16_t, uint8_t*, uint16_t);    // DevAddr, RegAddr, Data, DataSize
typedef uint8_t (*BMP280_WaitFunc)       (uint32_t);			                      // ms to Wait

// Enumeration for BMP280 status codes.
typedef enum _BMP280State_t {
    BMP280_STATUS_OK = 0,
    BMP280_STATUS_ERROR,
    BMP280_STATUS_BUSY,
    BMP280_STATUS_TIMEOUT,
} BMP280State_t;

// Structure for IO context
typedef struct _BMP280IOctx_t {
    BMP280_InitFunc       Init;
    BMP280_DeInitFunc     DeInit;
    BMP280_IsDevReadyFunc IsDevReady;
    BMP280_ReadRegFunc    ReadReg;
    BMP280_WriteRegFunc   WriteReg;
    BMP280_WaitFunc       Wait;
} BMP280IOctx_t;

// Structure for sensor values
typedef struct _BMP280Data_t {
    float pressure;
    float temp;
} BMP280Data_t;

// Object handle structure
typedef struct _BMP280Obj_t {
    uint16_t devAddr;              // Device address
    uint8_t isInit;                // Initialization status
    uint8_t* dataBuff;             // Data Buffer
    BMP280Data_t BPSData;          // Pressure, Temperature Data
    BMP280IOctx_t io;              // IO context
} BMP280Obj_t;


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

// Set the I/O context for the BMP280 object.
uint8_t BMP280_Init(BMP280Obj_t* bmp280_obj);

// Initialize the BMP280 object.
uint8_t BMP280_SetIoctx(BMP280Obj_t* bmp280_obj, BMP280IOctx_t* bmp280_ioctx);

// Retrieve the acclerometer and gyroscope data values from the BMP280 object.
uint8_t BMP280_GetValue(BMP280Obj_t* bmp280_obj);


#endif /* BMP280_INC_BMP280_H_ */
