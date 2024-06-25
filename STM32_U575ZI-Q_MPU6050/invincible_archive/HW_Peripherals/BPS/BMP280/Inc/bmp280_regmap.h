/*
 * bmp280_regmap.h
 *
 *  Created on: Jun 25, 2024
 *      Author: INVINCIBLE
 */

#ifndef BMP280_INC_BMP280_REGMAP_H_
#define BMP280_INC_BMP280_REGMAP_H_

/* Device Address */
// Device address configurations for BMP280
#define BMP280_DEV_ADDR_WRITE 				0b11101100
#define BMP280_DEV_ADDR_READ				0b11101101

/* USED configuration setting values */
#define BMP280_REG_CONFIG_VALUE			    0b01100000		// For 0xF5 register
#define BMP280_REG_CTRL_MEAS_VALUE			0b01101111		// For 0xF4 register
#define BMP280_CONTROL_SIZE					1U


/* Register Address */
#define BMP280_REG_TEMP_XLSB				0xFCU	// bits : 7 ~ 4
#define BMP280_REG_TEMP_LSB 				0xFBU
#define BMP280_REG_TEMP_MSB					0xFAU

#define BMP280_REG_PRESS_XLSB				0xF9U
#define BMP280_REG_PRESS_LSB				0xF8U
#define BMP280_REG_PRESS_MSB				0xF7U

#define BMP280_REG_CONFIG					0xF5U	// bits: 7-5 [t_sb], bits: 4-2 [filter], bits: 0 [spi3w_en]
#define BMP280_REG_CTRL_MEAS				0xF4U   // bits: 7-5 [osrs_t], bits: 4-2 [osrs_p], bits: 1-0 [mode]
#define BMP280_REG_STATUS					0xF3U   // bits: 3 [measuring], bits: 0 [im_update]

#define BMP280_REG_RESET					0xE0U
#define BMP280_REG_RESET_VALUE			    0xB6U

#endif /* BMP280_INC_BMP280_REGMAP_H_ */
