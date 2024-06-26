/*
 * vl53l0x_regmap.h
 *
 *  Created on: Jun 26, 2024
 *      Author: INVINCIBLE
 */

#ifndef TOF_VL53L0X_INC_VL53L0X_REGMAP_H_
#define TOF_VL53L0X_INC_VL53L0X_REGMAP_H_

/* Device Address */
// Device address configurations for VL53L0X
#define VL53L0X_DEV_ADDR_WRITE					0x52U
#define VL53L0X_DEV_ADDR_READ					0x53U

#define VL53L0X_CONTROL_SIZE					1U

#define VL53L0X_DISTANCE_H						0x1EU
#define VL53L0X_DISTANCE_L						0x1FU


/* USED configuration setting values */
#define VL53L0X_REG_SYSRANGE_START_VALUE		0x00U
#define VL53L0X_REG_PWR_MGMT_VALUE				0x01U
#define VL53L0X_REG_INTERNAL_TUNING_2_VALUE		0X01U


/* Register Address */
#define VL53L0X_REG_SYSRANGE_START				0x00U
#define VL53L0X_REG_PWR_MGMT					0x80U
#define VL53L0X_REG_INTERNAL_TUNING_2			0xFFU

#define VL53L0X_REG_RESULT_RANGE_STATUS			0x14U


#endif /* TOF_VL53L0X_INC_VL53L0X_REGMAP_H_ */
