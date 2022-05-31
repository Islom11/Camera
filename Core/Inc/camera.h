/*
 * camera.h
 *
 *  Created on: Oct 25, 2021
 *      Author: Islom
 */

#ifndef INC_CAMERA_H_
#define INC_CAMERA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dcmi.h"


#define CAMERA_WRITE (0x21 << 1)
#define CAMERA_READ (0x21 << 1 | 1)


HAL_StatusTypeDef Camera_init(I2C_HandleTypeDef *_hi2c);
HAL_StatusTypeDef Camera_write(uint8_t reg_adres, uint8_t data);
HAL_StatusTypeDef Camera_config();
void Camera_stop();
HAL_StatusTypeDef Camera_startCap(uint32_t *buff);

#ifdef __cplusplus
}
#endif

#endif /* INC_CAMERA_H_ */
