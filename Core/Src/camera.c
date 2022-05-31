/*
 * camera.c
 *
 *  Created on: Oct 25, 2021
 *      Author: Islom
 */
#include "camera.h"

#include "camera_config.h"
#include <stdio.h>
I2C_HandleTypeDef *hi2c;
DCMI_HandleTypeDef *hdcmi;

HAL_StatusTypeDef Camera_init(I2C_HandleTypeDef *_hi2c){
	hi2c = _hi2c;
	//hdcmi = _hdcmi;
	return HAL_I2C_Master_Transmit(hi2c, CAMERA_WRITE, 0x00, 1, 100);
	//return Camera_write(0x12, 0x80);
}



HAL_StatusTypeDef Camera_write(uint8_t reg_adres, uint8_t data){
	return HAL_I2C_Mem_Write(hi2c, CAMERA_WRITE, reg_adres, 1, &data, 1, 100);
}

HAL_StatusTypeDef Camera_config()
{
//	Camera_stop();
//  HAL_Delay(30);
//  uint32_t n = sizeof(OV7670_QCIF)/2;
	/*
  for(uint32_t i = 0; i< 122; i++) {
	Camera_write(OV7670_reg[i][0], OV7670_reg[i][1]);

  }
  */
  int i=0;
  while(OV7670_registrs[i][0]!=0xFF || OV7670_registrs[i][1]!=0xFF)
  {
	  Camera_write(OV7670_registrs[i][0], OV7670_registrs[i][1]);
	  ++i;
  }
  return HAL_OK;
}

void Camera_stop()
{
	HAL_DCMI_Stop(hdcmi);
}

HAL_StatusTypeDef Camera_startCap(uint32_t *buff)
{
	Camera_stop();
  return HAL_DCMI_Start_DMA(hdcmi, DCMI_MODE_SNAPSHOT, buff, 176*144/2);
//  HAL_DCMI_Start_DMA(&hdcmi_eval, DCMI_MODE_SNAPSHOT, (uint32_t)buff, GetSize(current_resolution));

//  if (capMode == OV7670_CAP_CONTINUOUS) {
//    /* note: continuous mode automatically invokes DCMI, but DMA needs to be invoked manually */
//    s_destAddressForContiuousMode = destAddress;
//    HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_CONTINUOUS, destAddress, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
//  } else if (capMode == OV7670_CAP_SINGLE_FRAME) {
//    s_destAddressForContiuousMode = 0;
//    HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_SNAPSHOT, destAddress, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
//  }

//  return RET_OK;
}


