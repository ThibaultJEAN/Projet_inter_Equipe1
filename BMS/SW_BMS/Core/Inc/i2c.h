/**
  ******************************************************************************
  * File Name          : I2C.h
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN Private defines */
static const uint8_t GAUGE_ADDR=0x64<<1;		//adresse i2c du LTC2944
static const uint8_t CTRL_REG=0x01; 			//register B
static const uint8_t AUTO_MODE=0xE8; 			//data + prescaler à 1024
static const uint8_t SHUTDOWN=0xE9; 			//shutdown au registre B sans écraser
static const uint8_t VOLT_REG=0x08;				//registre MSB pour la lecture de tension
static const uint8_t AMP_REG=0x0E;				//registre MSB pour la lecture du courant (attention, il n'y a pas de val abs)
static const uint8_t COUL_REG=0x02;				//registre MSB pour la lecture du SOC (il faut bien penser à l'init avec setSOC)

I2C_HandleTypeDef BATTERY1;
I2C_HandleTypeDef BATTERY2;
/* USER CODE END Private defines */

void MX_I2C1_Init(void);
void MX_I2C2_Init(void);



/* USER CODE BEGIN Prototypes */

/*
 *@brief: This function allows the user to get a 2-byte data from the MSB register in the LTC2944
 *@param: hi2c -> BATTERY1 or BATTERY2 ; REG_MSB -> data's register ; retVal -> 2-byte-buffer for the data
 *@retval: HAL_StatusTypeDef -> To deal with error messages (i.e Error if retval==HAL_ERROR or HAL_BUSY)
 */
HAL_StatusTypeDef read_register(I2C_HandleTypeDef hi2c, uint8_t REG_MSB,uint16_t retVal);

/*
 *@brief: This function allows the user to write a 1-byte data starting from the MSB register in the LTC2944 device
 *@param: hi2c -> BATTERY1 or BATTERY2 ; register_pointer -> data's register ; register_value -> value we want to put in the register
 *@retval: HAL_StatusTypeDef -> To deal with error messages (i.e Error if retval==HAL_ERROR or HAL_BUSY)
 */
HAL_StatusTypeDef Write_Register(I2C_HandleTypeDef hi2c, uint8_t register_pointer, uint8_t register_value);




#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT GEI Corp *****END OF FILE****/
