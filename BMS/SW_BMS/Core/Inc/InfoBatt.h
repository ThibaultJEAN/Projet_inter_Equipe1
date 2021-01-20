/*
 * InfoBatt.h
 *
 *  Created on: 27 Nov 2020
 *      Author: jeremy aubert for GEI Corp
 */
#include "BMS_Management.h"
#include "i2c.h"
#ifndef SRC_INFOBATT_H_
#define SRC_INFOBATT_H_


#define BATTERY1 hi2c1
#define BATTERY2 hi2c2

/*
 *	BATTERY 1 & 2
 */

/*
 * @brief: The user can access to the current value in
 * the battery 1 thanks to the gauge 1
 * @param: Battery ID
 * @retval: The value of the current in the battery 1
 * */
float InfoBatt_getCurrent(I2C_HandleTypeDef nb_batt);


/*
 * @brief: The user can access to the voltage value in
 * the battery 1 thanks to the gauge 1
 * @param: Battery ID
 * @retval: The value of the voltage in the battery 1
 * */
float InfoBatt_getVoltage(I2C_HandleTypeDef nb_batt);


/*
 * @brief: The user can access to the state of charge (Soc)
 * value in the battery 1 thanks to the gauge 1
 * @param: Battery ID
 * @retval: The value of the Soc in the battery 1
 * */
float InfoBatt_getSoc(I2C_HandleTypeDef nb_batt);


/*
 * @brief: The user can set the initial value of
 * SOC for the battery
 * @param: Initial value of SOC, Battery ID
 * @retval: HAL_StatusTypeDef Success (HAL_OK) or not (HAL_ERROR or HAL_BUSY)
 * */
HAL_StatusTypeDef setSoc(I2C_HandleTypeDef nb_batt, uint16_t initValue);


//Fonctions supports

/*
 * @brief: The user can use this function to get
 * the last hexadecimal value of the SOC. The user cannot
 * use the function InforBatt_getSoc to return the
 * last hexadecimal value
 * @param: Battery ID
 * @retval: uint16_t hexadecimal value of the SOC
 */
uint16_t saveSOC(I2C_HandleTypeDef nb_batt);

/*
 * @brief: The user can add this function to check
 *at every loop if the gauge is still connected through
 *the I²C channel to the µC.
 *@param: Battery ID
 *@retval: HAL_StatusTypeDef HAL_OK if connected.
 */
HAL_StatusTypeDef checkLink(I2C_HandleTypeDef nb_batt);

#endif /* SRC_INFOBATT_H_ */
