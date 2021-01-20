/*
 * InfoBatt.h
 *
 *  Created on: 27 Nov 2020
 *      Author: jerem
 */
#include "BMS_Management.h" //a retirer si c'est l'inverse
#ifndef SRC_INFOBATT_H_
#define SRC_INFOBATT_H_

/*
 *	BATTERY 1 & 2
 */

/*
 * @brief: The user can access to the current value in
 * the battery 1 thanks to the gauge 1
 * @param: Battery ID
 * @retval: The value of the current in the battery 1
 * */
float InfoBatt_getCurrent(uint8_t nb_batt);


/*
 * @brief: The user can access to the voltage value in
 * the battery 1 thanks to the gauge 1
 * @param: Battery ID
 * @retval: The value of the voltage in the battery 1
 * */
float InfoBatt_getVoltage(uint8_t nb_batt);


/*
 * @brief: The user can access to the state of charge (Soc)
 * value in the battery 1 thanks to the gauge 1
 * @param: Battery ID
 * @retval: The value of the Soc in the battery 1
 * */
float InfoBatt_getSoc(uint8_t nb_batt);


/*
 * @brief: The user can set the initial value of
 * SOC for the battery
 * @param: Initial value of SOC, Battery ID
 * @retval: HAL_StatusTypeDef Success (HAL_OK) or not (HAL_ERROR or HAL_BUSY)
 * */
HAL_StatusTypeDef setSOC(uint8_t nb_batt, float initValue);


//Fonctions supports

/*
 * @brief: The user can use this function to get
 * the last hexadecimal value of the SOC. The user cannot
 * use the function InforBatt_getSoc to return the
 * last hexadecimal value
 * @param: Battery ID
 * @retval: uint16_t hexadecimal value of the SOC
 */
uint16_t saveSOC(uint8_t nb_batt);

/*
 * @brief: The user can add this function to check
 *at every loop if the gauge is still connected through
 *the I²C channel to the µC.
 *@param: Battery ID
 *@retval: HAL_StatusTypeDef HAL_OK if connected.
 */
HAL_StatusTypeDef checkLink(uint8_t nb_batt);

#endif /* SRC_INFOBATT_H_ */
