/*
 * BMS_Management.h
 *
 *  Created on: 27 Nov 2020
 *      Author: jeremy aubert for GEI Corp
 */

#ifndef SRC_BMS_MANAGEMENT_H_
#define SRC_BMS_MANAGEMENT_H_

#include "i2c.h"
#include "InfoBatt.h"
#include "ModeBatt.h"

/*
 * @brief: Function who will manage batteries configuration
 * according to if we are in the load phase or not.
 * @param: None
 * @retval: None
 *
 * */
void BMSManagement_setModeBatt(void);


/*
 * @brief: When the power source is disconnected, the user
 * can recover a code corresponding to the origin of the
 * problem
 * @param: None.
 * @retval: 0 if low battery problem OR 1 if power overload.
 *
 * */
int BMSManagement_getLastStatus(void); //Niveau SOC


/*
 * @brief: The user can choose which information he wants to access
 * from either the battery 1 or 2
 * @param: info=[I,V,S], num_batt=[BATTERY_1,BATTERY_2]
 * @retval: The value of the current (I), voltage (V), state of
 * charge (S)
 * */
float BMSManagement_getInfo(char info, I2C_HandleTypeDef nb_batt);


void BMSManagement_Init(void);

#endif /* SRC_BMS_MANAGEMENT_H_ */
