/*
 * ModeBatt.h
 *
 *  Created on: 27 Nov 2020
 *      Author: jerem
 */

#include "BMS_Management.h"
#include "gpio.h"


#ifndef SRC_MODEBATT_H_
#define SRC_MODEBATT_H_



#define GPIO_MODE0 GPIOA
#define PIN_MODE0 GPIO_PIN_12

#define GPIO_MODE1 GPIOA
#define PIN_MODE1 GPIO_PIN_11

#define GPIO_LOAD GPIOA
#define PIN_LOAD GPIO_PIN_10

#define GPIO_RESET GPIOA
#define PIN_RESET GPIO_PIN_9


#define GPIO_ACK GPIOC
#define PIN_ACK GPIO_PIN_8

#define TIMEOUT 800

typedef enum{
	MODE_PARALLELE,
	MODE_SERIE,
	MODE_BATT1,
	MODE_BATT2,
}t_mode;

extern TIM_HandleTypeDef htim6;

/*
 * @brief: This function configure the switches to put
 * the two batteries in series. The information is sent
 * to the CPLD IO inputs.
 * @param: mode - le mode que l'on souhaite avoir ; nb_batt la batterie que l'on souhaite charger
 * @retval: None
 * */
int ModeBatt_setMode(t_mode mode);


/*
 * @brief: This function configures the switches to reset the system.
 * It happens after an overload has occured. The information is sent
 * to the CPLD IO inputs.
 * @retval: 1 :success
 * 			0 : communication failure
 * */
int ModeBatt_resetMode(void);


/*
 * @brief: This function retrieves, from the CPLD, the information
 * saying if the configuration mode is correct.
 * @param: None
 * @retval: 0 if 'Correct' OR 1 if there is a problem
 * */
int ModeBatt_checkModeBattery(void);

#endif /* SRC_MODEBATT_H_ */
