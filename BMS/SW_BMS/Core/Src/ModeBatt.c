#include "ModeBatt.h"



int ModeBatt_setMode(t_mode mode)
{
	int ret_val;
	ModeBatt_resetMode(); //reset before doing anything else

	switch (mode){ //switch mode

	case MODE_PARALLELE:
		HAL_GPIO_WritePin(GPIO_MODE0,PIN_MODE0,0);
		HAL_GPIO_WritePin(GPIO_MODE1,PIN_MODE1,0);
		break;

	case MODE_SERIE:
		HAL_GPIO_WritePin(GPIO_MODE0,PIN_MODE0,1);
		HAL_GPIO_WritePin(GPIO_MODE1,PIN_MODE1,1);
		break;

	case MODE_BATT1:
		HAL_GPIO_WritePin(GPIO_MODE0,PIN_MODE0,0);
		HAL_GPIO_WritePin(GPIO_MODE1,PIN_MODE1,1);
		break;

	case MODE_BATT2:
		HAL_GPIO_WritePin(GPIO_MODE0,PIN_MODE0,1);
		HAL_GPIO_WritePin(GPIO_MODE1,PIN_MODE1,0);
		break;

	}
	HAL_GPIO_WritePin(GPIO_LOAD,PIN_LOAD,1); //LOAD pin triggers CPLD
	ret_val= ModeBatt_checkModeBattery();

	HAL_GPIO_WritePin(GPIO_LOAD,PIN_LOAD,0);
	return ret_val;


}


int ModeBatt_resetMode(void)
{
	int ret_val;

	//do your thing mate
	HAL_GPIO_WritePin(GPIO_RESET,PIN_RESET,1);
	//lancer timer

	ret_val =ModeBatt_checkModeBattery();
	HAL_GPIO_WritePin(GPIO_RESET,PIN_RESET,0);
	return ret_val;
}


int ModeBatt_checkModeBattery(void)
{
	//do your thing mate
	/*
	 * Faire un while on reçoit pas un ACK (1) ou que l'on a pas encore dépassé le timer
	 * Si on a dépassé le timer sans avoir un ACK : on renvoit un message d'erreur
	 *
	 */
	HAL_TIM_Base_Start(&htim6); //normalement on écrit cette ligne dans le main puisqu'on ne l'écrit qu'une fois mais je savais pas où la mettre sinon
	__HAL_TIM_SET_COUNTER(&htim6,0);  // set the counter value a 0
	//wait for ACK or Timeout
	while ((HAL_GPIO_ReadPin(GPIO_ACK,PIN_ACK)!=1)||__HAL_TIM_GET_COUNTER(&htim6)<TIMEOUT) {}  //TIMEOUT en ms

			if (HAL_GPIO_ReadPin(GPIO_ACK,PIN_ACK)==1) {
				return 1;
			}
			else { //erreur de com avec le CPLD
				return 0;
			}
}
