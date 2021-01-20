/*
 * BMS_Management.c
 *
 *  Created on: 27 Nov 2020
 *      Author: jerem
 */

#include "BMS_Management.h"


void BMSManagement_setModeBatt(t_mode mode)
{
	ModeBatt_setMode(mode);
}

int BMSManagement_getLastStatus(void)
{
	int val=0;
	float cmp;

	val=ModeBatt_checkModeBattery();

	cmp=BMSManagement_getInfo('S',BATTERY1);
	if (cmp>17.001) 							//17.001 Ah - valeur maximale de charge dans la batterie (/17Ah)
		val=1;
	else if (cmp<10.0)							//10.0 Ah - valeur minimale de charge dans la batterie (/17Ah)
		val=1;

	cmp=BMSManagement_getInfo('S',BATTERY2);
	if (cmp>17.001)
		val=1;
	else if (cmp<10.0)
		val=1;
	return val;
}


float BMSManagement_getInfo(char info, I2C_HandleTypeDef nb_batt)
{
	float val;
	switch (info)
	{
		case 'V':
			val=InfoBatt_getVoltage(nb_batt);
			break;
		case 'I':
			val=InfoBatt_getCurrent(nb_batt);
			break;
		case 'S':
			val=InfoBatt_getSoc(nb_batt);
			break;
	}
	return val;
}


void BMSManagement_Init(void) 						//Error management to add!
{
	HAL_StatusTypeDef ret;
	while (HAL_I2C_IsDeviceReady(&BATTERY1,GAUGE_ADDR,1,100)!=HAL_OK)
	{
	  //strcpy((char*)msg,"I2C Device not ready.\r\n");
	  //HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY); //Attention aux includes de l'UART et strcpy
	  HAL_Delay(5);
	}
	ret=Write_Register(BATTERY1, CTRL_REG,AUTO_MODE); //I2C Master Transmit
	if (ret!=HAL_OK)
	{
		//strcpy((char*)msg,"Control Register Automatic Mode Error.\r\n");
		//HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
		HAL_Delay(5);
	}


	while (HAL_I2C_IsDeviceReady(&BATTERY2,GAUGE_ADDR,1,100)!=HAL_OK)
		{
		  //strcpy((char*)msg,"I2C Device not ready.\r\n");
		  //HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY); //Attention aux includes de l'UART et strcpy
		  HAL_Delay(5);
		}
		ret=Write_Register(BATTERY2, CTRL_REG,AUTO_MODE); //I2C Master Transmit
		if (ret!=HAL_OK)
		{
			//strcpy((char*)msg,"Control Register Automatic Mode Error.\r\n");
			//HAL_UART_Transmit(&huart2,msg,strlen((char*)msg),HAL_MAX_DELAY);
			HAL_Delay(5);
		}

}
