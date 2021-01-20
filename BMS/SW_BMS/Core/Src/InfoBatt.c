/*
 * InfoBatt.c
 *
 *  Created on: 27 Nov 2020
 *      Author: jeremy aubert for GEI Corp
 */

#include "InfoBatt.h"
#include "i2c.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

static const float DeltaQ=0.00069901315; 	//	float qLSB=(0.34/1000)*(50/Rsense)*(M/4096) Ah;
static const float Rsense=6.08; 			//mOhm

#define BATTERY1 hi2c1
#define BATTERY2 hi2c2

/***********************************
 *	BATTERY 1 (BATT1) & 2 (BATT2)
 **********************************/

float InfoBatt_getCurrent(I2C_HandleTypeDef nb_batt)
{
	float voltage;
	uint16_t bufRx16=0x0000;

	//HAL_StatusTypeDef ret;							//Error Message management
	//ret=read_register(nb_batt,VOLT_REG,bufRx16);

	read_register(nb_batt,VOLT_REG,bufRx16);
	voltage=70.8*bufRx16/65535;
	return voltage;
}


float InfoBatt_getVoltage(I2C_HandleTypeDef nb_batt)
{
	float current;
	uint16_t bufRx16=0x0000;

	//HAL_StatusTypeDef ret;							//Error Message management
	//ret=read_register(nb_batt,AMP_REG,bufRx16);

	read_register(nb_batt,AMP_REG,bufRx16);
	current=64*(bufRx16-32767)/(32767*Rsense);
	return current;
}


float InfoBatt_getSoc(I2C_HandleTypeDef nb_batt)
{
	float SOC;
	uint16_t bufRx16=0x0000;

	//HAL_StatusTypeDef ret;							//Error Message management
	//ret=read_register(nb_batt,COUL_REG,bufRx16);

	read_register(nb_batt,COUL_REG,bufRx16);
	SOC=bufRx16*DeltaQ;
	return SOC;
}


HAL_StatusTypeDef setSoc(I2C_HandleTypeDef nb_batt, uint16_t initValue)
{
	HAL_StatusTypeDef ret;
	ret=Write_Register(nb_batt,CTRL_REG,SHUTDOWN);
	if (ret==HAL_OK)
	{
		HAL_Delay(5);
		Write_Register(nb_batt, COUL_REG,initValue>>8); 		//Ces deux lignes utilisent la valeur sur 2 octets, en hexadecimal, du SOC initial
		ret=Write_Register(nb_batt, COUL_REG+1,initValue<<8);   //et la coupe en deux afin d'envoyer un octet par un octet. Le dernier 'ret' fait l'hypothèse
		if (ret==HAL_OK)										//que si on a une erreur au niveau du write, alors elle sera forcément sur le deuxième write - d'où la gestion des erreurs au 2e write.
		{
			HAL_Delay(5);
			ret=Write_Register(nb_batt,CTRL_REG,AUTO_MODE);
			if (ret==HAL_OK)
			{
				HAL_Delay(5);
			}
		}
	}
	return ret;
}


//Fonctions supports

uint16_t saveSOC(I2C_HandleTypeDef nb_batt)
{
	uint16_t val=0x0000;

	//HAL_StatusTypeDef ret;									//Error Message management
	//ret=read_register(nb_batt,COUL_REG,val);

	read_register(nb_batt,COUL_REG,val);
	return val;													//On utilise une nouvelle fonction et pas InfoBatt_getSOC car il nous faut la valeur en hexa.
}


HAL_StatusTypeDef checkLink(I2C_HandleTypeDef nb_batt)
{
	return HAL_I2C_IsDeviceReady(&nb_batt,GAUGE_ADDR,1,500); 	//A utiliser selon le bon vouloir de l'utilisateur
}
