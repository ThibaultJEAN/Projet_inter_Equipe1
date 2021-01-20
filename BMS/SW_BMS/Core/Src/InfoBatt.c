/*
 * InfoBatt.c
 *
 *  Created on: 27 Nov 2020
 *      Author: jerem
 */

#include "InfoBatt.h" //Voir les fichiers HAL pour faire le lien entre "nb_batt" et le type de port i2c que l'on utilise
#include "i2c.h"
#include <string.h>
#include <stdio.h>

static const float DeltaQ=0.00069901315; //	float qLSB=(0.34/1000)*(50/Rsense)*(M/4096) Ah;
static const float Rsense=6.08; //mOhm


/***********************************
 *	BATTERY 1 (BATT1) & 2 (BATT2)
 **********************************/

float InfoBatt_getCurrent(uint8_t nb_batt) //done
{
	float voltage;
	uint16_t bufRx16=read_register(nb_batt,VOLT_REG);
	voltage=70.8*bufRx16/65535;
	return voltage;
}


float InfoBatt_getVoltage(uint8_t nb_batt) //done
{
	float current;
	uint16_t bufRx16=read_register(nb_batt, AMP_REG);
	current=64*(bufRx16-32767)/(32767*Rsense);
	return current;
}


float InfoBatt_getSoc(uint8_t nb_batt) //done
{
	float SOC;
	uint16_t bufRx16=read_register(nb_batt, COUL_REG);
	SOC=bufRx16*DeltaQ;
	return SOC;
}


HAL_StatusTypeDef setSOC(uint8_t nb_batt, uint16_t initValue) //A verifier
{
	HAL_StatusTypeDef ret;
	ret=Write_Register(nb_batt,CTRL_REG,SHUTDOWN);
	if (ret==HAL_OK)
	{
		HAL_Delay(5);
		Write_Register(nb_batt, COUL_REG_MSB,initValue>>8); //A verifier
		ret=Write_Register(nb_batt, COUL_REG_LSB,initValue<<8); //faire des tests pour voir si les <<8 et >>8 sont OK
		if (ret==HAL_OK)
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


//Fonctions support

uint16_t saveSOC(uint8_t nb_batt)
{
	return read_register(nb_batt, COUL_REG);
}


HAL_StatusTypeDef checkLink(uint8_t nb_batt)
{
	return HAL_I2C_IsDeviceReady(&nb_batt,GAUGE_ADDR,1,500);
}
