#ifndef INC_BUCK_H_
#define INC_BUCK_H_

#include "stm32l4xx_hal.h"
#include <stdio.h>

#define F_PWM    150000UL // min : 1220 Hz max: 80 MHz
#define F_ACQ    5000UL // min : 0.018 Hz max: 80 MHz
#define KVOUTMON (6.2/(110.0+6.2))
#define KVINMON  (0.942*0.05838)//(6.2/(100.0+6.2))
#define KIMON    (0.006*51.0)
#define VCC      (3.3)

#define VOUT_MAX 48.0
#define VOUT_MIN 0.00
#define IOUT_MAX 10.0
#define IOUT_MIN 0.00
#define DUTY_MAX 0.90
#define DUTY_MIN 0.10

#define REG_MODE_CV   0
#define REG_MODE_CC   1
#define REG_MODE_MPPT 2
#define REG_MODE_OL   3

#define F_TIM1 80000000UL/F_PWM

#define KP_CC 0.1 			// Valeur du coefficient proportionnel régulation CC
#define KI_CC .001 			// Valeur du coefficient intégral régulation CC
#define KP_CV 0.05//(250.0/2370.0) 		// Valeur du coefficient proportionnel régulation CV
#define KI_CV 1 			// Valeur du coefficient intégral régulation CV

#define SAT_ERR_TOT 400 // Valeur pour la saturation de l'erreur totale

float Duty_Cycle;	//Timer duty value, from 0 to F_TIM1
int Vout_mon;		//Measured output voltage, from 0 to 4095, PC0
int Vin_mon;		//Measured input  voltage, from 0 to 4095, PC1
int I_mon;			//Measured output current, from 0 to 4095, PC2

//Onboard blue button : PC13
//Onboard LED LD2 : PA5
//PWM Output : PA8

float Pin;   //Measured input power for MPPT regulation
float Pin_p; //Previously measured input power for MPPT regulation
int I_inc;	//

int Reg_Mode;
int Vout_set;
int Iout_set;

uint32_t ADC_buffer[3];

float Delta_Err;  	//Valeur de l'erreur entre la tension/courant souhaitée et la valeur lue
float Err_Tot; 		//Pour la partie intégrale de la régulation
float Delta_Duty; 	//Variation du duty après la correction proportionnelle, intégrale

void Set_Duty_Cycle();
void SetVout(float target);
void SetI(float target);
void Set_Duty_OL(float target);

void RegulateCV(void);
void RegulateCC(void);
void RegulateMPPT(void);

float getVout(void);
float getIout(void);
float getVin(void);

void printBuckStatus(void);

#endif /* INC_BUCK_H_ */
