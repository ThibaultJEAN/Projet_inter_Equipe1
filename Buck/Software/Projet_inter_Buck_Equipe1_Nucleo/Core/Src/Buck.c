#include "Buck.h"

extern TIM_HandleTypeDef htim1;

void Set_Duty_Cycle(){

	if (Duty_Cycle>F_TIM1*DUTY_MAX){
		Duty_Cycle= F_TIM1*DUTY_MAX;
	} else if (Duty_Cycle<F_TIM1*DUTY_MIN)	{
		Duty_Cycle=F_TIM1*DUTY_MIN;
	}
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Duty_Cycle);
}

void SetVout(float target){
	if(target>VOUT_MAX)target=VOUT_MAX;
	if(target<VOUT_MIN)target=VOUT_MIN;
	Vout_set = (target)*(KVOUTMON*4096.0)/VCC;
}

void SetI(float target){
	if(target>IOUT_MAX)target=IOUT_MAX;
	if(target<IOUT_MIN)target=IOUT_MIN;
	Iout_set = (target)*(KIMON*4096.0)/VCC;
}

void Set_Duty_OL(float target){
	if(target>DUTY_MAX)target=DUTY_MAX;
	if(target<DUTY_MIN)target=DUTY_MIN;
	Duty_Cycle=(target)*F_TIM1;
	Set_Duty_Cycle();
}

void RegulateCV(void){
	Delta_Err = Vout_set - Vout_mon;
	/*Err_Tot += Delta_Err;

	if (Err_Tot > SAT_ERR_TOT){
		Err_Tot = SAT_ERR_TOT;
	}else if (Err_Tot < -(SAT_ERR_TOT)){
		Err_Tot = -SAT_ERR_TOT;
	}
*/
	Delta_Duty = (Delta_Err *KP_CV) ;//+ (Err_Tot *KI_CV );
	Duty_Cycle += Delta_Duty;

	Set_Duty_Cycle();
}

void RegulateCC(void){
	Delta_Err = Iout_set - I_mon;
	/*Err_Tot += Delta_Err;

	if (Err_Tot > SAT_ERR_TOT)
	{
		Err_Tot = SAT_ERR_TOT;
	}
	if (Err_Tot < -(SAT_ERR_TOT))
	{
		Err_Tot = -SAT_ERR_TOT;
	}*/

	Delta_Duty = (Delta_Err *KP_CC);//+ (Err_Tot * KI_CC);
	Duty_Cycle += Delta_Duty;

	Set_Duty_Cycle();
}

void RegulateMPPT(void){
	if(cnt++>25){
		Pin = I_mon*Vin_mon;
		if (Pin < (Pin_p - 551)){
			I_inc=-1;
		}else if(Pin > (Pin_p + 551)){
			I_inc=1;
		}
		Pin_p = Pin;
		Iout_set+=I_inc;
		cnt=0;
	}
	RegulateCC();
}


float getVout(void){
	return (float)Vout_mon*(VCC/KVOUTMON/4096.0);
}
float getIout(void){
	return (float)I_mon*(VCC/KIMON/4096.0);
}
float getVin(void){
	return (float)Vin_mon*(VCC/KVINMON/4096.0);
}

void printBuckStatus(void){
	printf("Vin %.1fV, Vout %.1fV, Iout %.2fA, Duty : %.1f%%, Pout %.1fW Regmode ",getVin(), getVout(), getIout(), Duty_Cycle*100/533,getIout()*getVout());
	switch(Reg_Mode){
	case REG_MODE_CV :
		printf("CV");
		break;
	case REG_MODE_OL:
		printf("OL");
		break;
	case REG_MODE_CC :
		printf("CC");
		break;
	case REG_MODE_MPPT:
		printf("MPPT");
		break;
	default :
		break;
	}
	printf("I_inc %d, Iset %d\r\n",I_inc,Iout_set);
//	printf("\tI_mon : %d\t Vin_mon : %d Err : %d\r\n",I_mon,Vin_mon,Delta_Err);
}
