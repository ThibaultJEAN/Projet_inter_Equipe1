#include <stdio.h>
#define FCY 16000000UL
#include <libpic30.h>
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/system.h"


//ADC scaling coefficients
#define KVOUTMON (6.2/(110.0+6.2))  //Vout_mon divider
#define KVINMON (6.8/(100.0+6.8))  //Vin_mon divider
#define KIMON (.005*50.0)          //I_mon Rshunt * CSAmp gain
#define VCC (3.3)

#define REGMODE_CV      0   //Constant voltage mode, call setVout())
#define REGMODE_CC      1   //Constant current mode, call setI())
#define REGMODE_MPPT    2   //MPPT mode
#define REGMODE_OL      3   //Open Loop mode, call setDuty())

uint8_t regMode = REGMODE_OL;           //Regulation mode
uint16_t vout_mon, vin_mon, i_mon, pot; //ADC values (0-4095)
uint16_t duty_cycle=100;
bool state=0;                           //temporary
uint16_t vout_set, i_set;               //Target values (0-4095))

void setVout(float target){
    vout_set=(target)*(KVOUTMON)*4096.0/(VCC);
}
void setI(float target){
    i_set=(target)*(KIMON)*4096.0/(VCC);
}
void setDuty(float target){
    duty_cycle=(target)*TMR1_Period16BitGet();
}
void setRegMode(uint8_t mode, float target){
    regMode=mode;
    switch(regMode){
         case REGMODE_CV :
            setVout(target);
            break;
        case REGMODE_CC :
            setI(target);
            break;
        case REGMODE_MPPT :
            //TODO 
            break;
        case REGMODE_OL :
            setDuty(target);
            break;
    }
}

float getVin(void){
    return vin_mon/4096.0*VCC/KVINMON;
}
float getI(void){
    return i_mon/4096.0*VCC/KIMON;
}
float getVout(void){
    return vout_mon/4096.0*VCC/KVOUTMON;
}
float getDuty(void){
    return (float)duty_cycle/TMR1_Period16BitGet();
}

void TIM2_ISR(void){
    
    
	vout_mon = ADC1_GetConversion(Vout_mon);
	i_mon = ADC1_GetConversion(I_mon);
	pot = ADC1_GetConversion(Pot);
	
	
    switch(regMode){
        case REGMODE_CV :
            if(vout_mon<vout_set){
                duty_cycle++;
            }else if(vout_mon>vout_set){
                duty_cycle--;
            }
            break;
        case REGMODE_CC :
            if(i_mon<i_set){
                duty_cycle++;
            }else if(i_mon>i_set){
                duty_cycle--;
            }
            break;
        case REGMODE_MPPT :
            //TODO 
            break;
        case REGMODE_OL :   //Direct pot reading
            duty_cycle=1+(((TMR1_Period16BitGet()-1)*(uint32_t)pot)/4096);
            break;
    }
    
    //saturate duty cycle
    if(duty_cycle>(TMR1_Period16BitGet()-1))duty_cycle=TMR1_Period16BitGet()-1;
    else if(duty_cycle<1)duty_cycle=1;
    
    //assign duty cycle to output
    OC1_SecondaryValueSet(duty_cycle);
}
int main(void){
    SYSTEM_Initialize();
    TMR2_SetInterruptHandler(TIM2_ISR);
    
    //TMR1 : 125n - 4.096m , 8MHz - 244Hz : 20u = 50kHz = 0x13F //13u = 75kHz
    //TMR2 : 125n - 4.096m , 8MHz - 244Hz : 500u = 2kHz = 0x1F3F
    TMR2_Start();   //ADC timer, 2kHz
    TMR1_Start();   //PWM timer, 50kHz
    OC1_Start();    //Output compare : PWM
    
    printf("Hello world !\r\n");
    setI(10.0);
    setVout(5.0);
    
    LED0_Toggle();
    
    while (1){
        __delay_ms(10);
        printf("Vin : %.2f\tVout : %.2f\tPot : %d\tduty : %.2f\r\n",(double)getVin(),(double)getVout(),pot,(double)getDuty());
        __delay_ms(1000);
        
        if(BUTTON_GetValue() != state){	//Toggle button between CV & OL
            state=!state;
            if(state){
                LED0_Toggle();
                if(regMode == REGMODE_CV){
                    regMode = REGMODE_OL;
                    printf("Switched to OL mode\r\n");
                }else{
                    regMode = REGMODE_CV;
                    printf("Switched to CV mode\r\n");
                }
            }
            __delay_ms(1000);	//Crude button debounce
        }
    }
    return 1;
}