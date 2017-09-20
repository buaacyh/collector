/*
 * clocks.c
 *
 *  Created on: 2017Äê9ÔÂ14ÈÕ
 *      Author: saber
 */


#include "clocks.h"

//65535/ACLK CLOCK*divider=64s
const Timer_A_ContinuousModeConfig timer_config =
{
        TIMER_A_CLOCKSOURCE_ACLK,           // ACLK Clock = 1024
        TIMER_A_CLOCKSOURCE_DIVIDER_10,      //
        TIMER_A_TAIE_INTERRUPT_ENABLE,      // Enable Overflow ISR
        TIMER_A_DO_CLEAR                    // Clear Counter
};

void clock_init(){
    /* Setting DCO to 24MHz (upping Vcore) */
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_6);//DCO 6MHz

    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_32KHZ);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_32);//ACLK 1k

    MAP_PCM_setPowerStateNonBlocking(PCM_AM_LDO_VCORE1);
}

void timer_init(){

    MAP_Timer_A_configureContinuousMode(TIMER_A1_BASE, &timer_config);
    MAP_Interrupt_enableInterrupt(INT_TA1_N);
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);

}

/*
 * delay
 * time  ms for delay
 */


