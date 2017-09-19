/*
 * rainfall.c
 *
 *  Created on: 2017Äê9ÔÂ13ÈÕ
 *      Author: saber
 */
#include "rainfall.h"

static uint32_t rainfall_value = 0;

void rainfall_init(void){
    MAP_GPIO_setAsInputPin(GPIO_PORT_P10,GPIO_PIN3);     //MSP DI 0

    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN0);      //MSP DI 1
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN0);

    MAP_Interrupt_enableInterrupt(INT_PORT1);
}
uint32_t get_rainfall_value(void){
    return rainfall_value;
}
void clear_rainfall_value(void){
    rainfall_value = 0;
}

//IRQ MSP DI 1
void PORT1_IRQHandler(void)
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if(status & GPIO_PIN0)
    {
        rainfall_value++;
    }

}
