/*
 * lpc_relay.c
 *
 *  Created on: 2017Äê9ÔÂ12ÈÕ
 *      Author: saber
 */

#include "relay.h"


uint8_t get_relay_L_status(uint8_t relay_no)
{
    uint8_t uRet = 0;
    if(relay_no == RELAY_K_GSM)
    {
        uRet = GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3);
    }
    else if(relay_no == RELAY_K_CAMERA)
    {
        uRet = GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN4);
    }
    else if(relay_no == RELAY_K_IO_POWER)
    {

    }
    else if(relay_no == RELAY_K_IO)
    {
        uRet = GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN7);
    }
    else
    {
        uRet = 0xFF;
    }
    return uRet;
}

uint8_t get_relay_H_status(uint8_t relay_no)
{
    uint8_t uRet = 0;
    if(relay_no == RELAY_K_GSM)
    {
        uRet = GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2);
    }
    else if(relay_no == RELAY_K_CAMERA)
    {
        uRet = GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN5);
    }
    else if(relay_no == RELAY_K_IO_POWER)
    {
        uRet = GPIO_getInputPinValue(GPIO_PORT_P9, GPIO_PIN3);
    }
    else if(relay_no == RELAY_K_IO)
    {
        uRet = GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN6);
    }
    else
    {
        uRet = 0xFF;
    }
    return uRet;
}

void set_relay(uint8_t relay_no)
{
    volatile uint32_t i;
    if(relay_no&RELAY_K_GSM)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);
        for(i=100000; i>0; i--){
            _nop();
        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
    }
    if(relay_no&RELAY_K_CAMERA)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN0);
        for(i=100000; i>0; i--)
            ;
        GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0);
    }
    if(relay_no&RELAY_K_IO_POWER)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN2);
        for(i=100000; i>0; i--);
        GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);
    }
    if(relay_no&RELAY_K_IO)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN0);
        for(i=100000; i>0; i--);
        GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN0);
    }
}

void reset_relay(uint8_t relay_no)
{
    volatile uint32_t i;
    if(relay_no&RELAY_K_GSM)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
        for(i=100000; i>0; i--);
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
    }
    if(relay_no&RELAY_K_CAMERA)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN1);
        for(i=100000; i>0; i--);
        GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN1);
    }
    if(relay_no&RELAY_K_IO_POWER)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN4);
        for(i=100000; i>0; i--);
        GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN4);
    }
    if(relay_no&RELAY_K_IO)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN1);
        for(i=100000; i>0; i--);
        GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN1);
    }
}

void config_relay(uint8_t relay_no)
{

    if(relay_no&RELAY_K_GSM)
    {
        GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0); //IO_GSM_RELAY_SET
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
        GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1); //IO_GSM_RELAY_RESET
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
        GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN2); //IO_GSM_5V_STATUS
        GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN3); //IO_GSM_3.3V_STATUS
    }
    if(relay_no&RELAY_K_CAMERA)
    {
        GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0); //IO_CAMERA_RELAY_SET
        GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0);
        GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN1); //IO_CAMERA_RELAY_RESET
        GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN1);
        GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN4); //IO_CAMERA_3.3V_STATUS
        GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN5); //IO_CAMERA_12V_STATUS
    }
    if(relay_no&RELAY_K_IO_POWER)
    {
        GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN2); //IO_K3_RELAY_SET
        GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);
        GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN4); //IO_K3_RELAY_RESET
        GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN4);

        GPIO_setAsInputPin(GPIO_PORT_P9, GPIO_PIN3); //IO_K3_12V_STATUS
    }
    if(relay_no&RELAY_K_IO)
    {
        GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN0); //IO_L_RELAY_SET
        GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN0);
        GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN1); //IO_L_RELAY_RESET
        GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN1);
        GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN6); //IO_5V_STATUS
        GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN7); //IO_3.3V_STATUS
    }

}
