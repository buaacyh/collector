/*
 * power.c
 *
 *  Created on: 2017Äê9ÔÂ16ÈÕ
 *      Author: saber
 */


#include "power.h"

typedef struct{
    uint8_t gpio_group;
    uint16_t gpio_pins;
}gpio_port;

const gpio_port dc_5v_ena_port ={.gpio_group = GPIO_PORT_P3, .gpio_pins=GPIO_PIN0};
const gpio_port dc_33v_ena_port = {.gpio_group = GPIO_PORT_P5, .gpio_pins=GPIO_PIN6};
const gpio_port tfcard_ena_port = {.gpio_group = GPIO_PORT_P10, .gpio_pins=GPIO_PIN4};

void power_manage_init(){
    MAP_GPIO_setAsOutputPin(dc_5v_ena_port.gpio_group, dc_5v_ena_port.gpio_pins);

    MAP_GPIO_setAsOutputPin(dc_33v_ena_port.gpio_group, dc_33v_ena_port.gpio_pins);

    MAP_GPIO_setAsOutputPin(tfcard_ena_port.gpio_group, tfcard_ena_port.gpio_pins);

}
void dc_5v_enable(void){
    MAP_GPIO_setOutputLowOnPin(dc_5v_ena_port.gpio_group, dc_5v_ena_port.gpio_pins);
}
void dc_5v_disable(void){
    MAP_GPIO_setOutputHighOnPin(dc_5v_ena_port.gpio_group, dc_5v_ena_port.gpio_pins);
}

void dc_33v_enable(void){
    MAP_GPIO_setOutputLowOnPin(dc_33v_ena_port.gpio_group, dc_33v_ena_port.gpio_pins);
}
void dc_33v_disable(void){
    MAP_GPIO_setOutputHighOnPin(dc_5v_ena_port.gpio_group, dc_5v_ena_port.gpio_pins);
}

void tfcard_enable(void){
    MAP_GPIO_setOutputLowOnPin(tfcard_ena_port.gpio_group, tfcard_ena_port.gpio_pins);
}
void tfcard_disable(void){
    MAP_GPIO_setOutputHighOnPin(tfcard_ena_port.gpio_group, tfcard_ena_port.gpio_pins);
}



