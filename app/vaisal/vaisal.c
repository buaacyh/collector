/*
 * vaisal.c
 *
 *  Created on: 2017年9月13日
 *      Author: saber
 */


#include "vaisal.h"
#include "uart/uart.h"
#include "fatfs/ff.h"



typedef enum {
    HEADER_1 = 0,
    HEADER_2,
    RECIEVE_DATA,
    RECIEVE_END,
}vaisal_status;

const eUSCI_UART_Config vaisal_uart_config=
{
     EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // 选择SMCLK时钟为源，6MHz
     19,                                      // bps=19200;
     8,                                       //
     85,                                      //
     EUSCI_A_UART_NO_PARITY,                  // No Parity
     EUSCI_A_UART_LSB_FIRST,                  // LSB First
     EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
     EUSCI_A_UART_MODE,                       // UART mode
     EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
};

vaisal_data_struct vaisal_datas[TH_DATAS_NUM] = {{0,0,"Dm",0,0},{0,0,"Sm",0,0},{0,0,"Ta",0,0},{0,0,"Ua",0,0},{0,0,"Pa",0,0}};



void reset_vaisal_datas(){
    uint32_t i = 0;
    for(i=0;i<TH_DATAS_NUM;++i){
        vaisal_datas[i].value = 0;
        vaisal_datas[i].pv = 0;
        vaisal_datas[i].completed = 0;
    }
}

void vaisal_init(void){
     uart_config _vaisal_uart_conf = {.gpio_group = GPIO_PORT_P1,
     .gpio_pins= GPIO_PIN2 | GPIO_PIN3,
     .port = EUSCI_A0_BASE,
     .pconf = &vaisal_uart_config};

     uart_init(&_vaisal_uart_conf);

     /*
      uart_config _vaisal_uart_config =
      {
             GPIO_PORT_P1,
             GPIO_PIN2 | GPIO_PIN3,
             EUSCI_A0_BASE,
             &vaisal_uart_config
      };
      */
}


void start_vaisal_rev(){
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
}

void stop_vaisal_rev(){
    MAP_UART_disableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_disableInterrupt(INT_EUSCIA0);
}


vaisal_status vaisal_stat = HEADER_1;
uint8_t va_res_buffer[MAX_BUFFER];
uint8_t va_pbuffer=0;


uint8_t vaisal_process(){
    char *token,*strstrn;
    uint32_t i = 0;
    token=strtok(va_res_buffer,",");
    while(token !=NULL){

        token=strtok(NULL,",");

        for(i = 0;i<TH_DATAS_NUM;++i)
        {
            strstrn=strstr(token,vaisal_datas[i].sign);
            if(*strstrn){
                vaisal_datas[i].value+=atof(token+3);
                if(++vaisal_datas[i].pv == MAX_REV_NUM){
                    vaisal_datas[i].avg_value =vaisal_datas[i].value/vaisal_datas[i].pv;
                    vaisal_datas[i].value = 0;
                    vaisal_datas[i].pv = 0;
                    vaisal_datas[i].completed = 1;
                }
            }
        }
    }
    uint8_t all_completed = 1;
    for(i = 0;i<TH_DATAS_NUM;++i){
        all_completed&=vaisal_datas[i].completed;
    }
    return all_completed;
}
void EUSCIA0_IRQHandler(void){
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    uint8_t res;
    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);
    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG){
        res = MAP_UART_receiveData(EUSCI_A0_BASE);
        switch(vaisal_stat){
        case HEADER_1:

            if(res == '0'){
                vaisal_stat = HEADER_2;
                va_res_buffer[va_pbuffer++] = res;
            }
            break;
        case HEADER_2:
            if(res == 'R'){
                vaisal_stat = RECIEVE_DATA;
                va_res_buffer[va_pbuffer++] = res;
            }
            else{
                vaisal_stat = HEADER_1;
                va_pbuffer = 0;
            }
            break;
        case RECIEVE_DATA:
            if(res == 0x0d){
                va_res_buffer[va_pbuffer++] = res;
                vaisal_stat = RECIEVE_END;
            }else{
                va_res_buffer[va_pbuffer++] = res;
            }
            break;
        case RECIEVE_END:
            if(res == 0x0a){
                va_res_buffer[va_pbuffer++] = res;
                va_res_buffer[va_pbuffer++] = '\0';
                //deal with the data format
                vaisal_stat = HEADER_1;

                if(vaisal_process(va_res_buffer)){
                    //all completed and write the res to tfcard

                }
                memset(va_res_buffer,'\0',sizeof(va_res_buffer));
                va_pbuffer=0;
            }
            else{
                vaisal_stat = HEADER_1;
                va_pbuffer = 0;
            }
            break;
        default:
            break;
        }
    }
}

