/*
 * protocol.c
 *
 *  Created on: 2017Äê9ÔÂ14ÈÕ
 *      Author: saber
 */

#include "protocol.h"

#include "app/gsm/gsm.h"
#include "base/clocks/clocks.h"

#define RETRY 2

char *PUSH_DATA_HEAD="device_id:123,device_config_id:1,method:push_data,count:5;";
char *PACKAGE_FORMAT="%s:%.2f,ts:%d;";
char *PUSH_DATA_FINISH="method:push_data_finish;";
char *PUSH_DATA_READY="push_data_ready";
char *DATA_UPLOADED="data_uploaded";
char *FAIL="ERROR";
char *TIME_SECOND="ts:";

uint32_t now_time = 0;

uint8_t protocol_buffer[MAX_TX_BUFFER];

bool upload_data_start(){
    char* p = NULL;
    char* pres = NULL;
    uint8_t i = 0;
    for(;i<RETRY;++i){
        clear_buffer();

        msg_send(PUSH_DATA_HEAD);
        delay_ms(1000);

        if(wait_msg_reved()){
            p = msg_read();
            pres = strstr(p, PUSH_DATA_READY);
            if(*pres){
                pres = strstr(p, TIME_SECOND);
                if(*pres){
                   now_time = atoi(pres+strlen(TIME_SECOND));
                }
                return true;
            }
        }
    }
    return false;


}

bool upload_data(char * key, float value){
    char* p = NULL;
    char* pres = NULL;


    memset(protocol_buffer,0,MAX_TX_BUFFER);
    sprintf((char *)protocol_buffer, PACKAGE_FORMAT, key, value, now_time);

    msg_send(protocol_buffer);
    /*
    uint8_t i = 0;
    for(;i<RETRY;++i){
        clear_buffer();
        msg_send(protocol_buffer);
        if(wait_msg_reved())
        {
            p = msg_read();
            pres = strstr(p, FAIL);
            if(*pres){
            }
            else{
                return true;
            }
        }
    }


    return false;
    */
    return true;
}

bool upload_data_end(){
    char* p = NULL;
    char* pres = NULL;
    uint8_t i = 0;
    for(;i<RETRY;++i){
        clear_buffer();
        msg_send(PUSH_DATA_FINISH);
        delay_ms(1000);
        if(wait_msg_reved()){
            p = msg_read();
            pres = strstr(p, DATA_UPLOADED);
            if(*pres){
                return true;
            }
        }
    }
    return false;


}
