/*
 * gsm.h
 *
 *  Created on: 2017年9月13日
 *      Author: saber
 */

#ifndef APP_GSM_GSM_H_
#define APP_GSM_GSM_H_

#include "comm.h"

#define MAX_TX_BUFFER 1024
#define MAX_RX_BUFFER 2048

#define awake_gsm()       MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0)
#define sleep_gsm()       MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0)

#define open_net()          _msg_send(AT_NETOPEN)
#define close_net()         _msg_send(AT_NETCLOSE)


//GSM各管脚，RI接收中断初始化
extern void gsm_init();

extern void gsm_close();

//信息通过DMA传送至GSM发送
extern void msg_send(char *message);

extern void start_msg_send();

extern void stop_msg_send();

//定时器中断里与服务器进行连接
extern void gsm_connect();

//清除接收缓存器
extern void clear_buffer();

extern char * msg_read();

#endif /* APP_GSM_GSM_H_ */
