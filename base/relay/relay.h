/*
 * lpc_relay.h
 *
 *  Created on: 2017Äê9ÔÂ12ÈÕ
 *      Author: saber
 */

#ifndef LPCRELAY_LPC_RELAY_H_
#define LPCRELAY_LPC_RELAY_H_

#include "comm.h"
#include <_lock.h>

#define RELAY_K_GSM 1
#define RELAY_K_CAMERA 2
#define RELAY_K_IO_POWER 4
#define RELAY_K_IO 8

extern uint8_t get_relay_L_status(uint8_t relay_no);

extern uint8_t get_relay_H_status(uint8_t relay_no);

extern void set_relay(uint8_t relay_no);

extern void reset_relay(uint8_t relay_no);

extern void config_relay(uint8_t relay_no);

#endif /* LPCRELAY_LPC_RELAY_H_ */
