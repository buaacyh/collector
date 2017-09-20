/*
 * power.h
 *
 *  Created on: 2017Äê9ÔÂ16ÈÕ
 *      Author: saber
 */

#ifndef APP_POWER_POWER_H_
#define APP_POWER_POWER_H_

#include "comm.h"

extern void power_manage_init();
extern void dc_5v_enable(void);
extern void dc_5v_disable(void);
extern void dc_33v_enable(void);
extern void dc_33v_disable(void);
extern void tfcard_enable(void);
extern void tfcard_disable(void);



#endif /* APP_POWER_POWER_H_ */
