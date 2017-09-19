/*
 * clocks.h
 *
 *  Created on: 2017Äê9ÔÂ14ÈÕ
 *      Author: saber
 */

#ifndef BASE_CLOCKS_CLOCKS_H_
#define BASE_CLOCKS_CLOCKS_H_

#include "comm.h"
#include <_lock.h>


#define CPU_CLOCK 6000000
#define delay_ms(ms) __delay_cycles(CPU_CLOCK/1000*ms)


extern void clock_init(void);
extern void timer_init(void);


#endif /* BASE_CLOCKS_CLOCKS_H_ */
