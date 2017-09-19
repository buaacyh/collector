/*
 * rtc.c
 *
 *  Created on: 2017Äê9ÔÂ13ÈÕ
 *      Author: saber
 */

#include "rtc.h"

const RTC_C_Calendar currentTime =
 {
         0x00,
         0x10,
         0x14,
         0x04,
         0x20,
         0x07,
         0x2017
 };

void RTC_init()
 {
     MAP_RTC_C_initCalendar(&currentTime, RTC_C_FORMAT_BCD);
     MAP_RTC_C_startClock();
 }
