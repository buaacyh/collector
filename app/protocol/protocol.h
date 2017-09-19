/*
 * protocol.h
 *
 *  Created on: 2017Äê9ÔÂ14ÈÕ
 *      Author: saber
 */

#ifndef APP_PROTOCOL_PROTOCOL_H_
#define APP_PROTOCOL_PROTOCOL_H_

#include "comm.h"

extern bool upload_data_start();
extern bool upload_data(char * key, float value);
extern bool upload_data_end();

#endif /* APP_PROTOCOL_PROTOCOL_H_ */
