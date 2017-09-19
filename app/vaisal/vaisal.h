/*
 * vaisal.h
 *
 *  Created on: 2017Äê9ÔÂ13ÈÕ
 *      Author: saber
 */

#ifndef APP_VAISAL_VAISAL_H_
#define APP_VAISAL_VAISAL_H_


#include "comm.h"

#define MAX_REV_NUM 4
#define MAX_BUFFER 100
#define TH_DATAS_NUM 5

typedef struct vaisal_data_t{
    float value;
    uint8_t pv;
    char sign[5];
    float avg_value;
    uint8_t completed;
}vaisal_data_struct;

extern vaisal_data_struct vaisal_datas[TH_DATAS_NUM];
extern void vaisal_init(void);

extern void start_vaisal_rev();
extern void stop_vaisal_rev();

#endif /* APP_VAISAL_VAISAL_H_ */
