#ifndef _TRACE_H_
#define _TRACE_H_

#include "ti_msp_dl_config.h"
#include "PID.h"

float Trace_task(void);
void Trace_Init(void);



typedef struct {
		float pid_output;
}trace_fetch_data_q;




#endif
