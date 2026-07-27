#ifndef _TRACE_H_
#define _TRACE_H_

#include "ti_msp_dl_config.h"
#include "PID.h"

float Trace_task(void);
void Trace_Init(void);
float raw_transform_easy(uint8_t current_trace);
float second_process(float raw_val);
uint8_t filter_raw(uint8_t raw);


typedef enum {
	TRACE_END = 0,
	TRACE_INLINE,
	TRACE_LOST,
}trace_state_e;


typedef struct {
		float pid_output;
}trace_fetch_data_q;




#endif
