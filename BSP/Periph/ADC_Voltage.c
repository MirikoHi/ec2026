// #include "ADC_Voltage.h"
// float voltage_value = 0;
// volatile bool gCheckADC;
// void ADC_VOLTAGE_Init(void)
// {
// 	NVIC_EnableIRQ(ADC_VOLTAGE_INST_INT_IRQN);
// 	DL_ADC12_startConversion(ADC_VOLTAGE_INST);
// }
//
//
// void ADC_VOLTAGE_INST_IRQHandler(void)
// {
//
// 	switch (DL_ADC12_getPendingInterrupt(ADC_VOLTAGE_INST))
// 	{
//
// 		case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
// //				voltage_value = DL_ADC12_getMemResult(ADC_VOLTAGE_INST, ADC_VOLTAGE_ADCMEM_ADC_CH4)/4096.0f*1.40f*11.0f; ;
// //				DL_ADC12_clearInterruptStatus(ADC_VOLTAGE_INST, DL_ADC12_IIDX_MEM0_RESULT_LOADED);
// 			break;
// 		default:
// 			break;
// 	}
// }