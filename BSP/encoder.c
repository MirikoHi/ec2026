#include "encoder.h"
#include "encoder_timer.h"
#include "stdlib.h"
#include "string.h"
#include "dwt.h"
#include "NRF24L01.h"
ENCODER_RES encoder_instance[MAX_ENCODER_NUM] = {0};
uint8_t idx_encoder = 0;
uint32_t gpioA_Pin_registered = 0;
uint32_t gpioB_Pin_registered = 0;

void encoder_interrupt_begin(void)
{
	bool GPIOA_falg=0;
	bool GPIOB_flag=0;
	for(uint8_t i=0;i<idx_encoder;i++)
	{
		if(encoder_instance[i].PortPin.A_PORT==GPIOA||encoder_instance[i].PortPin.B_PORT==GPIOA)
		{
			GPIOA_falg=1;
		}
		if(encoder_instance[i].PortPin.A_PORT==GPIOB||encoder_instance[i].PortPin.B_PORT==GPIOB)
		{
			GPIOB_flag=1;
		}
	}
	if(GPIOA_falg)
	{
		NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
		NVIC_EnableIRQ(GPIOA_INT_IRQn);
	}
	if(GPIOB_flag)
	{
		NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
	}

}

ENCODER_RES* encoder_init(encoder_PortPin_s* init)
{
	ENCODER_RES* encoder=&encoder_instance[idx_encoder++];
	
	encoder->PortPin=*init;
	
	if(init->A_PORT==GPIOA)
	{
		gpioA_Pin_registered |= init->A_pin; 
	}
	else if(init->A_PORT==GPIOB)
	{
		gpioB_Pin_registered |= init->A_pin; 
	}
	if(init->B_PORT==GPIOA)
	{
		gpioA_Pin_registered |= init->B_pin; 
	}
	else if(init->B_PORT==GPIOB)
	{
		gpioB_Pin_registered |= init->B_pin; 
	}
	

	
	return encoder;
}


void encoder_update(void)
{
	for(uint8_t i=0;i<idx_encoder;i++)
	{
		encoder_instance[i].count = encoder_instance[i].temp_count;
		encoder_instance[i].dir = (encoder_instance[i].count >= 0 ) ? FORWARD : REVERSAL;
		encoder_instance[i].total_count = encoder_instance[i].total_count + encoder_instance[i].count;
		encoder_instance[i].temp_count = 0;
		
	}
}


void GROUP1_IRQHandler(void)
{
//	NRF24L01_IRQHandler();
	uint32_t gpio_status[2];//0:GPIOA,1:GPIOB
	
	gpio_status[0] =  DL_GPIO_getEnabledInterruptStatus(GPIOA,gpioA_Pin_registered);
	gpio_status[1] = 	DL_GPIO_getEnabledInterruptStatus(GPIOB,gpioB_Pin_registered);
	
	for(uint8_t i=0;i<idx_encoder;i++)
	{
		if(gpio_status[encoder_instance[i].PortPin.A_PORT == GPIOA ? 0:1]&encoder_instance[i].PortPin.A_pin)
		{
			if(!DL_GPIO_readPins(encoder_instance[i].PortPin.B_PORT,encoder_instance[i].PortPin.B_pin))
			{
				encoder_instance[i].temp_count--;
			}
			else
			{
				encoder_instance[i].temp_count++;
			}
		}
		if(gpio_status[encoder_instance[i].PortPin.B_PORT == GPIOA ? 0:1]&encoder_instance[i].PortPin.B_pin)
		{
			if(!DL_GPIO_readPins(encoder_instance[i].PortPin.A_PORT,encoder_instance[i].PortPin.A_pin))
			{
				encoder_instance[i].temp_count++;
			}
			else
			{
				encoder_instance[i].temp_count--;
			}
		}
	}
	DL_GPIO_clearInterruptStatus(GPIOA,gpioA_Pin_registered);
	DL_GPIO_clearInterruptStatus(GPIOB,gpioB_Pin_registered);


}