#include "KEY.h"

#define KEY_PRESSED    1
#define KEY_UNPRESSED  0

#define KEY_TIME_DOUBLE    200
#define KEY_TIME_LONG				500	
#define KEY_TIME_REPEAT    100

#define KEY_COUNT    4


uint8_t Key_Flag[KEY_COUNT];



/**
 * @brief 读取指定按键当前电平状态，低电平表示按下
 */
uint8_t Key_GetState(uint8_t n)
{
	if(n == 0)
	{
		if(DL_GPIO_readPins(KEY_key1_PORT,KEY_key1_PIN) == 0)
		{
			return KEY_PRESSED;
		}
	}
	if(n == 1)
	{
		if(DL_GPIO_readPins(KEY_key2_PORT,KEY_key2_PIN)  == 0)
		{
			return KEY_PRESSED;
		}
	}
	if(n == 2)
	{
		if(DL_GPIO_readPins(KEY_key3_PORT,KEY_key3_PIN)  == 0)
		{
			return KEY_PRESSED;
		}
	}
	if(n == 3)
	{
		if(DL_GPIO_readPins(KEY_key4_PORT,KEY_key4_PIN)  == 0)
		{
			return KEY_PRESSED;
		}
	}
	return KEY_UNPRESSED;	
}

/**
 * @brief 查询按键事件标志，非保持类事件读取后会自动清除
 */
uint8_t Key_Check(uint8_t n,uint8_t Flag)
{
		if(Key_Flag[n] & Flag)
		{
				if(Flag != KEY_HOLD)
				{
						Key_Flag[n] &= ~Flag;
				}
				return 1;
		}
		return 0;
}



/**
 * @brief 按键扫描任务,周期性更新按键状态、消抖及单双击/长按/连发事件
 */
void Key_Tick(void)
{
		static uint8_t Count,i;
		static uint8_t CurrState[KEY_COUNT],PrevState[KEY_COUNT];
		static uint8_t S[KEY_COUNT];
		static uint16_t Time[KEY_COUNT];
	
		for(i=0;i<KEY_COUNT;i++)
		{
				if(Time[i] > 0)
				{
						Time[i] --;
				}
		}
		Count ++;
	if(Count >= 20)
	{
		Count = 0;
		for(i=0;i<KEY_COUNT;i++)
		{
				PrevState[i] = CurrState[i];
				CurrState[i] = Key_GetState(i);
		
				if(CurrState[i] == KEY_PRESSED)
				{
						Key_Flag[i] |= KEY_HOLD;	
				}
				else
				{
						Key_Flag[i] &= ~KEY_HOLD;
				}
				if(CurrState[i] == KEY_PRESSED && PrevState[i] == KEY_UNPRESSED)
				{
						Key_Flag[i] |= KEY_DOWN;
				}
				if(CurrState[i] == KEY_UNPRESSED && PrevState[i] == KEY_PRESSED)
				{
						Key_Flag[i] |= KEY_UP;
				}
				if(S[i] == 0)
				{
						if(CurrState[i] == KEY_PRESSED)
						{
								Time[i] = KEY_TIME_LONG;
								S[i] = 1;
						}
				}
				else if(S[i] == 1)
				{
						if(CurrState[i] == KEY_UNPRESSED)
						{
								Time[i] = KEY_TIME_DOUBLE;
								S[i] = 2;
						}
						else if(Time[i] == 0)
						{
								Time[i] = KEY_TIME_REPEAT;
								Key_Flag[i] |= KEY_LONG;
								S[i] = 4;
						}
				}
				else if(S[i] == 2)
				{
						if(CurrState[i] == KEY_PRESSED)
						{
								Key_Flag[i] |= KEY_DOUBLE;
								S[i] = 3;
						}
						else if(Time[i] == 0)
						{
								Key_Flag[i] |= KEY_SINGLE;
								S[i] = 0;
						}
				}
				else if(S[i] == 3)
				{
						if(CurrState[i] == KEY_UNPRESSED)
						{
								S[i] = 0;
						}
				}
				else if(S[i] == 4)
				{
						if(CurrState[i] == KEY_UNPRESSED)
						{
								S[i] = 0;
						}
						else if(Time[i] == 0)
						{
								Time[i] = KEY_TIME_REPEAT;
								Key_Flag[i] |= KEY_REPEAT;
								S[i] = 4;
						}						
				}
		}
	}
}
/**
 *  @brief 清除所有按键事件标志
 * 
 */
void Key_ClearAllFlags(void)
{
    for(uint8_t i = 0; i < KEY_COUNT; i++) 
    {
        Key_Flag[i] = 0;  
    }
}




