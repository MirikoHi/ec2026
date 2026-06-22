#include "menu.h"
#include "KEY.h"
#include "OLED.h"
#include "bsp_log.h"
#include "string.h"
#include <locale.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Chassis.h"
#include "Robot_Cmd.h"
#include "Gimbal.h"
MenuInstance* now_menu;
uint8_t row_idx=0;//屏幕上选中行
uint8_t row_max_idx=0;//当前屏幕最高行数（0-3）
uint8_t upper_limit_row_idx;//上界在string中的索引
uint8_t lower_limit_row_idx;
static MenuInstance* single_menu_init(MenuInitConfig_s *config,MenuInstance *pre_menu);
MenuInstance ALL_Menu_Instance[MAX_ALL_MENU_NUM]={0};
uint8_t idx_ALL_Menu=0;
int count_chars(const char *str);

void MenuInit(void)
{
MenuInitConfig_s third_menu_config[1] = {
			[0] = {
			.string={
				[0] = "普通",
				[1] = "巡线",
				[2] = "陀螺仪",
				[3] = "位置",
				[4]	= NULL,
			},
			.callback={
				[0] = Chassis_Mode_Switch_Callback,
				[1] = Chassis_Mode_Switch_Callback,
				[2] = Chassis_Mode_Switch_Callback,
				[3] = Chassis_Mode_Switch_Callback,
			},
			.next_menu_config={
			
			},
			.pre_idx=1,
		},
};
MenuInitConfig_s second_menu_config[3]	={
		[0] = {
			.string={
				[0] = "使能",
				[1] = "失能",
				[2] = NULL,
				[3] = NULL,
				[4]	= NULL,
			},
			.callback={
				[0] = Motor_Cmd_CallBack,
				[1] = Motor_Cmd_CallBack,
			},
			.next_menu_config={
			
			},
			.pre_idx=0,
		},
		[1] = {
			.string={
				[0] = "程序控制",
				[1] = "按键控制",
				[2] = NULL,
				[3] = NULL,
				[4]	= NULL,
			},
			.callback={
				[0] = Control_Switch_Callback,
				[1] = Control_Switch_Callback,
			},
			.next_menu_config={
				[1] = &third_menu_config[0],
			},
			.pre_idx=1,
		},
		[2] = {
			.string={
				[0] = "任务一",
				[1] = "任务二",
				[2] = NULL,
				[3] = NULL,
				[4]	= NULL,
			},
			.callback={
				[0] = Task_Callback,
				[1] = Task_Callback,
			},
			.next_menu_config={
			
			},
			.pre_idx=2,
		},
//		[3] = {
//			.string={
//				[0] = "李",
//				[1] = "志",
//				[2] = "超",
//				[3] = "机",
//				[4]	= "械",
//			},
//			.callback={
//				[0] = ChaoCallback,
//			},
//			.next_menu_config={
//			
//			},
//			.pre_idx=3,
//		},
//		[4] = {
//			.string={
//				[0] = "李",
//				[1] = "孜",
//				[2] = "宁",
//				[3] = "通",
//				[4]	= "信",
//			},
//			.callback={
//				[0] = NingCallback,
//			},
//			.next_menu_config={
//			
//			},
//			.pre_idx=4,
//		},
	};
MenuInitConfig_s first_menu_config={
		.string={
				[0] = "电机控制",
				[1] = "控制方式",
				[2] = "任务",
				[3] = NULL,
				[4]	= NULL,
				[5] = NULL,
			},
			.callback={
			},
			.next_menu_config={
				[0] = &second_menu_config[0],
				[1] = &second_menu_config[1],
				[2] = &second_menu_config[2],
				[3] = NULL,
				[4] = NULL,
			},
			.pre_idx=0,
	};
	now_menu = single_menu_init(&first_menu_config,NULL);
	
	
	
	
	
	
//	//下面为menu具体参数的初始化不需改动
	OLED_Init();
	row_idx=0;
	upper_limit_row_idx=0;
	row_max_idx=0;
	for(uint8_t i=0;i<MAX_MENU_NUM;i++)
	{
		if((now_menu->string[i])==NULL)
		{
	
			break;
		}
		row_max_idx++;
	}
	if(row_max_idx>=4)row_max_idx=3;
	else row_max_idx = row_max_idx-1;
	lower_limit_row_idx = row_max_idx;
	OLED_Clear();
	for(uint8_t i=0;i<=row_max_idx;i++)
	{
		OLED_ShowString(0,i*16,now_menu->string[i+upper_limit_row_idx],OLED_8X16);
	}
	OLED_AnimUpdate();
	OLED_Animation(0,0,0,0,
			0,0,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16);
	OLED_Update();
}
int count_chars(const char *str) {
    if (str == NULL) return 0;
    
    int char_count = 0;
    const uint8_t *ptr = (const uint8_t *)str;
    
    while (*ptr) {
        // 手动解析 UTF-8 字符
        if (*ptr < 0x80) {          // 1 字节字符 (0xxxxxxx)
            ptr += 1;
        } else if ((*ptr & 0xE0) == 0xC0) {  // 2 字节字符 (110xxxxx)
            ptr += 2;
        } else if ((*ptr & 0xF0) == 0xE0) {  // 3 字节字符 (1110xxxx)
            ptr += 3;
        } else if ((*ptr & 0xF8) == 0xF0) {  // 4 字节字符 (11110xxx)
            ptr += 4;
        } else {  // 无效 UTF-8 序列
            ptr += 1;  // 跳过无效字节
        }
        char_count++;
    }
    return char_count;
}

static MenuInstance* single_menu_init(MenuInitConfig_s *config,MenuInstance *pre_menu)
{

	MenuInstance *menu = &ALL_Menu_Instance[idx_ALL_Menu++];
	if(menu==NULL)
	{
		
	}
	memset(menu, 0, sizeof(MenuInstance));
	for(uint8_t i=0;i<MAX_MENU_NUM;i++)
	{
		menu->string[i] = config->string[i];
		//LOGERROR("%s","man!");
		menu->callback[i] = config->callback[i];
		menu->pre_menu = pre_menu;
		menu->CharNum[i] = count_chars((char *)config->string[i]);
		menu->pre_idx = config->pre_idx;
		if(config->next_menu_config[i]!=NULL)
		{
			menu->next_menu[i] = single_menu_init(config->next_menu_config[i],menu);
		}
	}
	return menu;
}
void menu_task(void)
{
	
	if((lower_limit_row_idx-upper_limit_row_idx)>=4)
	{
		LOGERROR("[menu]idx error!");
		return;
	}
	
	if(Key_Check(0,KEY_SINGLE))//前进
	{
		OLED_AnimUpdate();
		if(now_menu->next_menu[row_idx+upper_limit_row_idx]!=NULL)//不是最后一级，进入下一级菜单并更新相关参数
		{
			if(now_menu->callback[row_idx])
			{
				now_menu->callback[row_idx](row_idx);
			}
			now_menu=now_menu->next_menu[row_idx+upper_limit_row_idx];
			row_idx=0;
			upper_limit_row_idx=0;
			row_max_idx=0;
			for(uint8_t i=0;i<MAX_MENU_NUM;i++)
			{
				if(now_menu->string[i]!=NULL)
				{
					row_max_idx++;
				}
				else break;
			}
			if(row_max_idx>=4)row_max_idx=3;
			else row_max_idx = row_max_idx-1;
			lower_limit_row_idx = row_max_idx;
			OLED_Clear();
			for(uint8_t i=0;i<=row_max_idx;i++)
			{
				OLED_ShowString(0,i*16,(char *)now_menu->string[i+upper_limit_row_idx],OLED_8X16);
			}
			OLED_Animation(0,0,0,0,
			0,0,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16);
			
		}
		else if(now_menu->callback[row_idx])//如果是最后一级且存在回调函数就调用回调函数
		{
			now_menu->callback[row_idx](row_idx);
		}
		
	}
	else if(Key_Check(3,KEY_SINGLE))//后退
	{
		OLED_AnimUpdate();
		if(now_menu->pre_menu==NULL)//第一级不做处理
		{
			
		}
		else
		{
			OLED_Clear();
			row_idx = now_menu->pre_idx;
			now_menu = now_menu->pre_menu;
			row_max_idx = 0;
			for(uint8_t i=0;i<MAX_MENU_NUM;i++)
			{
				if(now_menu->string[i]!=NULL)
				{
					row_max_idx++;
				}
				else break;
			}
			if(row_max_idx>=4)row_max_idx=3;
			else row_max_idx = row_max_idx-1;
			lower_limit_row_idx = row_max_idx;
			if(row_idx>3)
			{
				upper_limit_row_idx = row_idx-3;
				lower_limit_row_idx = row_idx;
				row_idx = 3;
				for(uint8_t i=0;i<=row_max_idx;i++)
				{
					OLED_ShowString(0,i*16,(char *)now_menu->string[i+upper_limit_row_idx],OLED_8X16);
				}
				OLED_Animation(0,0,0,0,
				0,row_idx*16,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16);
			}
			else
			{
				upper_limit_row_idx = 0;
				lower_limit_row_idx = upper_limit_row_idx+row_max_idx;
				for(uint8_t i=0;i<=row_max_idx;i++)
				{
					OLED_ShowString(0,i*16,(char *)now_menu->string[i+upper_limit_row_idx],OLED_8X16);
				}
				OLED_Animation(0,0,0,0,
				0,row_idx*16,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16);
			}
		}
		
		
		
	}
	else if(Key_Check(2,KEY_SINGLE|KEY_REPEAT))//向上
	{
		OLED_AnimUpdate();
		if(row_idx>0)
		{
			for(uint8_t i=0;i<=row_max_idx;i++)
			{
				OLED_ShowString(0,i*16,(char *)now_menu->string[i+upper_limit_row_idx],OLED_8X16);
			}
			OLED_Animation(0,16*row_idx,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16,
			0,16*(row_idx-1),16*now_menu->CharNum[row_idx+upper_limit_row_idx-1],16);
			row_idx--;
		}
		else if(row_idx==0)
		{
			if(upper_limit_row_idx>0)
			{
				lower_limit_row_idx--;
				upper_limit_row_idx--;
				OLED_Clear();
				for(uint8_t i=0;i<=row_max_idx;i++)
				{
					OLED_ShowString(0,i*16,(char *)now_menu->string[i+upper_limit_row_idx],OLED_8X16);
				}
				OLED_Animation(0,0,0,0,
				0,0,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16);
			}
		}
	}
	else if(Key_Check(1,KEY_SINGLE|KEY_REPEAT))//向下
	{
		OLED_AnimUpdate();
		if(row_idx<row_max_idx)
		{
			for(uint8_t i=0;i<=row_max_idx;i++)
			{
				OLED_ShowString(0,i*16,(char *)now_menu->string[i+upper_limit_row_idx],OLED_8X16);
			}
			OLED_Animation(0,16*row_idx,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16,
			0,16*(row_idx+1),16*now_menu->CharNum[row_idx+upper_limit_row_idx+1],16);
			row_idx++;
		}
		else if(row_idx==row_max_idx)
		{
			if((lower_limit_row_idx<(MAX_MENU_NUM-1))&&(now_menu->string[lower_limit_row_idx+1]!=NULL))
			{
				lower_limit_row_idx++;
				upper_limit_row_idx++;
				OLED_Clear();
				for(uint8_t i=0;i<=row_max_idx;i++)
				{
					OLED_ShowString(0,i*16,(char *)now_menu->string[i+upper_limit_row_idx],OLED_8X16);
				}
				OLED_Animation(0,48,0,0,
				0,row_max_idx*16,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16);
			}
		}
	}
	
	OLED_Update();
}


	