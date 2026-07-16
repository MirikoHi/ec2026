#ifndef __MENU_H__
#define __MENU_H__
#include "ti_msp_dl_config.h"
#define MAX_MENU_NUM 6
#define MAX_ALL_MENU_NUM  10
/* 菜单处理函数指针 */
typedef void (*menu_callback)(uint8_t);

typedef struct MenuInstance{
		struct MenuInstance* next_menu[MAX_MENU_NUM];
		struct MenuInstance* pre_menu;
		menu_callback callback[MAX_MENU_NUM];
		char* string[MAX_MENU_NUM];
		uint8_t CharNum[MAX_MENU_NUM];
		uint8_t pre_idx;
}MenuInstance;

typedef struct MenuInitConfig{
	char* string[MAX_MENU_NUM];
	menu_callback callback[MAX_MENU_NUM];
	struct MenuInitConfig *next_menu_config[MAX_MENU_NUM];
	uint8_t pre_idx;
}MenuInitConfig_s;


void MenuInit(void);
void menu_task(void);
void MenuSetAnimationEnabled(uint8_t enabled);
uint8_t MenuAnimationEnabled(void);

#endif
