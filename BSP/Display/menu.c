/**
 * @file    menu.c
 * @brief   多级OLED菜单系统实现
 * @details 实现了基于递归初始化的多级菜单框架，支持导航（前/后/上/下）、
 *          动态翻页（菜单项>4时）以及回调函数绑定。菜单通过 FreeRTOS 任务
 *          menu_task() 周期轮询按键输入。
 *
 *          菜单数据结构说明：
 *          - MenuInitConfig_s：静态配置结构体，编译期定义菜单层级与回调
 *          - MenuInstance：运行时菜单实例，含前后级指针、显示字符串、回调等
 *          - 初始化时 single_menu_init() 递归遍历配置树，构建实例链表
 *          - 每屏最多显示 4 行（row_max_idx = 0~3），超出通过上下界索引翻页
 *
 */

#include "menu.h"
#include "KEY.h"
#include "OLED.h"
#include "bsp_log.h"
#include "string.h"
#include <locale.h>
#include "FreeRTOS.h"
#include "task.h"
#include "../../APP/Chassis.h"
#include "../../APP/Robot_Cmd.h"
#include "../../APP/Gimbal.h"

//当前激活的菜单实例指针（指向 ALL_Menu_Instance 中的某一项） */
MenuInstance* now_menu;

//屏幕光标所在行索引（0~3，对应 OLED 的第 0~3 行） */
uint8_t row_idx = 0;

//当前屏幕可显示的最高行索引（0~3），用于边界判定 */
uint8_t row_max_idx = 0;

//当前显示窗口在 now_menu->string[] 数组中的起始索引 */
uint8_t upper_limit_row_idx;

//当前显示窗口在 now_menu->string[] 数组中的末尾索引（闭区间） */
uint8_t lower_limit_row_idx;
static uint8_t menu_animation_enabled = 1U;
static uint8_t menu_last_refresh_width = 0U;
static uint8_t menu_last_refresh_height = 0U;

//所有菜单实例的静态存储池，避免动态内存分配 */
MenuInstance ALL_Menu_Instance[MAX_ALL_MENU_NUM] = {0};

//已分配的菜单实例计数，single_menu_init() 调用时自增 */
uint8_t idx_ALL_Menu = 0;

static MenuInstance* single_menu_init(MenuInitConfig_s *config, MenuInstance *pre_menu);
int count_chars(const char *str);
static uint8_t menu_row_width(uint8_t row, uint8_t upper);
static uint8_t menu_window_width(void);
static uint8_t menu_window_height(void);
static void menu_refresh_window(void);
static void menu_switch_highlight(uint8_t old_row, uint8_t old_upper, uint8_t new_row, uint8_t new_upper);

/**
 * @brief  菜单系统总初始化函数
 * @details 按树形结构定义三级菜单配置（一级→二级→三级），调用
 *          single_menu_init() 递归构建 MenuInstance 链表，完成 OLED
 *          初始化并绘制首屏界面动画。
 *
 *          菜单结构（实际项目）：
 *          - 一级菜单：电机控制 / 控制方式 / 任务
 *          - 二级菜单[0]：使能 / 失能 → Motor_Cmd_CallBack()
 *          - 二级菜单[1]：程序控制 / 按键控制 → Control_Switch_Callback()
 *            - 三级菜单[1]：普通 / 巡线 / 陀螺仪 / 位置 → Chassis_Mode_Switch_Callback()
 *          - 二级菜单[2]：任务一 / 任务二 → Task_Callback()
 *
 * @note   本函数在系统启动时调用一次，不应重复调用
 * @note   字符串以 UTF-8 编码存储，CharNum 字段记录实际字符数（而非字节数）
 */
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
	OLED_ReverseArea(0,0,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16);
	menu_refresh_window();
}

static uint8_t menu_row_width(uint8_t row, uint8_t upper)
{
	return (uint8_t)(16U * now_menu->CharNum[row + upper]);
}

static uint8_t menu_window_width(void)
{
	uint8_t max_width = 0;
	for(uint8_t i = 0; i <= row_max_idx; i++)
	{
		uint8_t width = menu_row_width(i, upper_limit_row_idx);
		if(width > max_width)
		{
			max_width = width;
		}
	}
	return max_width;
}

static uint8_t menu_window_height(void)
{
	return (uint8_t)((row_max_idx + 1U) * 16U);
}

static void menu_refresh_window(void)
{
	uint8_t width = menu_window_width();
	uint8_t height = menu_window_height();
	uint8_t refresh_width;
	uint8_t refresh_height;
	if(width == 0U)
	{
		width = 1U;
	}
	if(height == 0U)
	{
		height = 16U;
	}
	refresh_width = (width > menu_last_refresh_width) ? width : menu_last_refresh_width;
	refresh_height = (height > menu_last_refresh_height) ? height : menu_last_refresh_height;
	OLED_UpdateArea(0, 0, refresh_width, refresh_height);
	menu_last_refresh_width = width;
	menu_last_refresh_height = height;
}

static void menu_switch_highlight(uint8_t old_row, uint8_t old_upper, uint8_t new_row, uint8_t new_upper)
{
	uint8_t old_width = menu_row_width(old_row, old_upper);
	uint8_t new_width = menu_row_width(new_row, new_upper);

	if (menu_animation_enabled != 0U)
	{
		OLED_Animation(0, old_row * 16U, old_width, 16,
			0, new_row * 16U, new_width, 16);
		return;
	}

	if(old_width > 0U)
	{
		OLED_ReverseArea(0, old_row * 16U, old_width, 16);
		OLED_UpdateArea(0, old_row * 16U, old_width, 16);
	}
	if(new_width > 0U)
	{
		OLED_ReverseArea(0, new_row * 16U, new_width, 16);
		OLED_UpdateArea(0, new_row * 16U, new_width, 16);
	}
}

void MenuSetAnimationEnabled(uint8_t enabled)
{
	menu_animation_enabled = (enabled != 0U) ? 1U : 0U;
	if (menu_animation_enabled == 0U)
	{
		OLED_AnimUpdate();
	}
}

uint8_t MenuAnimationEnabled(void)
{
	return menu_animation_enabled;
}

/**
 * @brief  统计 UTF-8 字符串中的实际字符数（而非字节数）
 * @param  str 以 NULL 结尾的 UTF-8 编码字符串
 * @return 字符串中的 Unicode 字符个数，若 str 为 NULL 则返回 0
 *
 * @details 手动解析 UTF-8 编码规则：
 *          - 0xxxxxxx（1 字节）→ ASCII / 单字节字符
 *          - 110xxxxx（2 字节）→ 拉丁扩展、希腊字母等
 *          - 1110xxxx（3 字节）→ 中日韩统一表意文字（CJK）
 *          - 11110xxx（4 字节）→ emoji、罕见汉字等
 *          - 非法序列字节跳过 1 字节并计为 1 字符
 *
 * @note   该函数用于计算菜单字符串在 OLED 上显示的字符宽度，
 *         与 CharNum[] 字段配套使用
 */
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

/**
 * @brief  递归创建单个菜单实例（内部静态函数）
 * @param  config   指向菜单初始化配置结构体的指针，包含菜单项字符串、回调和下级菜单指针
 * @param  pre_menu 指向前一级菜单实例的指针，用于构建返回链（首级菜单传入 NULL）
 * @return 指向新创建的 MenuInstance 的指针，若 ALL_Menu_Instance 池已满则可能异常
 *
 * @details 从编译期静态配置 MenuInitConfig_s 转换为运行时 MenuInstance 链表：
 *          1. 从全局池 ALL_Menu_Instance 中分配一个新实例
 *          2. 遍历 config->string[] 数组，逐项拷贝字符串指针、回调函数、前置索引
 *          3. 调用 count_chars() 计算每个菜单项的显示字符宽度
 *          4. 若当前项存在下级菜单配置（next_menu_config[i] != NULL），递归调用自身
 *             构建子树，并将返回的实例指针存入 next_menu[i]
 *          5. 返回构建完成的实例指针，供父级链接
 *
 * @note   该函数为 static 限定，仅在本文件内使用
 * @note   实例从 ALL_Menu_Instance 静态数组中分配，idx_ALL_Menu 单调递增，无释放机制
 * @warning 若菜单总数超过 MAX_ALL_MENU_NUM，将导致数组越界，需确保配置总数在限制内
 */
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

/**
 * @brief  菜单任务处理函数（由 FreeRTOS 周期性调度）
 * @details 轮询四个按键输入并执行对应的菜单导航逻辑：
 *
 *          | 按键 | 功能 | 行为 |
 *          |------|------|------|
 *          | KEY0 | 前进 / 确认 | 有下级菜单→进入下级；无下级但有回调→执行回调 |
 *          | KEY3 | 后退 / 返回 | 有上级菜单→返回上级并恢复上次选中行；顶级→无操作 |
 *          | KEY2 | 向上 | 光标上移；已在顶部则向上翻页 |
 *          | KEY1 | 向下 | 光标下移；已在底部则向下翻页 |
 *
 *          **翻页机制：**
 *          当菜单项超过 4 条时，屏幕仅显示 4 行（row 0~3），通过
 *          upper_limit_row_idx / lower_limit_row_idx 维护滑动窗口。
 *          窗口大小 = lower_limit_row_idx - upper_limit_row_idx + 1 ≤ 4
 *
 *          **动画处理：**
 *          每次导航操作前调用 OLED_AnimUpdate() 保存当前帧，
 *          操作后调用 OLED_Animation() 执行平滑过渡动画，
 *          最终调用 OLED_Update() 刷新显示。
 *
 *          所有按键检测均支持 KEY_SINGLE（单击），方向键另外支持 KEY_REPEAT（长按连发）。
 *
 * @note   该函数应由 FreeRTOS 任务周期调用，典型调度周期为 10~20ms
 * @note   按键索引映射：0=前进, 1=向下, 2=向上, 3=后退（由 Key_Check 参数决定）
 */
void menu_task(void)
{
	if ((menu_animation_enabled != 0U) && OLED_AnimationBusy())
	{
		OLED_AnimationStep();
		return;
	}

	if((lower_limit_row_idx-upper_limit_row_idx)>=4)
	{
		LOGERROR("[menu]idx error!");
		return;
	}
	
	if(Key_Check(0,KEY_SINGLE))//前进
	{
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
			OLED_ReverseArea(0,0,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16);
			menu_refresh_window();
			
		}
		else if(now_menu->callback[row_idx])//如果是最后一级且存在回调函数就调用回调函数
		{
			now_menu->callback[row_idx](row_idx);
		}
		
	}
	else if(Key_Check(3,KEY_SINGLE))//后退
	{
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
				OLED_ReverseArea(0,row_idx*16,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16);
				menu_refresh_window();
			}
			else
			{
				upper_limit_row_idx = 0;
				lower_limit_row_idx = upper_limit_row_idx+row_max_idx;
				for(uint8_t i=0;i<=row_max_idx;i++)
				{
					OLED_ShowString(0,i*16,(char *)now_menu->string[i+upper_limit_row_idx],OLED_8X16);
				}
				OLED_ReverseArea(0,row_idx*16,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16);
				menu_refresh_window();
			}
		}
		
		
		
	}
	else if(Key_Check(2,KEY_SINGLE|KEY_REPEAT))//向上
	{
		if(row_idx>0)
		{
			uint8_t old_row = row_idx;
			row_idx--;
			menu_switch_highlight(old_row, upper_limit_row_idx, row_idx, upper_limit_row_idx);
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
				OLED_ReverseArea(0,0,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16);
				menu_refresh_window();
			}
		}
	}
	else if(Key_Check(1,KEY_SINGLE|KEY_REPEAT))//向下
	{
		if(row_idx<row_max_idx)
		{
			uint8_t old_row = row_idx;
			row_idx++;
			menu_switch_highlight(old_row, upper_limit_row_idx, row_idx, upper_limit_row_idx);
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
				OLED_ReverseArea(0,row_max_idx*16,16*now_menu->CharNum[row_idx+upper_limit_row_idx],16);
				menu_refresh_window();
			}
		}
	}
	
}
