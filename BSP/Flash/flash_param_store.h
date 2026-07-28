#ifndef __FLASH_PARAM_STORE_H__
#define __FLASH_PARAM_STORE_H__

#include <stdint.h>
#include "../Algorithm/PID.h"
#include "w25q128jv_spi.h"

/**
  * @brief Flash 双槽参数存储库用法介绍
  * @note 1. 上电后调用 FlashParam_Init()，参数服务会自动读取两个槽
  * @note 2. 业务模块通过 FlashParam_GetActive() 获取当前参数
  * @note 3. 参数修改后调用 FlashParam_Save() 保存，库会自动写入备用槽
  * @note 4. 若一个槽损坏，加载时会用有效槽自动修复；若两个槽都坏，则写入默认参数
  * @note 5. FlashParam_RunSelfTest() 只用于调试，会擦写两个参数槽，正常运行不要调用
  */

/* 参数区使用 Flash 最后的两个 4KB 扇区，写入时只擦目标槽，保留旧槽作为回退。 */
#define FLASH_PARAM_SLOT_COUNT          2U
#define FLASH_PARAM_SLOT_SIZE           W25Q128JV_SECTOR_SIZE
#define FLASH_PARAM_SLOT0_ADDR          (W25Q128JV_TOTAL_SIZE - (2U * W25Q128JV_SECTOR_SIZE))
#define FLASH_PARAM_SLOT1_ADDR          (W25Q128JV_TOTAL_SIZE - W25Q128JV_SECTOR_SIZE)

/* 当前比赛动作预留 4 段直线和 4 次转弯，后续扩展需要同步提升版本号。 */
#define FLASH_PARAM_LINE_SEGMENT_COUNT  4U
#define FLASH_PARAM_TURN_SEGMENT_COUNT  4U

typedef enum
{
    FLASH_PARAM_OK = 0,
    FLASH_PARAM_ERR_PARAM = -1,
    FLASH_PARAM_ERR_FLASH = -2,
    FLASH_PARAM_ERR_INVALID = -3,
} FlashParam_Status_e;

typedef enum
{
    FLASH_PARAM_SOURCE_DEFAULT = 0,
    FLASH_PARAM_SOURCE_SLOT0,
    FLASH_PARAM_SOURCE_SLOT1,
} FlashParam_Source_e;

typedef struct
{
    /* 循迹、直线航向、转向三个 PID 参数。 */
    pid_init_config_s trace_pid;
    pid_init_config_s line_yaw_pid;
    pid_init_config_s turn_pid;

    /* 每段动作的距离和角度，单位分别是 m 和 deg。 */
    float line_distance_m[FLASH_PARAM_LINE_SEGMENT_COUNT];
    float turn_angle_deg[FLASH_PARAM_TURN_SEGMENT_COUNT];

    /* 直线速度曲线和完成判定参数。 */
    float line_accel_m;
    float line_slowdown_m;
    float line_min_speed_mps;
    float line_done_err_m;
    uint16_t line_done_ticks;
    uint16_t reserved0;

    /* 转向完成判定和动作默认速度。 */
    float turn_done_err_deg;
    uint16_t turn_done_ticks;
    uint16_t reserved1;
    float action_speed_mps;
    float turn_speed_mps;
} FlashParam_Data_s;

typedef struct
{
    FlashParam_Data_s params;
    FlashParam_Source_e source;
    uint32_t sequence;
    uint8_t slot_valid[FLASH_PARAM_SLOT_COUNT];
} FlashParam_State_s;

/**
  * @brief 初始化参数服务
  * @return FLASH_PARAM_OK 表示初始化完成，其他值表示 Flash 通信或参数处理异常
  * @note 用法：上电后先调用一次，之后其他模块再通过 FlashParam_GetActive() 读取参数
  * @note 会读取双槽参数；单槽损坏会自动修复，双槽都坏会写入默认参数
  */
FlashParam_Status_e FlashParam_Init(void);

/**
  * @brief 从 Flash 重新加载参数
  * @param params 输出当前参数，可填 NULL
  * @param source 输出参数来源，可填 NULL
  * @param sequence 输出当前参数序号，可填 NULL
  * @return FLASH_PARAM_OK 表示加载完成，其他值表示 Flash 通信或参数处理异常
  * @note 用法：启动流程建议调用 FlashParam_Init()；调试或手动重载参数时调用本函数
  * @note 加载时同样会执行双槽自动修复逻辑
  */
FlashParam_Status_e FlashParam_Load(FlashParam_Data_s *params,
                                    FlashParam_Source_e *source,
                                    uint32_t *sequence);

/**
  * @brief 保存一份新的参数
  * @param params 待保存参数结构体指针
  * @return FLASH_PARAM_OK 表示保存成功，其他值表示擦除、写入或校验失败
  * @note 用法：修改参数后调用 FlashParam_Save(&params)，成功后新参数成为当前活动参数
  * @note 写入非活动槽，写后回读校验成功才切换当前槽，旧槽会保留作为掉电回退
  */
FlashParam_Status_e FlashParam_Save(const FlashParam_Data_s *params);

/**
  * @brief 读取指定槽中的参数
  * @param slot 槽号，范围 0~1
  * @param params 输出该槽参数，可填 NULL
  * @param sequence 输出该槽序号，可填 NULL
  * @return FLASH_PARAM_OK 表示该槽有效，其他值表示槽号错误、Flash 读取失败或校验失败
  * @note 用法：调试时可读取某个槽的原始保存内容
  * @note 只读取和校验，不改变当前活动参数
  */
FlashParam_Status_e FlashParam_ReadSlot(uint8_t slot,
                                        FlashParam_Data_s *params,
                                        uint32_t *sequence);

/**
  * @brief 直接写入指定槽
  * @param slot 槽号，范围 0~1
  * @param params 待写入参数结构体指针
  * @param sequence 写入序号，数值越大代表参数越新
  * @return FLASH_PARAM_OK 表示写入并回读校验成功，其他值表示擦除、写入或校验失败
  * @note 用法：用于测试、恢复、版本迁移等指定槽写入场景
  * @note 会擦除目标槽；普通参数保存请优先使用 FlashParam_Save()
  */
FlashParam_Status_e FlashParam_WriteSlot(uint8_t slot,
                                         const FlashParam_Data_s *params,
                                         uint32_t sequence);

/**
  * @brief 格式化参数区
  * @return FLASH_PARAM_OK 表示两个槽擦除成功，其他值表示擦除失败
  * @note 用法：确认需要清空所有已保存参数时调用
  * @note 会擦除两个参数槽，并把内存状态恢复为默认参数
  */
FlashParam_Status_e FlashParam_Format(void);

/**
  * @brief 运行双槽参数自测
  * @return FLASH_PARAM_OK 表示双槽读写、切换和损坏回退测试全部通过
  * @note 用法：调试 Flash 参数机制时临时调用，观察 RTT log 中的 [param_test] 输出
  * @note 会擦写两个参数槽，测试结束会把测试前读取到的参数值重新保存
  */
FlashParam_Status_e FlashParam_RunSelfTest(void);

/**
  * @brief 获取默认参数
  * @param params 输出默认参数结构体指针
  * @note 用法：需要基于默认值修改并保存时，先调用本函数填充结构体
  */
void FlashParam_GetDefault(FlashParam_Data_s *params);

/**
  * @brief 获取当前活动参数指针
  * @return 当前活动参数结构体指针
  * @note 用法：模块初始化时通过该接口读取 PID、距离、速度等参数
  * @note 返回的是内部状态指针，业务代码只读取，不要直接修改
  */
const FlashParam_Data_s *FlashParam_GetActive(void);

/**
  * @brief 获取当前参数服务状态
  * @return 当前状态结构体指针
  * @note 用法：调试时查看来源槽、序号和双槽有效状态
  * @note 返回的是内部状态指针，业务代码只读取，不要直接修改
  */
const FlashParam_State_s *FlashParam_GetState(void);

/**
  * @brief 获取当前参数来源
  * @return FLASH_PARAM_SOURCE_DEFAULT、FLASH_PARAM_SOURCE_SLOT0 或 FLASH_PARAM_SOURCE_SLOT1
  * @note 用法：调试或日志输出时判断当前参数来自默认值还是某个槽
  */
FlashParam_Source_e FlashParam_GetSource(void);

/**
  * @brief 获取当前参数序号
  * @return 当前参数序号，数值越大代表保存时间越新
  * @note 用法：调试或日志输出时判断当前参数新旧
  */
uint32_t FlashParam_GetSequence(void);

#endif
