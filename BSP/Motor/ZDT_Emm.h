#ifndef _ZDT_EMM_H_
#define _ZDT_EMM_H_

#include "ti_msp_dl_config.h"
#include "fifo.h"
#include <stdbool.h>
#include <stdint.h>

/**********************************************************
*** ZDT_Emm -- ZDT XS系列闭环步进电机 Emm固件模式 串口驱动
*** 原始作者：ZHANGDATOU (Emm_V5.0 闭环步进电机驱动库)
*** 移植适配：回调式UART抽象 + 响应解析
*** 淘宝店铺：https://zhangdatou.taobao.com
*** CSDN博客：https://blog.csdn.net/zhangdatou666
*** QQ交流群：262438510
**********************************************************/

/* ============================================================
 *  宏定义
 * ============================================================ */
#define ZDT_EMM_MMCL_LEN    512         ///< 多命令缓冲区大小

/* ============================================================
 *  类型定义
 * ============================================================ */

/** @brief UART发送回调函数类型
 *  用户需实现此函数，通过所选UART发送指定长度的数据
 *  @param data  待发送的数据指针
 *  @param len   发送字节数
 */
typedef void (*ZDT_Emm_SendCallback_t)(const uint8_t *data, uint8_t len);

/** @brief 系统参数枚举
 *  用于 Emm_V5_Read_Sys_Params / Emm_V5_Auto_Return_Sys_Params_Timed
 */
typedef enum {
    ZDT_EMM_S_VBUS  = 5,    ///< 读取总线电压
    ZDT_EMM_S_CBUS  = 6,    ///< 读取总线电流
    ZDT_EMM_S_CPHA  = 7,    ///< 读取相电流
    ZDT_EMM_S_ENCO  = 8,    ///< 读取编码器原始值
    ZDT_EMM_S_CLKC  = 9,    ///< 读取实时脉冲数
    ZDT_EMM_S_ENCL  = 10,   ///< 读取编码器线性化校准后的值
    ZDT_EMM_S_CLKI  = 11,   ///< 读取输入脉冲数
    ZDT_EMM_S_TPOS  = 12,   ///< 读取电机目标位置
    ZDT_EMM_S_SPOS  = 13,   ///< 读取电机实时设定的目标位置
    ZDT_EMM_S_VEL   = 14,   ///< 读取电机实时转速
    ZDT_EMM_S_CPOS  = 15,   ///< 读取电机实时位置
    ZDT_EMM_S_PERR  = 16,   ///< 读取电机位置误差
    ZDT_EMM_S_VBAT  = 17,   ///< 读取线圈/后备电池电压(Y42)
    ZDT_EMM_S_TEMP  = 18,   ///< 读取电机实时温度(Y42)
    ZDT_EMM_S_FLAG  = 19,   ///< 读取电机状态标志位
    ZDT_EMM_S_OFLAG = 20,   ///< 读取原点状态标志位
    ZDT_EMM_S_OAF   = 21,   ///< 读取电机状态 + 原点状态标志位(Y42)
    ZDT_EMM_S_PIN   = 22,   ///< 读取引脚状态(Y42)
} ZDT_Emm_SysParams_t;

/* ============================================================
 *  电机实例结构体
 * ============================================================ */

typedef struct {
    uint8_t  addr;          ///< 电机地址 (1-255)
    float    position_deg;  ///< 当前位置 (度), 0~360
    float    speed_rpm;     ///< 当前转速 (RPM)
    int32_t  raw_position;  ///< 原始编码器值 (0~65535)
    int16_t  raw_speed;     ///< 原始转速值
    bool     online;        ///< 通信是否正常
} ZDT_Emm_Motor_t;

/* ============================================================
 *  全局变量 (MMCL 多命令缓冲区)
 * ============================================================ */
extern volatile uint16_t ZDT_Emm_MMCL_count;
extern volatile uint16_t ZDT_Emm_MMCL_cmd[ZDT_EMM_MMCL_LEN];

/* ============================================================
 *  UART回调注册
 * ============================================================ */

/**
 * @brief   注册UART发送回调函数
 * @param   send_fn  用户提供的发送函数指针
 */
void ZDT_Emm_RegisterSendCallback(ZDT_Emm_SendCallback_t send_fn);

/* ============================================================
 *  电机校准功能 (Calibration)
 * ============================================================ */

void ZDT_Emm_Trig_Encoder_Cal(uint8_t addr);                  ///< 触发编码器校准
void ZDT_Emm_Reset_Motor(uint8_t addr);                       ///< 复位电机(Y42)
void ZDT_Emm_Reset_CurPos_To_Zero(uint8_t addr);              ///< 将当前位置清零
void ZDT_Emm_Reset_Clog_Pro(uint8_t addr);                    ///< 清除堵转保护
void ZDT_Emm_Restore_Motor(uint8_t addr);                     ///< 恢复出厂设置

/* ============================================================
 *  运动控制功能 (Motion Control)
 * ============================================================ */

void ZDT_Emm_Multi_Motor_Cmd(uint8_t addr);                   ///< 发送批量命令(Y42)

/**
 * @brief   使能信号控制
 * @param   addr   电机地址
 * @param   state  使能状态: true=使能电机, false=关断电机
 * @param   snF    同步运动标志: false=不启用, true=启用
 */
void ZDT_Emm_En_Control(uint8_t addr, bool state, bool snF);

/**
 * @brief   速度模式控制
 * @param   addr  电机地址
 * @param   dir   方向: 0=CW, 非0=CCW
 * @param   vel   速度 0~5000 RPM
 * @param   acc   加速度 0~255 (注意: 0=直接启动)
 * @param   snF   同步运动标志
 */
void ZDT_Emm_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                         uint8_t acc, bool snF);

/**
 * @brief   位置模式控制
 * @param   addr  电机地址
 * @param   dir   方向: 0=CW, 非0=CCW
 * @param   vel   最大速度 0~5000 RPM
 * @param   acc   加速度 0~255 (注意: 0=直接启动)
 * @param   clk   脉冲数 0~(2^32-1)
 * @param   raF   运动模式标志:
 *                0=相对上一次目标位置做增量运动
 *                1=绝对位置运动
 *                2=相对当前实时位置做增量运动
 * @param   snF   同步运动标志
 */
void ZDT_Emm_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                         uint8_t acc, uint32_t clk, uint8_t raF, bool snF);

/**
 * @brief   设置快速位置模式运动参数
 * @param   addr  电机地址
 * @param   vel   最大速度 0~5000 RPM
 * @param   acc   加速度 0~255
 * @param   raF   运动模式标志 (同Pos_Control)
 * @param   snF   同步运动标志
 */
void ZDT_Emm_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc,
                             uint8_t raF, bool snF);

/**
 * @brief   快速位置模式控制 (需先调用 Set_QPos_Params 设置参数)
 * @param   addr  电机地址
 * @param   clk   脉冲数(有符号): +3200=正转一圈, -3200=反转一圈 (16细分下)
 */
void ZDT_Emm_QPos_Control(uint8_t addr, int32_t clk);

/**
 * @brief   立即停止
 * @param   addr  电机地址
 * @param   snF   同步运动标志
 */
void ZDT_Emm_Stop_Now(uint8_t addr, bool snF);

/**
 * @brief   触发同步运动 (所有snF=true的待执行命令同步开始)
 * @param   addr  电机地址
 */
void ZDT_Emm_Synchronous_motion(uint8_t addr);

/* ============================================================
 *  原点返回功能 (Origin / Homing)
 * ============================================================ */

void ZDT_Emm_Origin_Set_O(uint8_t addr, bool svF);
void ZDT_Emm_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
void ZDT_Emm_Origin_Interrupt(uint8_t addr);
void ZDT_Emm_Origin_Read_Params(uint8_t addr);

/**
 * @brief   修改回零参数
 * @param   addr    电机地址
 * @param   svF     是否存储: false=不存储, true=存储至Flash
 * @param   o_mode  回零模式: 0=单圈就近找零点, 1=单圈限位找零点,
 *                  2=单圈单边限位碰撞回零, 3=单圈单边限位堵转回零
 * @param   o_dir   回零方向: 0=CW, 非0=CCW
 * @param   o_vel   回零速度 RPM
 * @param   o_tm    回零超时时间 ms
 * @param   sl_vel  单边限位碰撞检测转速 RPM
 * @param   sl_ma   单边限位碰撞检测电流 Ma
 * @param   sl_ms   单边限位碰撞检测时间 Ms
 * @param   potF    上电自动触发回零: false=不使能, true=使能
 */
void ZDT_Emm_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode,
                                  uint8_t o_dir, uint16_t o_vel, uint32_t o_tm,
                                  uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);

/* ============================================================
 *  读取系统参数 (System Parameters)
 * ============================================================ */

void ZDT_Emm_Auto_Return_Sys_Params_Timed(uint8_t addr, ZDT_Emm_SysParams_t s,
                                          uint16_t time_ms);
void ZDT_Emm_Read_Sys_Params(uint8_t addr, ZDT_Emm_SysParams_t s);

/* ============================================================
 *  读写电机配置参数 (Config)
 * ============================================================ */

void ZDT_Emm_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id);
void ZDT_Emm_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep);
void ZDT_Emm_Modify_PDFlag(uint8_t addr, bool pdf);
void ZDT_Emm_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode);
void ZDT_Emm_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir);
void ZDT_Emm_Modify_OM_mA(uint8_t addr, bool svF, uint16_t om_ma);
void ZDT_Emm_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA);
void ZDT_Emm_Read_PID_Params(uint8_t addr);
void ZDT_Emm_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki,
                               uint32_t kd);
void ZDT_Emm_Read_System_State_Params(uint8_t addr);
void ZDT_Emm_Read_Motor_Conf_Params(uint8_t addr);

/* ============================================================
 *  MMCL 批量命令版本 (追加到命令缓冲区，不立即发送)
 *  调用 ZDT_Emm_Multi_Motor_Cmd() 执行批量发送
 * ============================================================ */

void ZDT_Emm_MMCL_Trig_Encoder_Cal(uint8_t addr);
void ZDT_Emm_MMCL_Reset_Motor(uint8_t addr);
void ZDT_Emm_MMCL_Reset_CurPos_To_Zero(uint8_t addr);
void ZDT_Emm_MMCL_Reset_Clog_Pro(uint8_t addr);
void ZDT_Emm_MMCL_Restore_Motor(uint8_t addr);
void ZDT_Emm_MMCL_En_Control(uint8_t addr, bool state, bool snF);
void ZDT_Emm_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                              uint8_t acc, bool snF);
void ZDT_Emm_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                              uint8_t acc, uint32_t clk, uint8_t raF, bool snF);
void ZDT_Emm_MMCL_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc,
                                  uint8_t raF, bool snF);
void ZDT_Emm_MMCL_QPos_Control(uint8_t addr, int32_t clk);
void ZDT_Emm_MMCL_Stop_Now(uint8_t addr, bool snF);
void ZDT_Emm_MMCL_Synchronous_motion(uint8_t addr);
void ZDT_Emm_MMCL_Origin_Set_O(uint8_t addr, bool svF);
void ZDT_Emm_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
void ZDT_Emm_MMCL_Origin_Interrupt(uint8_t addr);
void ZDT_Emm_MMCL_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode,
                                       uint8_t o_dir, uint16_t o_vel, uint32_t o_tm,
                                       uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
void ZDT_Emm_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr, ZDT_Emm_SysParams_t s,
                                               uint16_t time_ms);
void ZDT_Emm_MMCL_Read_Sys_Params(uint8_t addr, ZDT_Emm_SysParams_t s);

/* ============================================================
 *  响应解析工具函数 (Response Parsing Utilities)
 *  用于解析电机返回的原始数据帧
 * ============================================================ */

/**
 * @brief   解析int32返回值 (用于CPOS/TPOS/SPOS/ENCO等4字节参数)
 * @param   rx          接收到的原始数据
 * @param   rx_len      接收数据长度
 * @param   expected_cmd 期望的命令字节 (如0x36=S_CPOS)
 * @param   out_sign    输出符号: 0=正, 非0=负
 * @return  解析出的int32值, 校验失败返回0
 */
int32_t ZDT_Emm_Parse_Int32_Response(const uint8_t *rx, uint8_t rx_len,
                                     uint8_t expected_cmd, uint8_t *out_sign);

/**
 * @brief   解析int16返回值 (用于VEL/电流/电压等2字节参数)
 * @param   rx          接收到的原始数据
 * @param   rx_len      接收数据长度
 * @param   expected_cmd 期望的命令字节 (如0x35=S_VEL)
 * @param   out_sign    输出符号
 * @return  解析出的int16值, 校验失败返回0
 */
int16_t ZDT_Emm_Parse_Int16_Response(const uint8_t *rx, uint8_t rx_len,
                                     uint8_t expected_cmd, uint8_t *out_sign);

/**
 * @brief   解析实时位置为角度值
 * @param   rx          接收到的原始数据
 * @param   rx_len      接收数据长度
 * @return  角度值(度), 校验失败返回0.0f
 * @note    转换公式: encoder_value * 360.0 / 65536.0
 */
float ZDT_Emm_Parse_Position_Deg(const uint8_t *rx, uint8_t rx_len);

/**
 * @brief   解析实时转速
 * @param   rx          接收到的原始数据
 * @param   rx_len      接收数据长度
 * @return  转速值(RPM), 校验失败返回0.0f
 */
float ZDT_Emm_Parse_Velocity_RPM(const uint8_t *rx, uint8_t rx_len);

/* ============================================================
 *  RX FIFO 接口 (供 UART ISR 使用)
 * ============================================================ */

/** ZDT电机RX FIFO，由UART ISR写入，应用层读取 */
extern FIFO_t zdt_emm_rx_fifo;

/** @brief ISR中调用，将接收字节推入RX FIFO */
void ZDT_Emm_RxPushByte(uint8_t byte);

/** @brief 从RX FIFO中读取一帧响应数据 */
uint8_t ZDT_Emm_GetResponse(uint8_t *buf, uint8_t max_len);

/** @brief 清空RX FIFO */
void ZDT_Emm_FlushRx(void);

/* ============================================================
 *  电机实例 API (封装查询和状态读取)
 * ============================================================ */

/**
 * @brief   初始化电机实例
 * @param   addr  电机地址
 * @return  电机实例指针，失败返回 NULL
 */
ZDT_Emm_Motor_t *ZDT_Emm_Motor_Create(uint8_t addr);

/**
 * @brief   查询并更新电机实时位置
 * @param   motor  电机实例
 * @return  true=成功, false=超时/通信失败
 * @note    阻塞调用，约需 2-5ms
 */
bool ZDT_Emm_Motor_UpdatePosition(ZDT_Emm_Motor_t *motor);

/**
 * @brief   查询并更新电机实时转速
 * @param   motor  电机实例
 * @return  true=成功, false=超时/通信失败
 * @note    阻塞调用，约需 2-5ms
 */
bool ZDT_Emm_Motor_UpdateSpeed(ZDT_Emm_Motor_t *motor);

/**
 * @brief   查询并更新位置+转速
 * @param   motor  电机实例
 * @note    阻塞调用，约需 5-10ms
 */
void ZDT_Emm_Motor_UpdateAll(ZDT_Emm_Motor_t *motor);

#endif /* _ZDT_EMM_H_ */
