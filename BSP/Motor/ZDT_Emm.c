#include "ZDT_Emm.h"
#include <string.h>
#include <stdlib.h>

/**********************************************************
*** ZDT_Emm -- ZDT XS系列闭环步进电机 Emm固件模式 串口驱动
*** 原始作者：ZHANGDATOU (Emm_V5.0 闭环步进电机驱动库)
*** 移植适配：回调式UART抽象 + 响应解析
*** 淘宝店铺：https://zhangdatou.taobao.com
*** CSDN博客：https://blog.csdn.net/zhangdatou666
*** QQ交流群：262438510
**********************************************************/

/* ============================================================
 *  内部变量
 * ============================================================ */

/** UART发送回调函数指针，由用户通过 RegisterSendCallback() 注册 */
static ZDT_Emm_SendCallback_t emm_send_cb = NULL;

/** MMCL 多命令缓冲区 */
volatile uint16_t ZDT_Emm_MMCL_count = 0;
volatile uint16_t ZDT_Emm_MMCL_cmd[ZDT_EMM_MMCL_LEN] = {0};

/* ============================================================
 *  内部辅助函数
 * ============================================================ */

/**
 * @brief   通过回调发送命令帧（内部使用）
 * @param   cmd  命令字节数组
 * @param   len  发送长度
 */
static void emm_send(const uint8_t *cmd, uint8_t len)
{
    if (emm_send_cb != NULL) {
        emm_send_cb(cmd, len);
    }
}

/**
 * @brief   将命令追加到MMCL批量缓冲区
 * @param   cmd  命令字节数组
 * @param   len  命令长度
 */
static void emm_mmcl_append(const uint8_t *cmd, uint8_t len)
{
    uint8_t j;
    for (j = 0; j < len; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

/* ============================================================
 *  UART回调注册
 * ============================================================ */

/**
 * @brief   注册UART发送回调函数
 * @param   send_fn  用户提供的发送函数指针
 */
void ZDT_Emm_RegisterSendCallback(ZDT_Emm_SendCallback_t send_fn)
{
    emm_send_cb = send_fn;
}

/* ============================================================
 *  电机校准功能 (Calibration)
 * ============================================================ */

/**
 * @brief   触发编码器校准
 * @param   addr  电机地址
 * @retval  地址 + 命令码 + 命令状态 + 校验字节
 */
void ZDT_Emm_Trig_Encoder_Cal(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x06;      // 命令码
    cmd[2] = 0x45;      // 命令状态
    cmd[3] = 0x6B;      // 校验字节

    emm_send(cmd, 4);
}

/**
 * @brief   复位电机(Y42)
 * @param   addr  电机地址
 */
void ZDT_Emm_Reset_Motor(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x08;      // 命令码
    cmd[2] = 0x97;      // 命令状态
    cmd[3] = 0x6B;      // 校验字节

    emm_send(cmd, 4);
}

/**
 * @brief   将当前位置清零
 * @param   addr  电机地址
 */
void ZDT_Emm_Reset_CurPos_To_Zero(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x0A;      // 命令码
    cmd[2] = 0x6D;      // 命令状态
    cmd[3] = 0x6B;      // 校验字节

    emm_send(cmd, 4);
}

/**
 * @brief   清除堵转保护
 * @param   addr  电机地址
 */
void ZDT_Emm_Reset_Clog_Pro(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x0E;      // 命令码
    cmd[2] = 0x52;      // 命令状态
    cmd[3] = 0x6B;      // 校验字节

    emm_send(cmd, 4);
}

/**
 * @brief   恢复出厂设置
 * @param   addr  电机地址
 */
void ZDT_Emm_Restore_Motor(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x0F;      // 命令码
    cmd[2] = 0x5F;      // 命令状态
    cmd[3] = 0x6B;      // 校验字节

    emm_send(cmd, 4);
}

/* ============================================================
 *  运动控制功能 (Motion Control)
 * ============================================================ */

/**
 * @brief   批量发送MMCL命令(Y42)
 * @param   addr  电机地址
 */
void ZDT_Emm_Multi_Motor_Cmd(uint8_t addr)
{
    uint16_t i = 0, j = 0, len = 0;
    static volatile uint8_t cmd[ZDT_EMM_MMCL_LEN] = {0};

    if (ZDT_Emm_MMCL_count > 0) {
        len = ZDT_Emm_MMCL_count + 5;

        cmd[0] = addr;                      // 地址
        cmd[1] = 0xAA;                      // 命令码
        cmd[2] = (uint8_t)(len >> 8);       // 长度高8位
        cmd[3] = (uint8_t)(len);            // 长度低8位
        for (i = 0, j = 4; i < ZDT_Emm_MMCL_count; i++, j++) {
            cmd[j] = ZDT_Emm_MMCL_cmd[i];
        }
        cmd[j] = 0x6B; ++j;                 // 校验字节

        emm_send(cmd, j);
        ZDT_Emm_MMCL_count = 0;
    } else {
        ZDT_Emm_MMCL_count = 0;
    }
}

/**
 * @brief   使能信号控制
 * @param   addr   电机地址
 * @param   state  使能状态: true=使能电机, false=关断电机
 * @param   snF    同步运动标志: false=不启用, true=启用
 */
void ZDT_Emm_En_Control(uint8_t addr, bool state, bool snF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;              // 地址
    cmd[1] = 0xF3;              // 命令码
    cmd[2] = 0xAB;              // 命令状态
    cmd[3] = (uint8_t)state;    // 使能状态
    cmd[4] = snF;               // 同步运动标志
    cmd[5] = 0x6B;              // 校验字节

    emm_send(cmd, 6);
}

/**
 * @brief   速度模式控制
 * @param   addr  电机地址
 * @param   dir   方向: 0=CW, 非0=CCW
 * @param   vel   速度 0~5000 RPM
 * @param   acc   加速度 0~255 (注意: 0=直接启动)
 * @param   snF   同步运动标志
 */
void ZDT_Emm_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                         uint8_t acc, bool snF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;                      // 地址
    cmd[1] = 0xF6;                      // 命令码
    cmd[2] = dir;                       // 方向
    cmd[3] = (uint8_t)(vel >> 8);       // 速度(RPM)高8位
    cmd[4] = (uint8_t)(vel >> 0);       // 速度(RPM)低8位
    cmd[5] = acc;                       // 加速度 (注意: 0=直接启动)
    cmd[6] = snF;                       // 同步运动标志
    cmd[7] = 0x6B;                      // 校验字节

    emm_send(cmd, 8);
}

/**
 * @brief   位置模式控制
 * @param   addr  电机地址
 * @param   dir   方向: 0=CW, 非0=CCW
 * @param   vel   最大速度 0~5000 RPM
 * @param   acc   加速度 0~255 (注意: 0=直接启动)
 * @param   clk   脉冲数 0~(2^32-1)
 * @param   raF   运动模式: 0=增量(相对上次目标), 1=绝对位置, 2=增量(相对当前实时位置)
 * @param   snF   同步运动标志
 */
void ZDT_Emm_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                         uint8_t acc, uint32_t clk, uint8_t raF, bool snF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0]  = addr;                     // 地址
    cmd[1]  = 0xFD;                     // 命令码
    cmd[2]  = dir;                      // 方向
    cmd[3]  = (uint8_t)(vel >> 8);      // 速度(RPM)高8位
    cmd[4]  = (uint8_t)(vel >> 0);      // 速度(RPM)低8位
    cmd[5]  = acc;                      // 加速度 (注意: 0=直接启动)
    cmd[6]  = (uint8_t)(clk >> 24);     // 脉冲数(bit24-bit31)
    cmd[7]  = (uint8_t)(clk >> 16);     // 脉冲数(bit16-bit23)
    cmd[8]  = (uint8_t)(clk >> 8);      // 脉冲数(bit8-bit15)
    cmd[9]  = (uint8_t)(clk >> 0);      // 脉冲数(bit0-bit7)
    cmd[10] = raF;                      // 绝对/相对标志: 0=增量, 1=绝对, 2=相对当前位置增量
    cmd[11] = snF;                      // 同步运动标志: false=不启用, true=启用
    cmd[12] = 0x6B;                     // 校验字节

    emm_send(cmd, 13);
}

/**
 * @brief   设置快速位置模式运动参数
 * @param   addr  电机地址
 * @param   vel   最大速度 0~5000 RPM
 * @param   acc   加速度 0~255
 * @param   raF   运动模式标志
 * @param   snF   同步运动标志
 */
void ZDT_Emm_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc,
                             uint8_t raF, bool snF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;                      // 地址
    cmd[1] = 0xF1;                      // 命令码
    cmd[2] = (uint8_t)(vel >> 8);       // 速度(RPM)高8位
    cmd[3] = (uint8_t)(vel >> 0);       // 速度(RPM)低8位
    cmd[4] = acc;                       // 加速度 (注意: 0=直接启动)
    cmd[5] = raF;                       // 绝对/相对标志
    cmd[6] = snF;                       // 同步运动标志
    cmd[7] = 0x6B;                      // 校验字节

    emm_send(cmd, 8);
}

/**
 * @brief   快速位置模式控制 (需先调用 Set_QPos_Params)
 * @param   addr  电机地址
 * @param   clk   脉冲数(有符号): +3200=正转一圈, -3200=反转一圈 (16细分下)
 */
void ZDT_Emm_QPos_Control(uint8_t addr, int32_t clk)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0]  = addr;                     // 地址
    cmd[1]  = 0xFC;                     // 命令码
    cmd[2]  = (uint8_t)(clk >> 24);     // 脉冲数(bit24-bit31)
    cmd[3]  = (uint8_t)(clk >> 16);     // 脉冲数(bit16-bit23)
    cmd[4]  = (uint8_t)(clk >> 8);      // 脉冲数(bit8-bit15)
    cmd[5]  = (uint8_t)(clk >> 0);      // 脉冲数(bit0-bit7)
    cmd[6]  = 0x6B;                     // 校验字节

    emm_send(cmd, 7);
}

/**
 * @brief   立即停止
 * @param   addr  电机地址
 * @param   snF   同步运动标志
 */
void ZDT_Emm_Stop_Now(uint8_t addr, bool snF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0xFE;      // 命令码
    cmd[2] = 0x98;      // 命令状态
    cmd[3] = snF;       // 同步运动标志
    cmd[4] = 0x6B;      // 校验字节

    emm_send(cmd, 5);
}

/**
 * @brief   触发同步运动
 * @param   addr  电机地址
 */
void ZDT_Emm_Synchronous_motion(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0xFF;      // 命令码
    cmd[2] = 0x66;      // 命令状态
    cmd[3] = 0x6B;      // 校验字节

    emm_send(cmd, 4);
}

/* ============================================================
 *  原点返回功能 (Origin / Homing)
 * ============================================================ */

void ZDT_Emm_Origin_Set_O(uint8_t addr, bool svF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x93;      // 命令码
    cmd[2] = 0x88;      // 命令状态
    cmd[3] = svF;       // 是否存储: false=不存储, true=存储
    cmd[4] = 0x6B;      // 校验字节

    emm_send(cmd, 5);
}

void ZDT_Emm_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x9A;      // 命令码
    cmd[2] = o_mode;    // 回零模式: 0=就近找零点, 1=限位找零点, 2=单边限位碰撞, 3=单边限位堵转
    cmd[3] = snF;       // 同步运动标志
    cmd[4] = 0x6B;      // 校验字节

    emm_send(cmd, 5);
}

void ZDT_Emm_Origin_Interrupt(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x9C;      // 命令码
    cmd[2] = 0x48;      // 命令状态
    cmd[3] = 0x6B;      // 校验字节

    emm_send(cmd, 4);
}

void ZDT_Emm_Origin_Read_Params(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x22;      // 命令码
    cmd[2] = 0x6B;      // 校验字节

    emm_send(cmd, 3);
}

void ZDT_Emm_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode,
                                  uint8_t o_dir, uint16_t o_vel, uint32_t o_tm,
                                  uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms,
                                  bool potF)
{
    static volatile uint8_t cmd[32] = {0};

    cmd[0]  = addr;                     // 地址
    cmd[1]  = 0x4C;                     // 命令码
    cmd[2]  = 0xAE;                     // 命令状态
    cmd[3]  = svF;                      // 是否存储
    cmd[4]  = o_mode;                   // 回零模式
    cmd[5]  = o_dir;                    // 回零方向
    cmd[6]  = (uint8_t)(o_vel >> 8);    // 回零速度(RPM)高8位
    cmd[7]  = (uint8_t)(o_vel >> 0);    // 回零速度(RPM)低8位
    cmd[8]  = (uint8_t)(o_tm >> 24);    // 回零超时(bit24-bit31)
    cmd[9]  = (uint8_t)(o_tm >> 16);    // 回零超时(bit16-bit23)
    cmd[10] = (uint8_t)(o_tm >> 8);     // 回零超时(bit8-bit15)
    cmd[11] = (uint8_t)(o_tm >> 0);     // 回零超时(bit0-bit7)
    cmd[12] = (uint8_t)(sl_vel >> 8);   // 碰撞检测转速高8位
    cmd[13] = (uint8_t)(sl_vel >> 0);   // 碰撞检测转速低8位
    cmd[14] = (uint8_t)(sl_ma >> 8);    // 碰撞检测电流高8位
    cmd[15] = (uint8_t)(sl_ma >> 0);    // 碰撞检测电流低8位
    cmd[16] = (uint8_t)(sl_ms >> 8);    // 碰撞检测时间高8位
    cmd[17] = (uint8_t)(sl_ms >> 0);    // 碰撞检测时间低8位
    cmd[18] = potF;                     // 上电自动触发回零
    cmd[19] = 0x6B;                     // 校验字节

    emm_send(cmd, 20);
}

/* ============================================================
 *  读取系统参数 (System Parameters)
 * ============================================================ */

void ZDT_Emm_Auto_Return_Sys_Params_Timed(uint8_t addr, ZDT_Emm_SysParams_t s,
                                          uint16_t time_ms)
{
    uint8_t i = 0;
    static volatile uint8_t cmd[16] = {0};

    cmd[i] = addr; ++i;                     // 地址
    cmd[i] = 0x11; ++i;                     // 命令码
    cmd[i] = 0x18; ++i;                     // 命令状态

    switch (s) {                            // 信息参数类型
        case ZDT_EMM_S_VBUS : cmd[i] = 0x24; ++i; break;
        case ZDT_EMM_S_CBUS : cmd[i] = 0x26; ++i; break;
        case ZDT_EMM_S_CPHA : cmd[i] = 0x27; ++i; break;
        case ZDT_EMM_S_ENCO : cmd[i] = 0x29; ++i; break;
        case ZDT_EMM_S_CLKC : cmd[i] = 0x30; ++i; break;
        case ZDT_EMM_S_ENCL : cmd[i] = 0x31; ++i; break;
        case ZDT_EMM_S_CLKI : cmd[i] = 0x32; ++i; break;
        case ZDT_EMM_S_TPOS : cmd[i] = 0x33; ++i; break;
        case ZDT_EMM_S_SPOS : cmd[i] = 0x34; ++i; break;
        case ZDT_EMM_S_VEL  : cmd[i] = 0x35; ++i; break;
        case ZDT_EMM_S_CPOS : cmd[i] = 0x36; ++i; break;
        case ZDT_EMM_S_PERR : cmd[i] = 0x37; ++i; break;
        case ZDT_EMM_S_VBAT : cmd[i] = 0x38; ++i; break;
        case ZDT_EMM_S_TEMP : cmd[i] = 0x39; ++i; break;
        case ZDT_EMM_S_FLAG : cmd[i] = 0x3A; ++i; break;
        case ZDT_EMM_S_OFLAG: cmd[i] = 0x3B; ++i; break;
        case ZDT_EMM_S_OAF  : cmd[i] = 0x3C; ++i; break;
        case ZDT_EMM_S_PIN  : cmd[i] = 0x3D; ++i; break;
        default: break;
    }

    cmd[i] = (uint8_t)(time_ms >> 8);  ++i; // 定时时间高8位
    cmd[i] = (uint8_t)(time_ms >> 0);  ++i; // 定时时间低8位
    cmd[i] = 0x6B; ++i;                     // 校验字节

    emm_send(cmd, i);
}

void ZDT_Emm_Read_Sys_Params(uint8_t addr, ZDT_Emm_SysParams_t s)
{
    uint8_t i = 0;
    static volatile uint8_t cmd[16] = {0};

    cmd[i] = addr; ++i;                     // 地址

    switch (s) {                            // 命令码
        case ZDT_EMM_S_VBUS : cmd[i] = 0x24; ++i; break;
        case ZDT_EMM_S_CBUS : cmd[i] = 0x26; ++i; break;
        case ZDT_EMM_S_CPHA : cmd[i] = 0x27; ++i; break;
        case ZDT_EMM_S_ENCO : cmd[i] = 0x29; ++i; break;
        case ZDT_EMM_S_CLKC : cmd[i] = 0x30; ++i; break;
        case ZDT_EMM_S_ENCL : cmd[i] = 0x31; ++i; break;
        case ZDT_EMM_S_CLKI : cmd[i] = 0x32; ++i; break;
        case ZDT_EMM_S_TPOS : cmd[i] = 0x33; ++i; break;
        case ZDT_EMM_S_SPOS : cmd[i] = 0x34; ++i; break;
        case ZDT_EMM_S_VEL  : cmd[i] = 0x35; ++i; break;
        case ZDT_EMM_S_CPOS : cmd[i] = 0x36; ++i; break;
        case ZDT_EMM_S_PERR : cmd[i] = 0x37; ++i; break;
        case ZDT_EMM_S_VBAT : cmd[i] = 0x38; ++i; break;
        case ZDT_EMM_S_TEMP : cmd[i] = 0x39; ++i; break;
        case ZDT_EMM_S_FLAG : cmd[i] = 0x3A; ++i; break;
        case ZDT_EMM_S_OFLAG: cmd[i] = 0x3B; ++i; break;
        case ZDT_EMM_S_OAF  : cmd[i] = 0x3C; ++i; break;
        case ZDT_EMM_S_PIN  : cmd[i] = 0x3D; ++i; break;
        default: break;
    }

    cmd[i] = 0x6B; ++i;                     // 校验字节

    emm_send(cmd, i);
}

/* ============================================================
 *  读写电机配置参数 (Config)
 * ============================================================ */

void ZDT_Emm_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0xAE;      // 命令码
    cmd[2] = 0x4B;      // 命令状态
    cmd[3] = svF;       // 是否存储: false=不存储, true=存储
    cmd[4] = id;        // 默认电机ID为1, 可修改为1-255, 0为广播地址
    cmd[5] = 0x6B;      // 校验字节

    emm_send(cmd, 6);
}

void ZDT_Emm_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x84;      // 命令码
    cmd[2] = 0x8A;      // 命令状态
    cmd[3] = svF;       // 是否存储
    cmd[4] = mstep;     // 默认细分16, 可修改为1-255, 0为256细分
    cmd[5] = 0x6B;      // 校验字节

    emm_send(cmd, 6);
}

void ZDT_Emm_Modify_PDFlag(uint8_t addr, bool pdf)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x50;      // 命令码
    cmd[2] = pdf;       // 堵转标志
    cmd[3] = 0x6B;      // 校验字节

    emm_send(cmd, 4);
}

void ZDT_Emm_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;          // 地址
    cmd[1] = 0x46;          // 命令码
    cmd[2] = 0x69;          // 命令状态
    cmd[3] = svF;           // 是否存储
    cmd[4] = ctrl_mode;     // 控制模式: 默认=1, 0=开环模式, 1=闭环FOC模式
    cmd[5] = 0x6B;          // 校验字节

    emm_send(cmd, 6);
}

void ZDT_Emm_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0xD4;      // 命令码
    cmd[2] = 0x60;      // 命令状态
    cmd[3] = svF;       // 是否存储
    cmd[4] = dir;       // 电机运动方向: 默认CW, 0=CW(顺时针), 1=CCW
    cmd[5] = 0x6B;      // 校验字节

    emm_send(cmd, 6);
}

void ZDT_Emm_Modify_OM_mA(uint8_t addr, bool svF, uint16_t om_ma)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;                      // 地址
    cmd[1] = 0x44;                      // 命令码
    cmd[2] = 0x33;                      // 命令状态
    cmd[3] = svF;                       // 是否存储
    cmd[4] = (uint8_t)(om_ma >> 8);     // 开环模式运行电流(mA)高8位
    cmd[5] = (uint8_t)(om_ma >> 0);     // 开环模式运行电流(mA)低8位
    cmd[6] = 0x6B;                      // 校验字节

    emm_send(cmd, 7);
}

void ZDT_Emm_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;                      // 地址
    cmd[1] = 0x45;                      // 命令码
    cmd[2] = 0x66;                      // 命令状态
    cmd[3] = svF;                       // 是否存储
    cmd[4] = (uint8_t)(foc_mA >> 8);    // 闭环模式运行电流(mA)高8位
    cmd[5] = (uint8_t)(foc_mA >> 0);    // 闭环模式运行电流(mA)低8位
    cmd[6] = 0x6B;                      // 校验字节

    emm_send(cmd, 7);
}

void ZDT_Emm_Read_PID_Params(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x21;      // 命令码
    cmd[2] = 0x6B;      // 校验字节

    emm_send(cmd, 3);
}

void ZDT_Emm_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki,
                               uint32_t kd)
{
    static volatile uint8_t cmd[20] = {0};

    cmd[0]  = addr;                     // 地址
    cmd[1]  = 0x4A;                     // 命令码
    cmd[2]  = 0xC3;                     // 命令状态
    cmd[3]  = svF;                      // 是否存储
    cmd[4]  = (uint8_t)(kp >> 24);      // kp
    cmd[5]  = (uint8_t)(kp >> 16);
    cmd[6]  = (uint8_t)(kp >> 8);
    cmd[7]  = (uint8_t)(kp >> 0);
    cmd[8]  = (uint8_t)(ki >> 24);      // ki
    cmd[9]  = (uint8_t)(ki >> 16);
    cmd[10] = (uint8_t)(ki >> 8);
    cmd[11] = (uint8_t)(ki >> 0);
    cmd[12] = (uint8_t)(kd >> 24);      // kd
    cmd[13] = (uint8_t)(kd >> 16);
    cmd[14] = (uint8_t)(kd >> 8);
    cmd[15] = (uint8_t)(kd >> 0);
    cmd[16] = 0x6B;                     // 校验字节

    emm_send(cmd, 17);
}

void ZDT_Emm_Read_System_State_Params(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x43;      // 命令码
    cmd[2] = 0x7A;      // 命令状态
    cmd[3] = 0x6B;      // 校验字节

    emm_send(cmd, 4);
}

void ZDT_Emm_Read_Motor_Conf_Params(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;      // 地址
    cmd[1] = 0x42;      // 命令码
    cmd[2] = 0x6C;      // 命令状态
    cmd[3] = 0x6B;      // 校验字节

    emm_send(cmd, 4);
}

/* ============================================================
 *  MMCL 批量命令版本 (追加到缓冲区，不立即发送)
 * ============================================================ */

void ZDT_Emm_MMCL_Trig_Encoder_Cal(uint8_t addr)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x06; cmd[2] = 0x45; cmd[3] = 0x6B;
    for (j = 0; j < 4; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Reset_Motor(uint8_t addr)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x08; cmd[2] = 0x97; cmd[3] = 0x6B;
    for (j = 0; j < 4; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Reset_CurPos_To_Zero(uint8_t addr)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x0A; cmd[2] = 0x6D; cmd[3] = 0x6B;
    for (j = 0; j < 4; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Reset_Clog_Pro(uint8_t addr)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x0E; cmd[2] = 0x52; cmd[3] = 0x6B;
    for (j = 0; j < 4; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Restore_Motor(uint8_t addr)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x0F; cmd[2] = 0x5F; cmd[3] = 0x6B;
    for (j = 0; j < 4; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_En_Control(uint8_t addr, bool state, bool snF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0xF3; cmd[2] = 0xAB;
    cmd[3] = (uint8_t)state; cmd[4] = snF; cmd[5] = 0x6B;
    for (j = 0; j < 6; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                              uint8_t acc, bool snF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0xF6; cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8); cmd[4] = (uint8_t)(vel >> 0);
    cmd[5] = acc; cmd[6] = snF; cmd[7] = 0x6B;
    for (j = 0; j < 8; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                              uint8_t acc, uint32_t clk, uint8_t raF, bool snF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0]  = addr; cmd[1] = 0xFD; cmd[2] = dir;
    cmd[3]  = (uint8_t)(vel >> 8); cmd[4] = (uint8_t)(vel >> 0);
    cmd[5]  = acc;
    cmd[6]  = (uint8_t)(clk >> 24); cmd[7] = (uint8_t)(clk >> 16);
    cmd[8]  = (uint8_t)(clk >> 8);  cmd[9] = (uint8_t)(clk >> 0);
    cmd[10] = raF; cmd[11] = snF; cmd[12] = 0x6B;
    for (j = 0; j < 13; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc,
                                  uint8_t raF, bool snF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0xF1;
    cmd[2] = (uint8_t)(vel >> 8); cmd[3] = (uint8_t)(vel >> 0);
    cmd[4] = acc; cmd[5] = raF; cmd[6] = snF; cmd[7] = 0x6B;
    for (j = 0; j < 8; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_QPos_Control(uint8_t addr, int32_t clk)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0xFC;
    cmd[2] = (uint8_t)(clk >> 24); cmd[3] = (uint8_t)(clk >> 16);
    cmd[4] = (uint8_t)(clk >> 8);  cmd[5] = (uint8_t)(clk >> 0);
    cmd[6] = 0x6B;
    for (j = 0; j < 7; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Stop_Now(uint8_t addr, bool snF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0xFE; cmd[2] = 0x98;
    cmd[3] = snF; cmd[4] = 0x6B;
    for (j = 0; j < 5; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Synchronous_motion(uint8_t addr)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0xFF; cmd[2] = 0x66; cmd[3] = 0x6B;
    for (j = 0; j < 4; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Origin_Set_O(uint8_t addr, bool svF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x93; cmd[2] = 0x88;
    cmd[3] = svF; cmd[4] = 0x6B;
    for (j = 0; j < 5; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x9A; cmd[2] = o_mode;
    cmd[3] = snF; cmd[4] = 0x6B;
    for (j = 0; j < 5; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Origin_Interrupt(uint8_t addr)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x9C; cmd[2] = 0x48; cmd[3] = 0x6B;
    for (j = 0; j < 4; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode,
                                       uint8_t o_dir, uint16_t o_vel, uint32_t o_tm,
                                       uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms,
                                       bool potF)
{
    uint8_t j, cmd[32] = {0};
    cmd[0]  = addr; cmd[1] = 0x4C; cmd[2] = 0xAE;
    cmd[3]  = svF; cmd[4] = o_mode; cmd[5] = o_dir;
    cmd[6]  = (uint8_t)(o_vel >> 8);  cmd[7]  = (uint8_t)(o_vel >> 0);
    cmd[8]  = (uint8_t)(o_tm >> 24);  cmd[9]  = (uint8_t)(o_tm >> 16);
    cmd[10] = (uint8_t)(o_tm >> 8);   cmd[11] = (uint8_t)(o_tm >> 0);
    cmd[12] = (uint8_t)(sl_vel >> 8); cmd[13] = (uint8_t)(sl_vel >> 0);
    cmd[14] = (uint8_t)(sl_ma >> 8);  cmd[15] = (uint8_t)(sl_ma >> 0);
    cmd[16] = (uint8_t)(sl_ms >> 8);  cmd[17] = (uint8_t)(sl_ms >> 0);
    cmd[18] = potF; cmd[19] = 0x6B;
    for (j = 0; j < 20; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr, ZDT_Emm_SysParams_t s,
                                               uint16_t time_ms)
{
    uint8_t i = 0, j = 0, cmd[16] = {0};

    cmd[i] = addr; ++i;
    cmd[i] = 0x11; ++i;
    cmd[i] = 0x18; ++i;

    switch (s) {
        case ZDT_EMM_S_VBUS : cmd[i] = 0x24; ++i; break;
        case ZDT_EMM_S_CBUS : cmd[i] = 0x26; ++i; break;
        case ZDT_EMM_S_CPHA : cmd[i] = 0x27; ++i; break;
        case ZDT_EMM_S_ENCO : cmd[i] = 0x29; ++i; break;
        case ZDT_EMM_S_CLKC : cmd[i] = 0x30; ++i; break;
        case ZDT_EMM_S_ENCL : cmd[i] = 0x31; ++i; break;
        case ZDT_EMM_S_CLKI : cmd[i] = 0x32; ++i; break;
        case ZDT_EMM_S_TPOS : cmd[i] = 0x33; ++i; break;
        case ZDT_EMM_S_SPOS : cmd[i] = 0x34; ++i; break;
        case ZDT_EMM_S_VEL  : cmd[i] = 0x35; ++i; break;
        case ZDT_EMM_S_CPOS : cmd[i] = 0x36; ++i; break;
        case ZDT_EMM_S_PERR : cmd[i] = 0x37; ++i; break;
        case ZDT_EMM_S_VBAT : cmd[i] = 0x38; ++i; break;
        case ZDT_EMM_S_TEMP : cmd[i] = 0x39; ++i; break;
        case ZDT_EMM_S_FLAG : cmd[i] = 0x3A; ++i; break;
        case ZDT_EMM_S_OFLAG: cmd[i] = 0x3B; ++i; break;
        case ZDT_EMM_S_OAF  : cmd[i] = 0x3C; ++i; break;
        case ZDT_EMM_S_PIN  : cmd[i] = 0x3D; ++i; break;
        default: break;
    }

    cmd[i] = (uint8_t)(time_ms >> 8);  ++i;
    cmd[i] = (uint8_t)(time_ms >> 0);  ++i;
    cmd[i] = 0x6B; ++i;

    for (j = 0; j < i; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

void ZDT_Emm_MMCL_Read_Sys_Params(uint8_t addr, ZDT_Emm_SysParams_t s)
{
    uint8_t i = 0, j = 0, cmd[16] = {0};

    cmd[i] = addr; ++i;

    switch (s) {
        case ZDT_EMM_S_VBUS : cmd[i] = 0x24; ++i; break;
        case ZDT_EMM_S_CBUS : cmd[i] = 0x26; ++i; break;
        case ZDT_EMM_S_CPHA : cmd[i] = 0x27; ++i; break;
        case ZDT_EMM_S_ENCO : cmd[i] = 0x29; ++i; break;
        case ZDT_EMM_S_CLKC : cmd[i] = 0x30; ++i; break;
        case ZDT_EMM_S_ENCL : cmd[i] = 0x31; ++i; break;
        case ZDT_EMM_S_CLKI : cmd[i] = 0x32; ++i; break;
        case ZDT_EMM_S_TPOS : cmd[i] = 0x33; ++i; break;
        case ZDT_EMM_S_SPOS : cmd[i] = 0x34; ++i; break;
        case ZDT_EMM_S_VEL  : cmd[i] = 0x35; ++i; break;
        case ZDT_EMM_S_CPOS : cmd[i] = 0x36; ++i; break;
        case ZDT_EMM_S_PERR : cmd[i] = 0x37; ++i; break;
        case ZDT_EMM_S_VBAT : cmd[i] = 0x38; ++i; break;
        case ZDT_EMM_S_TEMP : cmd[i] = 0x39; ++i; break;
        case ZDT_EMM_S_FLAG : cmd[i] = 0x3A; ++i; break;
        case ZDT_EMM_S_OFLAG: cmd[i] = 0x3B; ++i; break;
        case ZDT_EMM_S_OAF  : cmd[i] = 0x3C; ++i; break;
        case ZDT_EMM_S_PIN  : cmd[i] = 0x3D; ++i; break;
        default: break;
    }

    cmd[i] = 0x6B; ++i;

    for (j = 0; j < i; j++) {
        ZDT_Emm_MMCL_cmd[ZDT_Emm_MMCL_count] = cmd[j];
        ++ZDT_Emm_MMCL_count;
    }
}

/* ============================================================
 *  响应解析工具函数 (Response Parsing Utilities)
 * ============================================================ */

/**
 * @brief   解析int32返回值 (用于CPOS/TPOS/SPOS/ENCO等4字节参数)
 *
 * 响应帧格式 (以S_CPOS为例):
 *   rx[0] = 电机地址
 *   rx[1] = 命令字节 (0x36 = S_CPOS)
 *   rx[2] = 方向符号 (0=正, 非0=负)
 *   rx[3..6] = 32位数据值 (big-endian)
 *
 * @param   rx           接收到的原始数据
 * @param   rx_len       接收数据长度
 * @param   expected_cmd 期望的命令字节
 * @param   out_sign     输出符号: 0=正, 非0=负
 * @return  解析出的int32值, 校验失败返回0
 */
int32_t ZDT_Emm_Parse_Int32_Response(const uint8_t *rx, uint8_t rx_len,
                                     uint8_t expected_cmd, uint8_t *out_sign)
{
    if (rx == NULL || rx_len < 7) {
        return 0;
    }

    /* rx[1] = 命令码, rx_len应 = 8 (addr + cmd + sign + 4bytes data + checksum) */
    if (rx[1] != expected_cmd || rx_len < 8) {
        return 0;
    }

    if (out_sign != NULL) {
        *out_sign = rx[2];
    }

    return (int32_t)(
        ((uint32_t)rx[3] << 24) |
        ((uint32_t)rx[4] << 16) |
        ((uint32_t)rx[5] << 8)  |
        ((uint32_t)rx[6] << 0)
    );
}

/**
 * @brief   解析int16返回值 (用于VEL/电流/电压等2字节参数)
 *
 * 响应帧格式 (以S_VEL为例):
 *   rx[0] = 电机地址
 *   rx[1] = 命令字节 (0x35 = S_VEL)
 *   rx[2] = 方向符号
 *   rx[3..4] = 16位数据值 (big-endian)
 *
 * @param   rx           接收到的原始数据
 * @param   rx_len       接收数据长度
 * @param   expected_cmd 期望的命令字节
 * @param   out_sign     输出符号
 * @return  解析出的int16值, 校验失败返回0
 */
int16_t ZDT_Emm_Parse_Int16_Response(const uint8_t *rx, uint8_t rx_len,
                                     uint8_t expected_cmd, uint8_t *out_sign)
{
    if (rx == NULL || rx_len < 5) {
        return 0;
    }

    /* rx_len应 = 6 (addr + cmd + sign + 2bytes data + checksum) */
    if (rx[1] != expected_cmd || rx_len < 6) {
        return 0;
    }

    if (out_sign != NULL) {
        *out_sign = rx[2];
    }

    return (int16_t)(
        ((uint16_t)rx[3] << 8) |
        ((uint16_t)rx[4] << 0)
    );
}

/**
 * @brief   解析实时位置为角度值
 *
 * 转换公式: encoder_value * 360.0 / 65536.0
 * 如果符号位(rx[2])非0, 角度取反
 *
 * @param   rx      接收到的原始数据
 * @param   rx_len  接收数据长度
 * @return  角度值(度), 校验失败返回0.0f
 */
float ZDT_Emm_Parse_Position_Deg(const uint8_t *rx, uint8_t rx_len)
{
    uint8_t sign = 0;
    int32_t raw = ZDT_Emm_Parse_Int32_Response(rx, rx_len, 0x36, &sign);

    if (raw == 0 && rx_len < 8) {
        return 0.0f;
    }

    float angle = (float)raw * 360.0f / 65536.0f;

    if (sign) {
        angle = -angle;
    }

    return angle;
}

/**
 * @brief   解析实时转速
 *
 * 响应格式: rx[1]=0x35, rx[2]=sign, rx[3..4]=uint16 speed value
 *
 * @param   rx      接收到的原始数据
 * @param   rx_len  接收数据长度
 * @return  转速值(RPM), 校验失败返回0.0f
 */
float ZDT_Emm_Parse_Velocity_RPM(const uint8_t *rx, uint8_t rx_len)
{
    uint8_t sign = 0;
    int16_t raw = ZDT_Emm_Parse_Int16_Response(rx, rx_len, 0x35, &sign);

    if (raw == 0 && rx_len < 6) {
        return 0.0f;
    }

    float vel = (float)raw;

    if (sign) {
        vel = -vel;
    }

    return vel;
}

/* ============================================================
 *  RX FIFO 接口实现
 * ============================================================ */

/** ZDT电机RX FIFO实例，由UART ISR写入，应用层读取 */
FIFO_t zdt_emm_rx_fifo = {0};

/**
 * @brief   ISR中调用，将接收字节推入RX FIFO
 * @param   byte  接收到的字节
 */
void ZDT_Emm_RxPushByte(uint8_t byte)
{
    fifo_enQueue(&zdt_emm_rx_fifo, byte);
}

/**
 * @brief   从RX FIFO中读取一帧响应数据
 * @param   buf      输出缓冲区
 * @param   max_len  最大读取长度
 * @return  实际读取的字节数
 */
uint8_t ZDT_Emm_GetResponse(uint8_t *buf, uint8_t max_len)
{
    uint8_t len = fifo_queueLength(&zdt_emm_rx_fifo);

    if (len > max_len) {
        len = max_len;
    }

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = fifo_deQueue(&zdt_emm_rx_fifo);
    }

    return len;
}

/**
 * @brief   清空RX FIFO（丢弃所有缓冲数据）
 */
void ZDT_Emm_FlushRx(void)
{
    fifo_initQueue(&zdt_emm_rx_fifo);
}

/* ============================================================
 *  电机实例 API 实现
 * ============================================================ */

/** 查询超时计数器 (循环次数, 约 1000 次 ≈ 5ms @80MHz) */
#define ZDT_EMM_QUERY_TIMEOUT   50000U

/** 查询轮询间隔 (空循环, 约 1µs) */
#define ZDT_EMM_QUERY_TICK      50U

/**
 * @brief   等待 RX FIFO 中收到至少 min_len 字节
 * @param   min_len  最少需要的字节数
 * @param   timeout  超时计数值
 * @return  true=数据就绪, false=超时
 */
static bool emm_wait_response(uint8_t min_len, uint32_t timeout)
{
    uint32_t cnt = 0;
    while (fifo_queueLength(&zdt_emm_rx_fifo) < min_len) {
        /* 简单忙等待 */
        for (volatile uint32_t i = 0; i < ZDT_EMM_QUERY_TICK; i++) {}
        cnt++;
        if (cnt > timeout) {
            return false;
        }
    }
    return true;
}

/**
 * @brief   初始化电机实例
 */
ZDT_Emm_Motor_t *ZDT_Emm_Motor_Create(uint8_t addr)
{
    ZDT_Emm_Motor_t *motor = (ZDT_Emm_Motor_t *)malloc(sizeof(ZDT_Emm_Motor_t));
    if (motor == NULL) {
        return NULL;
    }
    memset(motor, 0, sizeof(ZDT_Emm_Motor_t));
    motor->addr = addr;
    motor->online = false;
    return motor;
}

/**
 * @brief   查询并更新电机实时位置
 *
 * 发送 S_CPOS 查询 → 等待 8 字节响应 → 解析 → 更新 motor->position_deg
 */
bool ZDT_Emm_Motor_UpdatePosition(ZDT_Emm_Motor_t *motor)
{
    uint8_t rx[16];
    uint8_t sign = 0;
    int32_t raw;
    uint8_t rx_len;

    if (motor == NULL) {
        return false;
    }

    /* 清空 RX FIFO (丢弃旧数据) */
    ZDT_Emm_FlushRx();

    /* 发送位置查询 */
    ZDT_Emm_Read_Sys_Params(motor->addr, ZDT_EMM_S_CPOS);

    /* 等待 8 字节响应 (addr + cmd + sign + 4B data + checksum(=0x6B)) */
    if (!emm_wait_response(8, ZDT_EMM_QUERY_TIMEOUT)) {
        motor->online = false;
        return false;
    }

    /* 读取并解析 */
    rx_len = ZDT_Emm_GetResponse(rx, sizeof(rx));
    raw = ZDT_Emm_Parse_Int32_Response(rx, rx_len, 0x36, &sign);

    motor->raw_position = raw;

    /* 转角度: raw * 360 / 65536 */
	motor->position_deg = (float)raw * 360.0f / 65536.0f;
    if (sign) {
        motor->position_deg = -motor->position_deg;
    }

    motor->online = true;
    return true;
}

/**
 * @brief   查询并更新电机实时转速
 *
 * 发送 S_VEL 查询 → 等待 6 字节响应 → 解析 → 更新 motor->speed_rpm
 */
bool ZDT_Emm_Motor_UpdateSpeed(ZDT_Emm_Motor_t *motor)
{
    uint8_t rx[16];
    uint8_t sign = 0;
    int16_t raw;
    uint8_t rx_len;

    if (motor == NULL) {
        return false;
    }

    /* 清空 RX FIFO */
    ZDT_Emm_FlushRx();

    /* 发送转速查询 */
    ZDT_Emm_Read_Sys_Params(motor->addr, ZDT_EMM_S_VEL);

    /* 等待 6 字节响应 (addr + cmd + sign + 2B data + checksum) */
    if (!emm_wait_response(6, ZDT_EMM_QUERY_TIMEOUT)) {
        motor->online = false;
        return false;
    }

    /* 读取并解析 */
    rx_len = ZDT_Emm_GetResponse(rx, sizeof(rx));
    raw = ZDT_Emm_Parse_Int16_Response(rx, rx_len, 0x35, &sign);

    motor->raw_speed = raw;
    motor->speed_rpm = (float)raw;
    if (sign) {
        motor->speed_rpm = -motor->speed_rpm;
    }

    motor->online = true;
    return true;
}

/**
 * @brief   查询并更新位置+转速
 */
void ZDT_Emm_Motor_UpdateAll(ZDT_Emm_Motor_t *motor)
{
    ZDT_Emm_Motor_UpdatePosition(motor);
    ZDT_Emm_Motor_UpdateSpeed(motor);
}
