// #include "Emm_V5.h"
// #include "stdlib.h"
// #include "string.h"
// #include "Robot_Cmd.h"
// #include "misc.h"
// /**********************************************************
// ***	Emm_V5.0步进闭环控制例程
// ***	编写作者：ZHANGDATOU
// ***	技术支持：张大头闭环伺服
// ***	淘宝店铺：https://zhangdatou.taobao.com
// ***	CSDN博客：http s://blog.csdn.net/zhangdatou666
// ***	qq交流群：262438510
// **********************************************************/
// static uint8_t idx=0;
// static Step_t *step_instance[8]={NULL};
// static uint32_t arr[2][3]={{1,0,0},{2,0,0}};
// static uint8_t uart_data = 0;
// uint8_t i=0;
// #define UART_RX_BUF_SIZE 128  // 缓冲区大小（根据需求调整）
// static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
// static volatile uint16_t uart_rx_head = 0;  // 写指针
// static volatile uint16_t uart_rx_tail = 0;  // 读指针
// static uint16_t test=0;
// static uint8_t data_arr[13]={0};
// uint8_t cnt=0;
// Step_t *Step_Init(Step_Config_s *motor)
// {
//
// 	NVIC_ClearPendingIRQ(StepMotorUart_INST_INT_IRQN);
// 	NVIC_EnableIRQ(StepMotorUart_INST_INT_IRQN);
// 	DL_UART_clearInterruptStatus(StepMotorUart_INST,DL_UART_INTERRUPT_RX);//清除中断标志位,开启接收步进电机信息的串口中断
// 	Step_t *instance =(Step_t*)malloc(sizeof (Step_t));
// 	memset(instance,0,sizeof(Step_t));
// 	instance->addr =motor->addr;
// 	instance->mode=motor->mode;
// 	instance->vel=motor->vel;
// 	instance->acc=motor->acc;
// 	instance->snf=motor->snf;
// 	step_instance[idx++]=instance;
// 	return instance;
//
// }
// void StepMotorUart_INST_IRQHandler(void)
// {
//
//     //如果产生了串口中断
//     switch( DL_UART_getPendingInterrupt(StepMotorUart_INST) )
//     {
//         case DL_UART_IIDX_RX://如果是接收中断
//
//             uart_data = DL_UART_Main_receiveData(StepMotorUart_INST);
// 				if(uart_data==0x00)
// 				{
// 					break;
// 				}
//             //将保存的数据再发送出去
// 				else if(uart_data==0x6B)
// 				{
//
// 					data_arr[cnt++]=uart_data;
// 					cnt=0;
//
// 				}
// 				else
//
// 				{
// 					data_arr[cnt++]=uart_data;
// 					cnt++;
// 				}
//             break;
//
//         default://其他的串口中断
//             break;
//     }
//
// }
//
// void Step()
// {
// 	Step_t *motor;
//
// 	 for ( i = 0;  i<= idx; ++i)
//
// 	{
// 		vTaskDelay(pdMS_TO_TICKS(2));
// 		motor =step_instance[i];
// 		if(Is_send(motor,motor->addr))
// 			{
//
// 	  switch (motor->mode)
// 				{
// 			case Reset_To_Zero:
// 				Emm_V5_Origin_Trigger_Return(motor->addr, 0, false);
// 				break;
// 			case Speed_Mode:
// 				Emm_V5_Vel_Control(motor->addr,motor->dir,motor->vel,motor->acc,motor->snf);
// 				break;
// 			case Pos_Mode:
// 				Emm_V5_Pos_Control(motor->addr,motor->dir,motor->vel,motor->acc,motor->clk,motor->raF,motor->snf);
// 				break;
// 			default:
// 				break;
// 				}
// 			}
// 	}
//
// }
// bool Is_send(Step_t *motor,uint8_t num)
// {
//
// 	if(motor->addr==arr[num-1][0]&&(motor->clk!=arr[num-1][1]||motor->mode!=arr[num-1][2]))
// 	{
// 		arr[num-1][0]=motor->addr;
// 		arr[num-1][1]=motor->clk;
// 		arr[num-1][2]=motor->mode;
// 		return 1;
//
// 	}
// 	else
// 	{
// 		return 0;
// 	}
//
//
// }
// void Step_ReturnTo_Zero(Step_t *motor)
// {
// 	motor->mode = Reset_To_Zero;
// }
// void Step_Set_Position(Step_t *motor,float angle,Step_PositionRaf_e raf)
// {
// 	uint32_t clk = 3200*abs_out(angle)/360.0f;
// 	uint8_t dir = angle>0?0:1;
//
// 	motor->dir = dir;
// 	motor->clk = clk;
// 	motor->raF = 1;//绝对位置
// 	motor->mode = Pos_Mode;
// }
//
// 	void StepMotorUart_send_char(uint8_t ch)
// {
//     //当串口0忙的时候等待，不忙的时候再发送传进来的字符
//     while( DL_UART_isBusy(StepMotorUart_INST) == true );
//     //发送单个字符
//     DL_UART_Main_transmitData(StepMotorUart_INST, ch);
// }
// void uart0_send_string(uint8_t* str, uint32_t length)
// {
//     if (str == NULL) return;  // 安全检查
//
//     for (uint32_t i = 0; i < length; i++) {
//         StepMotorUart_send_char(str[i]);
//     }
// }
// /**
//   * @brief    将当前位置清零
//   * @param    addr  ：电机地址
//   * @retval   地址 + 功能码 + 命令状态 + 校验字节
//   */
//
// void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
// {
//   uint8_t cmd[16] = {0};
//
//   // 装载命令
//   cmd[0] =  addr;                       // 地址
//   cmd[1] =  0x0A;                       // 功能码
//   cmd[2] =  0x6D;                       // 辅助码
//   cmd[3] =  0x6B;                       // 校验字节
//
//   // 发送命令
// 	uart0_send_string((uint8_t *)cmd,4);
// }
//
//
// /**
//   * @brief    解除堵转保护
//   * @param    addr  ：电机地址
//   * @retval   地址 + 功能码 + 命令状态 + 校验字节
//   */\
//
// //void Emm_V5_Reset_Clog_Pro(uint8_t addr)
// //{
// //  uint8_t cmd[16] = {0};
// //
// //  // 装载命令
// //  cmd[0] =  addr;                       // 地址
// //  cmd[1] =  0x0E;                       // 功能码
// //  cmd[2] =  0x52;                       // 辅助码
// //  cmd[3] =  0x6B;                       // 校验字节
// //
// //  // 发送命令
// //  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 4);
// //}
//
// ///**
// //  * @brief    读取系统参数
// //  * @param    addr  ：电机地址
// //  * @param    s     ：系统参数类型
// //  * @retval   地址 + 功能码 + 命令状态 + 校验字节
// //  */
// //void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
// //{
// //  uint8_t i = 0;
// //  uint8_t cmd[16] = {0};
// //
// //  // 装载命令
// //  cmd[i] = addr; ++i;                   // 地址
//
// //  switch(s)                             // 功能码
// //  {
// //    case S_VER  : cmd[i] = 0x1F; ++i; break;
// //    case S_RL   : cmd[i] = 0x20; ++i; break;
// //    case S_PID  : cmd[i] = 0x21; ++i; break;
// //    case S_VBUS : cmd[i] = 0x24; ++i; break;
// //    case S_CPHA : cmd[i] = 0x27; ++i; break;
// //    case S_ENCL : cmd[i] = 0x31; ++i; break;
// //    case S_TPOS : cmd[i] = 0x33; ++i; break;
// //    case S_VEL  : cmd[i] = 0x35; ++i; break;
// //    case S_CPOS : cmd[i] = 0x36; ++i; break;
// //    case S_PERR : cmd[i] = 0x37; ++i; break;
// //    case S_FLAG : cmd[i] = 0x3A; ++i; break;
// //    case S_ORG  : cmd[i] = 0x3B; ++i; break;
// //    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
// //    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
// //    default: break;
// //  }
//
// //  cmd[i] = 0x6B; ++i;                   // 校验字节
// //
// //  // 发送命令
// //  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, i);
// //}
//
// ///**
// //  * @brief    修改开环/闭环控制模式
// //  * @param    addr     ：电机地址
// //  * @param    svF      ：是否存储标志，false为不存储，true为存储
// //  * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
// //  * @retval   地址 + 功能码 + 命令状态 + 校验字节
// //  */
// //void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
// //{
// //  uint8_t cmd[16] = {0};
// //
// //  // 装载命令
// //  cmd[0] =  addr;                       // 地址
// //  cmd[1] =  0x46;                       // 功能码
// //  cmd[2] =  0x69;                       // 辅助码
// //  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
// //  cmd[4] =  ctrl_mode;                  // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
// //  cmd[5] =  0x6B;                       // 校验字节
// //
// //  // 发送命令
// //  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 6);
// //}
//
// ///**
// //  * @brief    使能信号控制
// //  * @param    addr  ：电机地址
// //  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
// //  * @param    snF   ：多机同步标志 ，false为不启用，true为启用
// //  * @retval   地址 + 功能码 + 命令状态 + 校验字节
// //  */
// void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
// {
//   uint8_t cmd[16] = {0};
//
//   // 装载命令
//   cmd[0] =  addr;                       // 地址
//   cmd[1] =  0xF3;                       // 功能码
//   cmd[2] =  0xAB;                       // 辅助码
//   cmd[3] =  (uint8_t)state;             // 使能状态
//   cmd[4] =  snF;                        // 多机同步运动标志
//   cmd[5] =  0x6B;                       // 校验字节
//
//   // 发送命令
//   uart0_send_string((uint8_t *)cmd, 6);
// }
//
// ///**
// //  * @brief    速度模式
// //  * @param    addr：电机地址
// //  * @param    dir ：方向       ，0为CW，其余值为CCW
// //  * @param    vel ：速度       ，范围0 - 5000RPM
// //  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
// //  * @param    snF ：多机同步标志，false为不启用，true为启用
// //  * @retval   地址 + 功能码 + 命令状态 + 校验字节
// //  */
// void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
// {
//   uint8_t cmd[16] = {0};
//
//   // 装载命令
//   cmd[0] =  addr;                       // 地址
//   cmd[1] =  0xF6;                       // 功能码
//   cmd[2] =  dir;                        // 方向
//   cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
//   cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
//   cmd[5] =  acc;                        // 加速度，注意：0是直接启动
//   cmd[6] =  snF;                        // 多机同步运动标志
//   cmd[7] =  0x6B;                       // 校验字节
//
//   // 发送命令
//   uart0_send_string((uint8_t *)cmd,8);
// }
//
// ///**
// //  * @brief    位置模式
// //  * @param    addr：电机地址
// //  * @param    dir ：方向        ，0为CW，其余值为CCW
// //  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
// //  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
// //  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
// //  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
// //  * @param    snF ：多机同步标志 ，false为不启用，true为启用
// //  * @retval   地址 + 功能码 + 命令状态 + 校验字节
// //  */
// void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
// {
//   uint8_t cmd[16] = {0};
// 	bool test=0;
//   // 装载命令
//   cmd[0]  =  addr;                      // 地址
//   cmd[1]  =  0xFD;                      // 功能码
//   cmd[2]  =  dir;                       // 方向
//   cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
//   cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节
//   cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
//   cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
//   cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
//   cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
//   cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
//   cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
//   cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
//   cmd[12] =  0x6B;                      // 校验字节
//   // 发送命令
//  uart0_send_string((uint8_t *)cmd,13);
// }
//
// ///**
// //  * @brief    立即停止（所有控制模式都通用）
// //  * @param    addr  ：电机地址
// //  * @param    snF   ：多机同步标志，false为不启用，true为启用
// //  * @retval   地址 + 功能码 + 命令状态 + 校验字节
// //  */
// //void Emm_V5_Stop_Now(uint8_t addr, bool snF)
// //{
// //  uint8_t cmd[16] = {0};
// //
// //  // 装载命令
// //  cmd[0] =  addr;                       // 地址
// //  cmd[1] =  0xFE;                       // 功能码
// //  cmd[2] =  0x98;                       // 辅助码
// //  cmd[3] =  snF;                        // 多机同步运动标志
// //  cmd[4] =  0x6B;                       // 校验字节
// //
// //  // 发送命令
// //  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 5);
// //}
//
// ///**
// //  * @brief    多机同步运动
// //  * @param    addr  ：电机地址
// //  * @retval   地址 + 功能码 + 命令状态 + 校验字节
// //  */
// //void Emm_V5_Synchronous_motion(uint8_t addr)
// //{
// //  uint8_t cmd[16] = {0};
// //
// //  // 装载命令
// //  cmd[0] =  addr;                       // 地址
// //  cmd[1] =  0xFF;                       // 功能码
// //  cmd[2] =  0x66;                       // 辅助码
// //  cmd[3] =  0x6B;                       // 校验字节
// //
// //  // 发送命令
// //  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 4);
// //}
//
// ///**
// //  * @brief    设置单圈回零的零点位置
// //  * @param    addr  ：电机地址
// //  * @param    svF   ：是否存储标志，false为不存储，true为存储
// //  * @retval   地址 + 功能码 + 命令状态 + 校验字节
// //  */
// //void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
// //{
// //   uint8_t cmd[16] = {0};
// //
// //  // 装载命令
// //  cmd[0] =  addr;                       // 地址
// //  cmd[1] =  0x93;                       // 功能码
// //  cmd[2] =  0x88;                       // 辅助码
// //  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
// //  cmd[4] =  0x6B;                       // 校验字节
// //
// //  // 发送命令
// //  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 5);
// //}
//
// ///**
// //  * @brief    修改回零参数
// //  * @param    addr  ：电机地址
// //  * @param    svF   ：是否存储标志，false为不存储，true为存储
// //  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
// //  * @param    o_dir  ：回零方向，0为CW，其余值为CCW
// //  * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
// //  * @param    o_tm   ：回零超时时间，单位：毫秒
// //  * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
// //  * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
// //  * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
// //  * @param    potF   ：上电自动触发回零，false为不使能，true为使能
// //  * @retval   地址 + 功能码 + 命令状态 + 校验字节
// //  */
// //void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
// //{
// //  uint8_t cmd[32] = {0};
// //
// //  // 装载命令
// //  cmd[0] =  addr;                       // 地址
// //  cmd[1] =  0x4C;                       // 功能码
// //  cmd[2] =  0xAE;                       // 辅助码
// //  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
// //  cmd[4] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
// //  cmd[5] =  o_dir;                      // 回零方向
// //  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
// //  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节
// //  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
// //  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
// //  cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
// //  cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
// //  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 无限位碰撞回零检测转速(RPM)高8位字节
// //  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 无限位碰撞回零检测转速(RPM)低8位字节
// //  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 无限位碰撞回零检测电流(Ma)高8位字节
// //  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 无限位碰撞回零检测电流(Ma)低8位字节
// //  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 无限位碰撞回零检测时间(Ms)高8位字节
// //  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 无限位碰撞回零检测时间(Ms)低8位字节
// //  cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
// //  cmd[19] =  0x6B;                      // 校验字节
// //
// //  // 发送命令
// //  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 20);
// //}
//
// ///**
// //  * @brief    触发回零
// //  * @param    addr   ：电机地址
// //  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
// //  * @param    snF   ：多机同步标志，false为不启用，true为启用
// //  * @retval   地址 + 功能码 + 命令状态 + 校验字节
// //  */
// void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
// {
//   uint8_t cmd[16] = {0};
//
//   // 装载命令
//   cmd[0] =  addr;                       // 地址
//   cmd[1] =  0x9A;                       // 功能码
//   cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
//   cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
//   cmd[4] =  0x6B;                       // 校验字节
//
//   // 发送命令
//  uart0_send_string((uint8_t *)cmd,5);
// }
//
// ///**
// //  * @brief    强制中断并退出回零
// //  * @param    addr  ：电机地址
// //  * @retval   地址 + 功能码 + 命令状态 + 校验字节
// //  */
// //void Emm_V5_Origin_Interrupt(uint8_t addr)
// //{
// //  uint8_t cmd[16] = {0};
// //
// //  // 装载命令
// //  cmd[0] =  addr;                       // 地址
// //  cmd[1] =  0x9C;                       // 功能码
// //  cmd[2] =  0x48;                       // 辅助码
// //  cmd[3] =  0x6B;                       // 校验字节
// //
// //  // 发送命令
// //  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 5);
// //}
