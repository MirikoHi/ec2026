// #include "bsp_motor.h"
// #include "stdio.h"
// #include "bsp_pid.h"
// #include "bsp_key.h"
// #include "string.h"
// #include "trace.h"
// #include "math.h"
//
// uint32_t gpio_interrup;
// int32_t Get_Encoder_countA,Get_Encoder_countB;
// int32_t Get_totalEncoder_countA,Get_totalEncoder_countB;
// pid_type_def pid_Wzleft;
// pid_type_def pid_Wzright;
// pid_type_def pid_Yaw;
// pid_type_def pid_Pitch;
// pid_type_def pid_Pitch_wx;
// pid_type_def pid_Trace;
// Motor motor_left;
// Motor motor_right;
// MotorCtrlMode ctrl_mode;   // 控制模式
//
// //float pid_motorleft[3] = {1000, 10, 0};
// //float pid_motorright[3] = {1000, 10, 0};
// //float pid_wzleft[3] = {16, 0, 3.5};
// //float pid_wzright[3] = {16, 0, 3.5};
// float pid_motorleft[3] = {4500, 20, 0};
// float pid_motorright[3] = {4500, 20, 0};//40：3
// //平衡车
// //float pid_motorleft[3] = {1200, 10, 0};
// //float pid_motorright[3] = {1200, 10, 0};
// float pid_wzleft[3] = {10, 0, 0};
// float pid_wzright[3] = {10, 0, 0};
// float distan_set = 0;
// float pid_yaw[3] = {0.0032f, 0, 0};
// //float pid_trace[3] = {0.02, 0, 0};//红线
// float pid_trace[3] = {0.015,0,0};//2025感为
//
// uint8_t stop_count,spin_count;
// uint8_t Spin_start_flag = 0, Spin_succeed_flag = 0 , Stop_Flag = 0;
// uint8_t Line_flag = 0, Turn_flag = 0 ;
//
// void motor_init(void)
// {
// 	//编码器引脚外部中断
// 	NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
// 	NVIC_EnableIRQ(GPIOB_INT_IRQn);
//   //定时器中断
// 	NVIC_ClearPendingIRQ(TIMG_6_INST_INT_IRQN);
// 	NVIC_EnableIRQ(TIMG_6_INST_INT_IRQN);
// 	NVIC_ClearPendingIRQ(TIMG_7_INST_INT_IRQN);
// 	NVIC_EnableIRQ(TIMG_7_INST_INT_IRQN);
// 	NVIC_ClearPendingIRQ(TIME_MOTOR_STOP_INST_INT_IRQN);
// 	NVIC_EnableIRQ(TIME_MOTOR_STOP_INST_INT_IRQN);
// //	PID_init(&motor_left.motor_pid,PID_POSITION,pid_motorleft,1000,500);
// //	PID_init(&motor_right.motor_pid,PID_POSITION,pid_motorright,1000*,500);
// 	PID_init(&motor_left.motor_pid,PID_POSITION,pid_motorleft,500,250);
// 	PID_init(&motor_right.motor_pid,PID_POSITION,pid_motorright,500,250);
// 	PID_init(&pid_Wzleft,PID_POSITION,pid_wzleft,0.08,0.04);
// 	PID_init(&pid_Wzright,PID_POSITION,pid_wzright,0.08,0.04);
// 	PID_init(&pid_Yaw,PID_POSITION,pid_yaw,1000,500);
// 	PID_init(&pid_Trace,PID_POSITION,pid_trace,1000,500);
//
// 		  // 初始化滤波器
//     motor_left.motor_filter.is_initialized = 0;
//     motor_left.motor_filter.bypass_filter = 0;
//     motor_left.motor_filter.last_ref = 0;
//     motor_left.motor_filter.last_filtered = 0;
//     motor_left.motor_filter.index = 0;
//
//     motor_right.motor_filter.is_initialized = 0;
//     motor_right.motor_filter.bypass_filter = 0;
//     motor_right.motor_filter.last_ref = 0;
//     motor_right.motor_filter.last_filtered = 0;
//     motor_right.motor_filter.index = 0;
// //	motor_left.motor_working_type = MOTOR_STOP;
// //	motor_right.motor_working_type = MOTOR_STOP;
// 	motor_left.motor_working_type = MOTOR_ENALBED;
// 	motor_right.motor_working_type = MOTOR_ENALBED;
// 	motor_left.basie_speed = 0;
// 	motor_right.basie_speed = 0;
//
// }
// // 动态滤波函数（带突变检测）
// // 改进的动态滤波函数
// float calcs_filterd(Motor *motor, float raw_speed) {
//     const float alpha_normal = 0.7f;
//     const float alpha_fast = 0.9f;
//     const float delta_threshold = 0.3f;
//
//     // 计算目标速度变化量
//     float ref_delta = ABS(motor->ref_speed - motor->motor_filter.last_ref);
//
//     // 首次初始化
//     if (!motor->motor_filter.is_initialized) {
//         motor->motor_filter.last_filtered = raw_speed;
//         motor->motor_filter.is_initialized = 1;
//
//         // 初始化缓冲区
//         for(int i=0; i<FILTER_NUMS; i++) {
//             motor->motor_filter.buffer[i] = raw_speed;
//         }
//         return raw_speed;
//     }
//
//     // 更新环形缓冲区
//     motor->motor_filter.buffer[motor->motor_filter.index] = raw_speed;
//     motor->motor_filter.index = (motor->motor_filter.index + 1) % FILTER_NUMS;
//
//     // 计算缓冲区平均值 (用于突变检测)
//     float buffer_avg = 0;
//     for(int i=0; i<FILTER_NUMS; i++) {
//         buffer_avg += motor->motor_filter.buffer[i];
//     }
//     buffer_avg /= FILTER_NUMS;
//
//     // 检测突变：当前值是否偏离平均值超过阈值
//     if (ABS(raw_speed - buffer_avg) > delta_threshold) {
//         // 突变期：使用轻滤波
//         float filtered = 0.9f * raw_speed + 0.1f * motor->motor_filter.last_filtered;
//         motor->motor_filter.last_filtered = filtered;
//         return filtered;
//     }
//
//     // 正常期：使用动态EWMA滤波
//     float alpha = (ref_delta > delta_threshold) ? alpha_fast : alpha_normal;
//     float filtered = alpha * raw_speed + (1 - alpha) * motor->motor_filter.last_filtered;
//     motor->motor_filter.last_filtered = filtered;
//
//     return filtered;
// }
//
// //左右电机速度控制，范围0-1000
// void Set_SpeedRight(int pwma)
// {
// 	if(pwma>0) //左轮前进
// 	{
// 		DL_GPIO_setPins(MOTOR_AIN1_A02_PORT,MOTOR_AIN1_A02_PIN);
// 		DL_GPIO_clearPins(MOTOR_AIN2_B24_PORT,MOTOR_AIN2_B24_PIN);
// 	}
// 	else if(pwma < 0)//左轮后退
// 	{
// 		DL_GPIO_setPins(MOTOR_AIN2_B24_PORT,MOTOR_AIN2_B24_PIN);
// 		DL_GPIO_clearPins(MOTOR_AIN1_A02_PORT,MOTOR_AIN1_A02_PIN);
// 	}
// 	else
// 	{
// 		DL_GPIO_setPins(MOTOR_AIN1_A02_PORT,MOTOR_AIN1_A02_PIN);
// 		DL_GPIO_setPins(MOTOR_AIN2_B24_PORT,MOTOR_AIN2_B24_PIN);
// 	}
// 	pwma = ABS(pwma);
// 	if(pwma >= 1000)pwma = 1000;
// 	DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,pwma,GPIO_PWM_MOTOR_C0_IDX);
// }
// void Set_SpeedLeft(int pwmb)
// {
// 	if(pwmb>0)//右轮前进
// 	{
// 		DL_GPIO_setPins(MOTOR_BIN1_B20_PORT,MOTOR_BIN1_B20_PIN);
// 		DL_GPIO_clearPins(MOTOR_BIN2_B19_PORT,MOTOR_BIN2_B19_PIN);
// 	}
//   else if(pwmb < 0)//右轮后退
// 	{
// 		DL_GPIO_setPins(MOTOR_BIN2_B19_PORT,MOTOR_BIN2_B19_PIN);
// 		DL_GPIO_clearPins(MOTOR_BIN1_B20_PORT,MOTOR_BIN1_B20_PIN);
// 	}
// 	else
// 	{
// 		DL_GPIO_setPins(MOTOR_BIN2_B19_PORT,MOTOR_BIN2_B19_PIN);
// 		DL_GPIO_setPins(MOTOR_BIN1_B20_PORT,MOTOR_BIN1_B20_PIN);
// 	}
// 	pwmb = ABS(pwmb);
// 	if(pwmb >= 1000)pwmb = 1000;
// 	DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,ABS(pwmb),GPIO_PWM_MOTOR_C1_IDX);
// }
// /*******************************************************
// 函数功能：外部中断模拟编码器信号
// 入口函数：无
// 返回  值：无
// ***********************************************************/
// void GROUP1_IRQHandler(void)
// {
// 	//获取中断信号
// 	gpio_interrup = DL_GPIO_getEnabledInterruptStatus(GPIOB,ENCODER_E1A_PIN|ENCODER_E1B_PIN|ENCODER_E2A_PIN|ENCODER_E2B_PIN);
// 	//encoderA
// 	if((gpio_interrup & ENCODER_E1A_PIN)==ENCODER_E1A_PIN)
// 	{
// 		if(!DL_GPIO_readPins(GPIOB,ENCODER_E1B_PIN))
// 		{
// 			Get_Encoder_countA--;
// 		}
// 		else
// 		{
// 			Get_Encoder_countA++;
// 		}
// 	}
// 	else if((gpio_interrup & ENCODER_E1B_PIN)==ENCODER_E1B_PIN)
// 	{
// 		if(!DL_GPIO_readPins(GPIOB,ENCODER_E1A_PIN))
// 		{
// 			Get_Encoder_countA++;
// 		}
// 		else
// 		{
// 			Get_Encoder_countA--;
// 		}
// 	}
// 	//encoderB
// 	if((gpio_interrup & ENCODER_E2A_PIN)==ENCODER_E2A_PIN)
// 	{
// 		if(!DL_GPIO_readPins(GPIOB,ENCODER_E2B_PIN))
// 		{
// 			Get_Encoder_countB--;
// 		}
// 		else
// 		{
// 			Get_Encoder_countB++;
// 		}
// 	}
// 	else if((gpio_interrup & ENCODER_E2B_PIN)==ENCODER_E2B_PIN)
// 	{
// 		if(!DL_GPIO_readPins(GPIOB,ENCODER_E2A_PIN))
// 		{
// 			Get_Encoder_countB++;
// 		}
// 		else
// 		{
// 			Get_Encoder_countB--;
// 		}
// 	}
// 	DL_GPIO_clearInterruptStatus(GPIOB,ENCODER_E1A_PIN|ENCODER_E1B_PIN|ENCODER_E2A_PIN|ENCODER_E2B_PIN);
// }
//
// void Encode_Updata(void)
// {
// 		motor_left.Encode = -Get_Encoder_countA;//两个电机安装相反，所以编码器值也要相反
// 		motor_right.Encode = Get_Encoder_countB;
// 		motor_left.total_Encode = motor_left.total_Encode-Get_Encoder_countA;
// 		motor_right.total_Encode = motor_right.total_Encode + Get_Encoder_countB;
// 		Get_Encoder_countA = 0;//编码器计数值清零
// 		Get_Encoder_countB = 0;
// 		motor_left.raw_speed = motor_left.Encode * ENCODER_TO_SPEED_MS;
// 		motor_right.raw_speed = motor_right.Encode * ENCODER_TO_SPEED_MS;
// 		motor_left.real_distance = motor_left.total_Encode * ENCODER_TO_DISDAN_M;
// 		motor_right.real_distance = motor_right.total_Encode * ENCODER_TO_DISDAN_M;
// }
// void Set_Motor_Ref_Speed(float left_speed, float right_speed)
// {
//     motor_left.basie_speed = left_speed;
//     motor_right.basie_speed = right_speed;
// }
// void Motor_Task(void)
// {
// 	if(clear_flag)
// 	{
// 		PID_clear(&pid_Wzright);
// 		PID_clear(&pid_Wzleft);
// 		PID_clear(&motor_left.motor_pid);
// 		PID_clear(&motor_right.motor_pid);
// 		PID_clear(&pid_Wzright);
// 		PID_clear(&pid_Wzright);
//
// 		clear_flag = 0;
// 	}
// //	motor_left.real_distance = 0;
// //	motor_right.real_distance = 0;
// 	static float outputleftW,outputrightW,outputleftN,outputrightN,outyaw,trace_out,output_pitch;
// //	if(ctrl_mode == MOTOR_CTRL_YAW)outyaw = PID_calc(&pid_Yaw,imu_data.totalYaw,Yaw_ref,NULL);
// //	else outyaw=0;
//
//
// 	float left1=motor_left.ref_position;
// 	float left2=motor_right.ref_position;
//
// 	if(ctrl_mode == MOTOR_CTRL_POS_VEL_TRACE)
// 	{
// 		outputleftW = PID_calc(&pid_Wzleft, motor_left.real_distance,left1);
// 		outputrightW = PID_calc(&pid_Wzright, motor_right.real_distance,left2);
// 		trace_out = PID_calc(&pid_Trace,track_err,0);
// 	}
// 	else if(ctrl_mode == MOTOR_CTRL_POS_VEL)
// 	{
// 			outputleftW = PID_calc(&pid_Wzleft, motor_left.real_distance,left1);
// 			outputrightW = PID_calc(&pid_Wzright, motor_right.real_distance,left2);
// 			trace_out = 0;
// 	}
// 	if(ctrl_mode == MOTOR_CTRL_VELOCITY)
// 	{
// 			outputleftW = 0;
// 			outputrightW = 0;
// 	}
// 	if(ctrl_mode == MOTOR_CTRL_TRACK)
// 	{
// 		trace_out = PID_calc(&pid_Trace,track_err,0);
// 		trace_out = trace_out * 1.2;
// 	}
// 	motor_left.real_speed = calcs_filterd(&motor_left,motor_left.raw_speed);
// 	motor_right.real_speed = calcs_filterd(&motor_right,motor_right.raw_speed);
//
//
// 	motor_left.ref_speed = motor_left.basie_speed +  outputleftW - outyaw - trace_out;
// 	motor_right.ref_speed = motor_right.basie_speed + outputrightW + outyaw + trace_out;
//
//
// 	outputleftN = PID_calc(&motor_left.motor_pid, motor_left.real_speed,motor_left.ref_speed);
// 	outputrightN = PID_calc(&motor_right.motor_pid, motor_right.real_speed,motor_right.ref_speed);
//
//
// 	if(ctrl_mode == MOTOR_CTRL_STOP)
// 	{
// 			outputleftN =0;
// 			outputrightN = 0;
// 	}
//     // 3. 电机控制
// 	switch(motor_left.motor_working_type)
// 	{
// 		case MOTOR_STOP:
// 				Set_SpeedLeft(0);
// 				break;
// 		case MOTOR_ENALBED:
// 				Set_SpeedLeft((int)outputleftN);
// 				break;
// 		default:
// 				break;
// 	}
// 	switch(motor_right.motor_working_type)
// 	{
// 		case MOTOR_STOP:
// 				Set_SpeedRight(0);
// 				break;
// 		case MOTOR_ENALBED:
// 				Set_SpeedRight((int)outputrightN);
// 				break;
// 		default:
// 				break;
// 	}
// }
// //电机编码器脉冲计数
// void TIMG7_IRQHandler(void)
// {
// 	if( DL_TimerG_getPendingInterrupt(TIMG_7_INST) == DL_TIMER_IIDX_ZERO )
// 	{
// 		Encode_Updata();
//
// 	}
// }
//
// void Motor_set_position(float left,float right)
// {
// 		Line_flag = 1;
// 		Stop_Flag = 0;
// 		Spin_start_flag = 0;
// 		Spin_succeed_flag = 0;
//
// 		motor_left.motor_pid.max_out = 0.1;
// 		motor_left.motor_pid.max_iout = 0.1;
// 		ctrl_mode = MOTOR_CTRL_POS_VEL_TRACE;
// 		motor_left.total_Encode = 0;
// 		motor_right.total_Encode = 0;
// 		motor_left.ref_position = left;
// 		motor_right.ref_position = right;
// }
//
// void Motor_Set_Turn(spin_dir_t mode)
// {
// 	Line_flag = 0;  //不进行巡线的补偿了
// 	Stop_Flag = 0;   //执行转弯时，将直走完成的标志位清零. 即如果上一次是直行，
// 	Spin_start_flag = 1;
// 	Spin_succeed_flag = 0;
// //	ctrl_mode = MOTOR_CTRL_POS_VEL;
// 	ctrl_mode = MOTOR_CTRL_POS_VEL_TRACE;
// 	motor_left.total_Encode = 0;
// 	motor_right.total_Encode = 0;
// 	motor_left.motor_pid.max_out = 0.05;
// 	motor_left.motor_pid.max_iout = 0.05;
// 	if(mode == right_90)
// 	{
// 		motor_left.ref_position = 0.005;
// 		motor_right.ref_position = -0.005;
// 	}
// 	else if(mode == left_90)
// 	{
// 		motor_left.ref_position = -0.005;
// 		motor_right.ref_position = 0.005;
// 	}
// 	else if( mode == back_180)
// 	{
// 		motor_left.ref_position = 0.02;
// 		motor_right.ref_position = -0.02;
// 	}
// }
// //TurnParams CalculateTurnParams(spin_dir_t mode) {
// //    // 常量定义
// //    const float W = 0.077f;     // 轮距77mm
// //    const float L = 0.130f;     // 轴距130mm
// //    const float MAX_STEER = 30.0f; // 最大舵机转角30°
// //
// //    TurnParams params;
// //
// //    // 计算最小转弯半径
// //    float R_min = L / tanf(MAX_STEER * PI/180.0f); // ≈0.225m
// //
// //    // 转弯起始点（距离顶点）
// //    params.start_distance = R_min; // 0.225m
// //
// //    // 计算转弯弧长（90度）
// //    float arc_length = R_min * PI/2; // ≈0.353m
// //
// //    // 根据转向方向设置参数
// //    if(mode == right_90) {
// //        params.steer_angle = 90.0f - MAX_STEER; // 60°
// //        params.left_distance = arc_length * (1 + W/(2*R_min));  // 外轮
// //        params.right_distance = arc_length * (1 - W/(2*R_min)); // 内轮
// //    }
// //    else if(mode == left_90) {
// //        params.steer_angle = 90.0f + MAX_STEER; // 120°
// //        params.left_distance = arc_length * (1 - W/(2*R_min));  // 内轮
// //        params.right_distance = arc_length * (1 + W/(2*R_min)); // 外轮
// //    }
// //    else if(mode == back_180) {
// //        params.steer_angle = 90.0f + MAX_STEER; // 120°
// //        // 180度转弯弧长翻倍
// //        params.left_distance = 2 * arc_length * (1 - W/(2*R_min));
// //        params.right_distance = 2 * arc_length * (1 + W/(2*R_min));
// //    }
// //
// //    return params;
// //}
//
// // 主转弯函数
// //void Motor_Set_Turn(spin_dir_t mode)
// //{
// //    // 获取转弯参数
// //    TurnParams turn = CalculateTurnParams(mode);
// //
// //    Line_flag = 0;
// //    Stop_Flag = 0;
// //    Spin_start_flag = 1;
// //    Spin_succeed_flag = 0;
// //    ctrl_mode = MOTOR_CTRL_POS_VEL;
// //    motor_left.total_Encode = 0;
// //    motor_right.total_Encode = 0;
//
// //    if(mode == right_90)
// //    {
// //        Set_Servo1_Angle(turn.steer_angle);  // 60°
// //        motor_left.ref_position = turn.left_distance;   // 外轮
// //        motor_right.ref_position = turn.right_distance; // 内轮
// //    }
// //    else if(mode == left_90)
// //    {
// //        Set_Servo1_Angle(turn.steer_angle);  // 120°
// //        motor_left.ref_position = turn.left_distance*0.1;   // 内轮
// //        motor_right.ref_position = turn.right_distance*0.1; // 外轮
// //    }
// //    else if(mode == back_180)
// //    {
// //        Set_Servo1_Angle(turn.steer_angle);  // 120°
// //        motor_left.ref_position = turn.left_distance;
// //        motor_right.ref_position = turn.right_distance;
// //    }
// //}
// void TIMA0_IRQHandler(void)
// {
// 	if( DL_TimerG_getPendingInterrupt(TIME_MOTOR_STOP_INST) == DL_TIMER_IIDX_ZERO )
// 	{
// 		if(Line_flag)
// 		{
// 				if((ABS(motor_left.ref_position - motor_left.real_distance) < 0.008) && (ABS(motor_right.ref_position - motor_right.real_distance) < 0.008))
// 				{
// 						stop_count++;
// 						if(stop_count >= 20)
// 						{
// 								Line_flag = 0;
// 								Stop_Flag = 1; //这个标志位可以用来判断是否执行下一阶段任务
// 								stop_count = 0;
// 								ctrl_mode = MOTOR_CTRL_STOP;
// 						}
// 				}
// 				else
// 				{
// 						Stop_Flag = 0;
// 						stop_count = 0;
// 				}
// 		}
// 		if(Spin_start_flag)
// 		{
// 			spin_count++;
// 			if(spin_count >= 20 && (ABS(motor_left.ref_position - motor_left.real_distance) < 0.008) && (ABS(motor_right.ref_position - motor_right.real_distance) < 0.008))
// 			{
// 					Spin_start_flag = 0;
// 					spin_count = 0;
// 					ctrl_mode = MOTOR_CTRL_STOP;
// 					Spin_succeed_flag = 1;
// 			}
// 		}
// 	}
// }
//
