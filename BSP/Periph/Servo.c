// #include "Servo.h"
// #include "misc.h"
// void Set_Servo1_Angle(uint16_t angle1)
// {
//
//
// 			limit_max_min(angle1,Servo1_Max,Servo1_Min);
//
//       if(angle1 > 180)
//       {
//             angle1 = 180;
//       }
//
//
//       float ServoAngle = 50.0f + (((float)angle1 / 180.0f) * 200.0f);
//
//       DL_TimerG_setCaptureCompareValue(Servo_INST, (unsigned int)(ServoAngle + 0.5f), GPIO_Servo_C0_IDX);
// }
// void Set_Servo2_Angle(uint16_t angle2)
// {
//
//
// 			limit_max_min(angle2,Servo2_Max,Servo2_Min);
//
// 			if(angle2 > 180)
//       {
//             angle2 = 180;
//       }
//
//
//       float ServoAngle = 50.0f + (((float)angle2 / 180.0f) * 200.0f);
//
//       DL_TimerG_setCaptureCompareValue(Servo_INST, (unsigned int)(ServoAngle + 0.5f), GPIO_Servo_C1_IDX);
// }