// #include "PAW3395.h"
// #include "dwt.h"
// volatile int16_t delta_x = 0;
// volatile int16_t delta_y = 0;
// volatile uint8_t motion_burst_data[12] = {0};
// float X_axis = 0,Y_axis = 0;
//
// static uint8_t SPI_SendReceive(uint8_t dat)
// {
// 	uint8_t data = 0;
//
//    //发送数据
// 	while(DL_SPI_isBusy(SPI_0_INST));
//         DL_SPI_transmitData8(SPI_0_INST,dat);
//         //等待SPI总线空闲
//
//         //接收数据
// 	while(DL_SPI_isBusy(SPI_0_INST));
//         data = DL_SPI_receiveData8(SPI_0_INST);
//         //等待SPI总线空闲
//
//
//         return data;
//
// }
// static void PAW3395_Write_Register(uint8_t reg,uint8_t dat)
// {
// 	SPI_CS(0);
// 	DWT_Delay(0.000000125);
//   //发送数据
//   SPI_SendReceive(reg+(0x80));
//
//   //发送数据
//   SPI_SendReceive(dat);
//   SPI_CS(1);
//   DWT_Delay(0.000005);
// }
//
// static uint8_t PAW3395_Read_Register(uint8_t reg)
// {
// 	uint8_t data = 0;
// 	SPI_CS(0);
// 	DWT_Delay(0.000000125);
//   //发送数据
//   SPI_SendReceive(reg);
//   DWT_Delay(0.000005);
//   //接收数据
//   data = SPI_SendReceive(0xff);
// // SPI_CS(1);
//   return data;
// }
//
// void PAW3395_Init(void)
// {
// 	DWT_Delay(0.05);
// 	SPI_CS(0);
// 	DWT_Delay(0.000000125);
// 	SPI_CS(1);
// 	DWT_Delay(0.000000125);
// 	SPI_CS(0);
// 	DWT_Delay(0.000000125);
// 	PAW3395_Write_Register(0x3A,0x5A);
// 	DWT_Delay(0.005);
//
//
// 		PAW3395_Write_Register(0x7F, 0x07);
//     PAW3395_Write_Register(0x40, 0x41);
//     PAW3395_Write_Register(0x7F, 0x00);
//     PAW3395_Write_Register(0x40, 0x80);
//     PAW3395_Write_Register(0x7F, 0x0E);
//     PAW3395_Write_Register(0x55, 0x0D);
//     PAW3395_Write_Register(0x56, 0x1B);
//     PAW3395_Write_Register(0x57, 0x08);
//     PAW3395_Write_Register(0x58, 0xD5);
//     PAW3395_Write_Register(0x7F, 0x14);
//     PAW3395_Write_Register(0x42, 0xBC);
//     PAW3395_Write_Register(0x43, 0x74);
//     PAW3395_Write_Register(0x4B, 0x20);
//     PAW3395_Write_Register(0x4D, 0x00);
//     PAW3395_Write_Register(0x53, 0x0E);
//     PAW3395_Write_Register(0x7F, 0x05);
//     PAW3395_Write_Register(0x43, 0x64);
//     PAW3395_Write_Register(0x44, 0x04);
//     PAW3395_Write_Register(0x4D, 0x06);
//     PAW3395_Write_Register(0x51, 0x40);
//     PAW3395_Write_Register(0x53, 0x40);
//     PAW3395_Write_Register(0x55, 0xCA);
//     PAW3395_Write_Register(0x5A, 0xE8);
//     PAW3395_Write_Register(0x5B, 0xEA);
//     PAW3395_Write_Register(0x61, 0x31);
//     PAW3395_Write_Register(0x62, 0x64);
//     PAW3395_Write_Register(0x6D, 0xB8);
//     PAW3395_Write_Register(0x6E, 0x0F);
//     PAW3395_Write_Register(0x70, 0x02);
//     PAW3395_Write_Register(0x4A, 0x2A);
//     PAW3395_Write_Register(0x60, 0x26);
//     PAW3395_Write_Register(0x7F, 0x06);
//     PAW3395_Write_Register(0x6D, 0x70);
//     PAW3395_Write_Register(0x6E, 0x60);
//     PAW3395_Write_Register(0x6F, 0x04);
//     PAW3395_Write_Register(0x53, 0x02);
//     PAW3395_Write_Register(0x55, 0x11);
//     PAW3395_Write_Register(0x7A, 0x01);
//     PAW3395_Write_Register(0x7D, 0x51);
//     PAW3395_Write_Register(0x7F, 0x07);
//     PAW3395_Write_Register(0x41, 0x10);
//     PAW3395_Write_Register(0x42, 0x32);
//     PAW3395_Write_Register(0x43, 0x00);
//     PAW3395_Write_Register(0x7F, 0x08);
//     PAW3395_Write_Register(0x71, 0x4F);
//     PAW3395_Write_Register(0x7F, 0x09);
//     PAW3395_Write_Register(0x62, 0x1F);
//     PAW3395_Write_Register(0x63, 0x1F);
//     PAW3395_Write_Register(0x65, 0x03);
//     PAW3395_Write_Register(0x66, 0x03);
//     PAW3395_Write_Register(0x67, 0x1F);
//     PAW3395_Write_Register(0x68, 0x1F);  // 第52条
//
//     // >>>>>>>> 图片2序列 (53-136) <<<<<<<<
//     PAW3395_Write_Register(0x69, 0x03);  // 第53条
//     PAW3395_Write_Register(0x6A, 0x03);  // 54
//     PAW3395_Write_Register(0x6C, 0x1F);  // 55
//     PAW3395_Write_Register(0x6D, 0x1F);  // 56
//     PAW3395_Write_Register(0x51, 0x04);  // 57
//     PAW3395_Write_Register(0x53, 0x20);  // 58
//     PAW3395_Write_Register(0x54, 0x20);  // 59
//     PAW3395_Write_Register(0x71, 0x0C);  // 60
//     PAW3395_Write_Register(0x72, 0x07);  // 61
//     PAW3395_Write_Register(0x73, 0x07);  // 62
//     PAW3395_Write_Register(0x7F, 0x0A);  // 63
//     PAW3395_Write_Register(0x4A, 0x14);  // 64
//     PAW3395_Write_Register(0x4C, 0x14);  // 65
//     PAW3395_Write_Register(0x55, 0x19);  // 66
//     PAW3395_Write_Register(0x7F, 0x14);  // 67
//     PAW3395_Write_Register(0x4B, 0x30);  // 68
//     PAW3395_Write_Register(0x4C, 0x03);  // 69
//     PAW3395_Write_Register(0x61, 0x0B);  // 70
//     PAW3395_Write_Register(0x62, 0x0A);  // 71
//     PAW3395_Write_Register(0x63, 0x02);  // 72
//     PAW3395_Write_Register(0x7F, 0x15);  // 73
//     PAW3395_Write_Register(0x4C, 0x02);  // 74
//     PAW3395_Write_Register(0x56, 0x02);  // 75
//     PAW3395_Write_Register(0x41, 0x91);  // 76
//     PAW3395_Write_Register(0x4D, 0x0A);  // 77
//     PAW3395_Write_Register(0x7F, 0x0C);  // 78
//     PAW3395_Write_Register(0x4A, 0x10);  // 79
//     PAW3395_Write_Register(0x4B, 0x0C);  // 80
//     PAW3395_Write_Register(0x4C, 0x40);  // 81
//     PAW3395_Write_Register(0x41, 0x25);  // 82
//     PAW3395_Write_Register(0x55, 0x18);  // 83
//     PAW3395_Write_Register(0x56, 0x14);  // 84
//     PAW3395_Write_Register(0x49, 0x0A);  // 85
//     PAW3395_Write_Register(0x42, 0x00);  // 86
//     PAW3395_Write_Register(0x43, 0x2D);  // 87
//     PAW3395_Write_Register(0x44, 0x0C);  // 88
//     PAW3395_Write_Register(0x54, 0x1A);  // 89
//     PAW3395_Write_Register(0x5A, 0x0D);  // 90
//     PAW3395_Write_Register(0x5F, 0x1E);  // 91
//     PAW3395_Write_Register(0x5B, 0x05);  // 92
//     PAW3395_Write_Register(0x5E, 0x0F);  // 93
//     PAW3395_Write_Register(0x61, 0x22);  // 94
//     PAW3395_Write_Register(0x62, 0x41);  // 95
//     PAW3395_Write_Register(0x7F, 0x0D);  // 96
//     PAW3395_Write_Register(0x48, 0xDD);  // 97
//     PAW3395_Write_Register(0x4F, 0x03);  // 98
//     PAW3395_Write_Register(0x52, 0x49);  // 99
//     PAW3395_Write_Register(0x51, 0x00);  // 100
//     PAW3395_Write_Register(0x54, 0x5B);  // 101
//     PAW3395_Write_Register(0x53, 0x00);  // 102
//     PAW3395_Write_Register(0x56, 0x64);  // 103
//     PAW3395_Write_Register(0x55, 0x00);  // 104
//     PAW3395_Write_Register(0x58, 0xA5);  // 105
//     PAW3395_Write_Register(0x57, 0x02);  // 106
//     PAW3395_Write_Register(0x5A, 0x29);  // 107
//     PAW3395_Write_Register(0x5B, 0x47);  // 108
//     PAW3395_Write_Register(0x5C, 0x81);  // 109
//     PAW3395_Write_Register(0x5D, 0x40);  // 110
//     PAW3395_Write_Register(0x71, 0xDC);  // 111
//     PAW3395_Write_Register(0x70, 0x07);  // 112
//     PAW3395_Write_Register(0x73, 0x00);  // 113
//     PAW3395_Write_Register(0x72, 0x08);  // 114
//     PAW3395_Write_Register(0x75, 0xDC);  // 115
//     PAW3395_Write_Register(0x74, 0x07);  // 116
//     PAW3395_Write_Register(0x77, 0x00);  // 117
//     PAW3395_Write_Register(0x76, 0x08);  // 118
//     PAW3395_Write_Register(0x7F, 0x10);  // 119
//     PAW3395_Write_Register(0x4C, 0xD0);  // 120
//     PAW3395_Write_Register(0x7F, 0x00);  // 121
//     PAW3395_Write_Register(0x4F, 0x63);  // 122
//     PAW3395_Write_Register(0x4E, 0x00);  // 123
//     PAW3395_Write_Register(0x52, 0x63);  // 124
//     PAW3395_Write_Register(0x51, 0x00);  // 125
//     PAW3395_Write_Register(0x54, 0x54);  // 126
//     PAW3395_Write_Register(0x5A, 0x10);  // 127
//     PAW3395_Write_Register(0x77, 0x4F);  // 128
//     PAW3395_Write_Register(0x47, 0x01);  // 129
//     PAW3395_Write_Register(0x5B, 0x40);  // 130
//     PAW3395_Write_Register(0x64, 0x60);  // 131
//     PAW3395_Write_Register(0x65, 0x06);  // 132
//     PAW3395_Write_Register(0x66, 0x13);  // 133
//     PAW3395_Write_Register(0x67, 0x0F);  // 134
//     PAW3395_Write_Register(0x78, 0x01);  // 135
//     PAW3395_Write_Register(0x79, 0x9C);  // 136
//
//     // >>>>>>>> 图片3序列 (137-148) <<<<<<<<
//     PAW3395_Write_Register(0x40, 0x00);  // 137
//     PAW3395_Write_Register(0x55, 0x02);  // 138
//     PAW3395_Write_Register(0x23, 0x70);  // 139
//     PAW3395_Write_Register(0x22, 0x01);  // 140
//
//     DWT_Delay(0.001);                 // 141: 精确延时1ms
//
//     uint8_t read_times = 0;
//     while(PAW3395_Read_Register(0x6C) != 0x80)  // 142
// 		{
// 			read_times++;
// 			 DWT_Delay(0.001);                 // 141: 精确延时1ms
// 			if(read_times >= 60) break;
// 		}
// 		if(read_times >= 60)
// 		{
// 				PAW3395_Write_Register(0x7F, 0x14);  // a
// 				PAW3395_Write_Register(0x6C, 0x00);  // b
// 				PAW3395_Write_Register(0x7F, 0x00);  // c
//
// 		}
//     PAW3395_Write_Register(0x22, 0x00);  // 143
//     PAW3395_Write_Register(0x55, 0x00);  // 144
//     PAW3395_Write_Register(0x7F, 0x07);  // 145
//     PAW3395_Write_Register(0x40, 0x40);  // 146
//     PAW3395_Write_Register(0x7F, 0x00);  // 147
// 		PAW3395_Write_Register(0x68, 0x01);  // 148
//
// 		SPI_CS(1);
// 		DWT_Delay(0.000000125);
//
// 		PAW3395_Read_Register(0x02);
// 		DWT_Delay(0.000000125);
// 		PAW3395_Read_Register(0x03);
// 		DWT_Delay(0.000000125);
// 		PAW3395_Read_Register(0x04);
// 		DWT_Delay(0.000000125);
// 		PAW3395_Read_Register(0x05);
// 		DWT_Delay(0.000000125);
// 		PAW3395_Read_Register(0x06);
// 		DWT_Delay(0.000000125);
//
// 		SPI_CS(1);
// 		DWT_Delay(0.000000125);
// }
// void PAW3395_Read_Motion(float* axis)
// {
// 	//Lower NCS
// 	SPI_CS(0);
// 	//Wait for t(NCS-SCLK)
// 	DWT_Delay(0.000000125);
// 	//Send Motion_Brust address(0x16)
// 	SPI_SendReceive(0x16);	//读
// 	//Wait for tSRAD
// 	DWT_Delay(0.000002);
// 	//Start reading SPI data continuously up to 12 bytes.
// 	for(uint8_t i = 0;i < 12;i++)
// 	{
// 		motion_burst_data[i] = SPI_SendReceive(0x00);
// 	}
// 	SPI_CS(1);
// 	DWT_Delay(0.000000500);
// 	delta_x = (int16_t)(motion_burst_data[2] + (motion_burst_data[3] << 8));
// 	delta_y = (int16_t)(motion_burst_data[4] + (motion_burst_data[5] << 8));
// 	axis[0] = (float)delta_x/5000*INCH_TO_METER;
// 	axis[1] = (float)delta_y/5000*INCH_TO_METER;
//
//
// }
//
//
