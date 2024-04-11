#ifndef __CMS_H
#define __CMS_H

#include "struct_typedef.h"

typedef enum
{
	Normal,
	Faster,
	Flying,
	Charging,
	Noforce
}CMSMode_e;


typedef struct
{
//反馈数据
	float cms_cap_v;   		//电容两端电压Ucr
	float chassis_power;  //底盘功率
	uint16_t cms_status;   //电容状态
	
	CMSMode_e cms_mode;//电容模式：充电00
	/*
			bit0	电容过压
			bit1	电容过流
			bit2	电容欠压
			bit3	裁判系统欠压
			bit4	未读到CAN通信数据
	*/
	
//发送数据	
	uint16_t cap_control;  										//10开启双向充放电，00关闭
	int16_t	 output_power_limit;							//限制电容放电功率,-120~300W，整数
	uint16_t input_power_limit;							  //限制电容充电功率,0~150W，整数
  
    /*
			bit0	电容过压
			bit1	电容过流
			bit2	电容欠压
			bit3	裁判系统欠压
			bit4	未读到CAN通信数据
	*/
}CMS_Data_t;

//typedef struct
//{
//	uint8_t Enable;
//	uint8_t RxOpen;
//	uint16_t Power;
//	int16_t Electricity;
//	int16_t charge_limit;//*100
//	uint8_t TxOpen;
//	
//	uint8_t Mode;  //1使用电容
//	
//}CMS_t;

extern CMS_Data_t CMS_Data;


extern int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
extern float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);

extern void CMS_BUFFER_SEND(int16_t buffer);
extern void CMS_POWER_SEND(int16_t input_power_limit,int16_t output_powe_limit,int16_t cap_power_limit,int16_t cap_control);

#endif
