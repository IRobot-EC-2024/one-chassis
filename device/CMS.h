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
//��������
	float cms_cap_v;   		//�������˵�ѹUcr
	float chassis_power;  //���̹���
	uint16_t cms_status;   //����״̬
	
	CMSMode_e cms_mode;//����ģʽ�����00
	/*
			bit0	���ݹ�ѹ
			bit1	���ݹ���
			bit2	����Ƿѹ
			bit3	����ϵͳǷѹ
			bit4	δ����CANͨ������
	*/
	
//��������	
	uint16_t cap_control;  										//10����˫���ŵ磬00�ر�
	int16_t	 output_power_limit;							//���Ƶ��ݷŵ繦��,-120~300W������
	uint16_t input_power_limit;							  //���Ƶ��ݳ�繦��,0~150W������
  
    /*
			bit0	���ݹ�ѹ
			bit1	���ݹ���
			bit2	����Ƿѹ
			bit3	����ϵͳǷѹ
			bit4	δ����CANͨ������
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
//	uint8_t Mode;  //1ʹ�õ���
//	
//}CMS_t;

extern CMS_Data_t CMS_Data;


extern int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
extern float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);

extern void CMS_BUFFER_SEND(int16_t buffer);
extern void CMS_POWER_SEND(int16_t input_power_limit,int16_t output_powe_limit,int16_t cap_power_limit,int16_t cap_control);

#endif