#include "CalculateThread.h"
#include "feet_motor.h"
#include "Remote.h"
#include "AttitudeThread.h"
#include "cmsis_os.h"
#include "pid.h"
#include "Setting.h"
#include "user_lib.h"
#include "CanPacket.h"
#include "stdio.h"
#include "InterruptService.h"
#include "RefereeBehaviour.h"
#include "usart.h"
#include "MahonyAHRS.h"
#include "arm_math.h"
#include "CMS.h"
#include "bsp_can.h"
#define START_POWER 8.0f


uint16_t count_down=0;
Chassis_t Chassis;
RC_ctrl_t Remote;
EulerSystemMeasure_t Imu;
Aim_t Aim;
PTZ_t PTZ;
ext_game_robot_status_t Referee;
extern ext_power_heat_data_t power_heat_data_t;
uint32_t F_Motor[8];

fp32 kw,wz;
fp32 roting_speed;
fp32 Angle_zero_6020[4] = {29.3810272, -121.633499, 121.76535, -132.709076};
//fp32 Angle_zero_6020[4] = {0, 0, 0, 0};
fp32 Direction[5] = {-1.0, -1.0, 1.0, 1.0, -1.0};
fp32 Maxspeed = 6000.0f;
fp32 speed[4];
fp32 angle[4];
KFP Power_kf;
fp32 chassis_max_power = 45.0f;
float angle_minus;
float run_per;
fp32 v_gain=0;

uint8_t Mode_last;
uint8_t Mode_now;

//power control
float last_speed[8] = {0};
fp32 forecast_power = 0;
const float kp = 1.3 * 9.99999999e-07;
float lijupower = 0.0f;
uint8_t power_flag=0;
float power_scale;

//pid_type_def follow_yaw;
pid_type_def follow;
pid_type_def left_front_6020_speed_pid;
pid_type_def right_front_6020_speed_pid;
pid_type_def right_back_6020_speed_pid;
pid_type_def left_back_6020_speed_pid;
pid_type_def left_front_6020_position_pid;
pid_type_def right_front_6020_position_pid;
pid_type_def right_back_6020_position_pid;
pid_type_def left_back_6020_position_pid;
pid_type_def left_front_3508_pid;
pid_type_def right_front_3508_pid;
pid_type_def right_back_3508_pid;
pid_type_def left_back_3508_pid;
pid_type_def power_control_pid;
//first_order_filter_type_t current_6020_filter_type;
//first_order_filter_type_t current_3508_filter_type;
//first_order_filter_type_t referee_power;
//first_order_filter_type_t wheel_angle_1;
//first_order_filter_type_t wheel_angle_2;
//first_order_filter_type_t wheel_angle_3;
//first_order_filter_type_t wheel_angle_4;
//first_order_filter_type_t wz_filter;



fp32 follow_angle;
fp32 follow_yaw_PID[3]={0.08,0,1};
fp32 follow_PID[3]={FOLLOW_KP,FOLLOW_KI,FOLLOW_KD};
fp32 left_front_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 right_front_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 right_back_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 left_back_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 left_front_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 right_front_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 right_back_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 left_back_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 left_front_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 right_front_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 right_back_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 left_back_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 power_control_PID[3] = {power_control_KP, power_control_KI, power_control_KD};

extern motor_measure_t LEFT_FRONT_6020_Measure;
extern motor_measure_t RIGHT_FRONT_6020_Measure;
extern motor_measure_t RIGHT_BACK_6020_Measure;
extern motor_measure_t LEFT_BACK_6020_Measure;
extern motor_measure_t LEFT_FRONT_3508_Measure;
extern motor_measure_t RIGHT_FRONT_3508_Measure;
extern motor_measure_t RIGHT_BACK_3508_Measure;
extern motor_measure_t LEFT_BACK_3508_Measure;
extern motor_measure_t YawMotorMeasure;
OfflineMonitor_t Offline;

void ChassisInit();
void ChassisModeUpdate();
void ChassisPidUpadte();
void ChassisCommandUpdate();
void ChassisCurrentUpdate();
void RefereeInfUpdate(ext_game_robot_status_t *referee);
void ChassisInfUpdate();
void Angle_Speed_calc();
void PowerControl();
void CMS__();
void chassis_power_control(Chassis_t *Chassis);
uint8_t chassis_powerloop(Chassis_t *Chassis);
void chassis_limit_update();


void CalculateThread(void const *pvParameters)
{

	ChassisInit();

	while (1)
	{
		//Remote = *get_remote_control_point();
		
		DeviceOfflineMonitorUpdate(&Offline);
		CMS__();
		ChassisModeUpdate();
		ChassisInfUpdate();
		//chassis_limit_update();
		//RefereeInfUpdate(&Referee);
		//GimbalEulerSystemMeasureUpdate(&Imu);
		ChassisCommandUpdate();
		//chassis_powerloop(&Chassis);
		chassis_power_control(&Chassis);
		Chassis_Control(Chassis.Current_6020[0],
										Chassis.Current_6020[1],
										Chassis.Current_6020[2],
										Chassis.Current_6020[3],
										Chassis.Current_3508[0],
										Chassis.Current_3508[1],
										Chassis.Current_3508[2],
										Chassis.Current_3508[3]);

		
	
		osDelay(1);
	}
}

void ChassisInit()
{
	PID_init(&left_front_6020_speed_pid, PID_POSITION, left_front_6020_speed_PID, 16000, 5000);
	PID_init(&right_front_6020_speed_pid, PID_POSITION, right_front_6020_speed_PID, 16000, 5000);
	PID_init(&right_back_6020_speed_pid, PID_POSITION, right_back_6020_speed_PID, 16000, 5000);
	PID_init(&left_back_6020_speed_pid, PID_POSITION, left_back_6020_speed_PID, 16000, 5000);
	PID_init(&left_front_6020_position_pid, PID_POSITION, left_front_6020_position_PID, 300, 60);              //6020
	PID_init(&right_front_6020_position_pid, PID_POSITION, right_front_6020_position_PID, 300, 60);
	PID_init(&right_back_6020_position_pid, PID_POSITION, right_back_6020_position_PID, 300, 60);
	PID_init(&left_back_6020_position_pid, PID_POSITION, left_back_6020_position_PID, 300, 60);

	PID_init(&left_front_3508_pid, PID_POSITION, left_front_3508_PID, 16384, 1000);
	PID_init(&right_front_3508_pid, PID_POSITION, right_front_3508_PID, 16384, 1000);
	PID_init(&right_back_3508_pid, PID_POSITION, right_back_3508_PID, 16384, 1000);							//3508
	PID_init(&left_back_3508_pid, PID_POSITION, left_back_3508_PID, 16384, 1000);
	
	//PID_init(&follow_yaw,PID_POSITION,follow_yaw_PID,1,1);
	PID_init(&follow,PID_POSITION,follow_PID,5,2);//?
	
	//KalmanFilter_init(&Power_kf, 0.0f , 0.0001f,0.0118f ,0.0,50.0,2.0);//A,B,P,Q,R                   //功率
//	first_order_filter_init(&current_6020_filter_type,0.002,0.1);
//	first_order_filter_init(&current_3508_filter_type,0.002,0.1);
	PID_init(&power_control_pid,PID_POSITION,power_control_PID,2.0f,0.5f);
//	first_order_filter_init(&referee_power,0.001,0.1);
//	first_order_filter_init(&wheel_angle_1,0.001,0.1);
//	first_order_filter_init(&wheel_angle_2,0.001,0.1);
//	first_order_filter_init(&wheel_angle_3,0.001,0.1);
//	first_order_filter_init(&wheel_angle_4,0.001,0.1);
	CMS_Data.cms_status = 1;													
	CMS_Data.output_power_limit = 300;							
	CMS_Data.input_power_limit 	= 150;	
};

void ChassisInfUpdate()
{
	memcpy(&Chassis.Motor3508[0], &LEFT_FRONT_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor3508[1], &RIGHT_FRONT_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor3508[2], &RIGHT_BACK_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor3508[3], &LEFT_BACK_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[0], &LEFT_FRONT_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[1], &RIGHT_FRONT_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[2], &RIGHT_BACK_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[3], &LEFT_BACK_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
}

void ChassisModeUpdate()
{
	
	switch (PTZ.ChassisStatueRequest)
	{
	case 0x01:
		Chassis.Mode = NOFORCE;
		break;
	case 0x12:
	case 0x32:
		Chassis.Mode = ROTING;
		break;
	case 0x0A:
	case 0x2A:
		Chassis.Mode = FALLOW;
		break;
	case 0x06:
	case 0x26:
		Chassis.Mode = STOP;
		break;

	default:
		break;
	}
	if((PTZ.ChassisStatueRequest & (0x01 << 5)) != 0)
	{
		Chassis.CapKey = 1;
	}
	else Chassis.CapKey = 0;
}

void ChassisCommandUpdate()
{
	
	if(chassis_max_power == 45){//注意复活是两倍功率，也要调试！！！
			v_gain=1.0;
	}else if(chassis_max_power == 50){
			v_gain=1.1;
	}else if(chassis_max_power == 55){
			v_gain=1.2;
	}else if(chassis_max_power == 60){
			v_gain=1.3;
	}else if(chassis_max_power == 65){
			v_gain=1.4;
	}else if(chassis_max_power == 70){
			v_gain=1.5;
	}else if(chassis_max_power == 75){
			v_gain=1.6;
	}else if(chassis_max_power == 80){
			v_gain=1.7;
	}else if(chassis_max_power == 85){
			v_gain=1.8;
	}else if(chassis_max_power == 90){
			v_gain=1.9;
	}else if(chassis_max_power == 95){
			v_gain=2.0;
	}else if(chassis_max_power == 100){
			v_gain=2.1;
	}else if(chassis_max_power == 110){
			v_gain=2.2;
	}else if(chassis_max_power == 120){
			v_gain=2.3;
	}else if(chassis_max_power == 130){
			v_gain=2.4;
	}else if(chassis_max_power == 140){
			v_gain=2.5;
	}else if(chassis_max_power == 150){
			v_gain=2.6;
	}else if(chassis_max_power == 160){
			v_gain=2.7;
	}else if(chassis_max_power == 170){
			v_gain=2.8;
	}else if(chassis_max_power == 180){
			v_gain=2.9;
	}else if(chassis_max_power == 190){
			v_gain=3.0;
	}else if(chassis_max_power == 200){
			v_gain=3.1;
	}else{
			v_gain=1.0;
	}
	
	
	

	
	
	
	if (Chassis.Mode == NOFORCE || Offline.PTZnode ==1){
		Chassis.Current_6020[0] = 0 ;
		Chassis.Current_6020[1] = 0 ;
		Chassis.Current_6020[2] = 0 ;
		Chassis.Current_6020[3] = 0 ;
		Chassis.Current_3508[0] = 0 ;
		Chassis.Current_3508[1] = 0 ;
		Chassis.Current_3508[2] = 0 ;
		Chassis.Current_3508[3] = 0 ;
		return;	
	}
	if (Chassis.Mode == FALLOW || Chassis.Mode == ROTING || Chassis.Mode == STOP ){
		follow_angle = loop_fp32_constrain(FollowAngle, YawMotorMeasure.angle - 180.0f,YawMotorMeasure.angle + 180.0f);
		
		if (Chassis.Mode == FALLOW){
			angle_minus = -YawMotorMeasure.angle + FollowAngle;
			if(angle_minus>180){
				angle_minus-=360;
			}else if(angle_minus < -180) {
				angle_minus += 360;
			}
			
			Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus/180.0*PI) - (PTZ.LRSpeed / 32767.0f) * sin(angle_minus/180.0*PI)) *v_gain;
			Chassis.vy = ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus/180.0*PI) + (PTZ.LRSpeed / 32767.0f) * cos(angle_minus/180.0*PI)) *v_gain;
			Chassis.wz = -PID_calc(&follow,YawMotorMeasure.angle,follow_angle); //* (1.0f + Chassis.Power_Proportion /chassis_max_power );
			
			if(Fabs(Chassis.wz)<0.2*v_gain&&Fabs(angle_minus)<0.5){
				Chassis.wz=0.01*Chassis.wz/Fabs(Chassis.wz);
			}
			
		}else if (Chassis.Mode == ROTING){
			power_control_pid.Iout=0;
			angle_minus = -YawMotorMeasure.angle + FollowAngle - YawMotorMeasure.speed_rpm * 0.65;
			Chassis.wz = -1.2;
			Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus/180.0*PI) - (PTZ.LRSpeed / 32767.0f) * sin(angle_minus/180.0*PI))*v_gain;
			Chassis.vy = ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus/180.0*PI) + (PTZ.LRSpeed / 32767.0f) * cos(angle_minus/180.0*PI))*v_gain;
		}else if (Chassis.Mode == STOP){
			angle_minus = -YawMotorMeasure.angle + FollowAngle;
			Chassis.vy = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus/180.0*PI) - (PTZ.LRSpeed / 32767.0f) * sin(angle_minus/180.0*PI)) ;
			Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus/180.0*PI) + (PTZ.LRSpeed / 32767.0f) * cos(angle_minus/180.0*PI)) ;
			Chassis.wz = 0.0;
		}
		
	}
	
		/*********************** 6020角度解算 ***********************/ 

		
		if (Fabs(PTZ.FBSpeed / 32767.0f) > 0.05 || Fabs(PTZ.LRSpeed / 32767.0f) > 0.05 ){
			for (uint8_t i = 0; i < 4; ){
				Chassis.WheelAngle[i] = atan2((Chassis.vy) + Chassis.wz * gen2 * Direction[i], (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1])) / 3.1415927 * 180.0 + Angle_zero_6020[i]; // ?????????
				i++;
			}
			count_down=0;
		}else{
			if(Chassis.wz > 0){
				Chassis.WheelAngle[0] = -135.0f + Angle_zero_6020[0];
				Chassis.WheelAngle[1] = -45.0f + Angle_zero_6020[1]; // 默认角度
				Chassis.WheelAngle[2] = 45.0f + Angle_zero_6020[2];
				Chassis.WheelAngle[3] = 135.0f + Angle_zero_6020[3];
			}else if(Chassis.wz < 0){
				Chassis.WheelAngle[0] = 45.0f + Angle_zero_6020[0];
				Chassis.WheelAngle[1] = 135.0f + Angle_zero_6020[1]; // 默认角度
				Chassis.WheelAngle[2] = -135.0f + Angle_zero_6020[2];
				Chassis.WheelAngle[3] = -45.0f + Angle_zero_6020[3];							
			}else if(Chassis.wz == 0){
				Chassis.WheelAngle[0] = 0 + Angle_zero_6020[0];
				Chassis.WheelAngle[1] = 0 + Angle_zero_6020[1]; // 默认角度
				Chassis.WheelAngle[2] = 0 + Angle_zero_6020[2];
				Chassis.WheelAngle[3] = 0 + Angle_zero_6020[3];						
			}
		}
			
		
		Chassis.WheelAngle[0] = loop_fp32_constrain(Chassis.WheelAngle[0], LEFT_FRONT_6020_Measure.angle - 180.0f, LEFT_FRONT_6020_Measure.angle + 180.0f);
		Chassis.WheelAngle[1] = loop_fp32_constrain(Chassis.WheelAngle[1], RIGHT_FRONT_6020_Measure.angle - 180.0f, RIGHT_FRONT_6020_Measure.angle + 180.0f);
		Chassis.WheelAngle[2] = loop_fp32_constrain(Chassis.WheelAngle[2], RIGHT_BACK_6020_Measure.angle - 180.0f, RIGHT_BACK_6020_Measure.angle + 180.0f);
		Chassis.WheelAngle[3] = loop_fp32_constrain(Chassis.WheelAngle[3], LEFT_BACK_6020_Measure.angle - 180.0f, LEFT_BACK_6020_Measure.angle + 180.0f);		

		/*********************** 3508速度解算 ***********************/
		
		for (uint8_t i = 0; i < 4;){
			speed[i] = sqrtf((Chassis.vy + Chassis.wz * gen2 * Direction[i]) * (Chassis.vy + Chassis.wz * gen2 * Direction[i]) 
							+ (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1]) * (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1]));
			i++;
		}
		Chassis.WheelSpeed[0] = -speed[0];
		Chassis.WheelSpeed[1] = speed[1];
		Chassis.WheelSpeed[2] = speed[2];
		Chassis.WheelSpeed[3] = -speed[3];

		Angle_Speed_calc(); // 角度优化

		Chassis.speed_6020[0] = PID_calc(&left_front_6020_position_pid, LEFT_FRONT_6020_Measure.angle, Chassis.WheelAngle[0]);
		Chassis.speed_6020[1] = PID_calc(&right_front_6020_position_pid, RIGHT_FRONT_6020_Measure.angle, Chassis.WheelAngle[1]);
		Chassis.speed_6020[2] = PID_calc(&right_back_6020_position_pid, RIGHT_BACK_6020_Measure.angle, Chassis.WheelAngle[2]);
		Chassis.speed_6020[3] = PID_calc(&left_back_6020_position_pid, LEFT_BACK_6020_Measure.angle, Chassis.WheelAngle[3]);
	


		
	ChassisCurrentUpdate();

	
	Mode_last = Mode_now;
	Mode_now = Chassis.Mode;

}

	

void Angle_Speed_calc()
{
	for (uint8_t i = 0; i < 4; i++){
		if (Chassis.WheelAngle[i] - Chassis.Motor6020[i].angle > 90.0f){
			Chassis.WheelAngle[i] -= 180.0f;
			Chassis.WheelSpeed[i] = -Chassis.WheelSpeed[i];
		}
		if (Chassis.WheelAngle[i] - Chassis.Motor6020[i].angle < -90.0f){
			Chassis.WheelAngle[i] += 180.0f;
			Chassis.WheelSpeed[i] = -Chassis.WheelSpeed[i];
		}
	}
}

void ChassisCurrentUpdate()
{
	Chassis.Current_6020[0] = PID_calc(&left_front_6020_speed_pid, LEFT_FRONT_6020_Measure.speed_rpm, Chassis.speed_6020[0]);;
	Chassis.Current_6020[1] = PID_calc(&right_front_6020_speed_pid, RIGHT_FRONT_6020_Measure.speed_rpm, Chassis.speed_6020[1]);
	Chassis.Current_6020[2] = PID_calc(&right_back_6020_speed_pid, RIGHT_BACK_6020_Measure.speed_rpm, Chassis.speed_6020[2]);;
	Chassis.Current_6020[3] = PID_calc(&left_back_6020_speed_pid, LEFT_BACK_6020_Measure.speed_rpm, Chassis.speed_6020[3]);
	
	Chassis.Current_3508[0] = PID_calc(&left_front_3508_pid, LEFT_FRONT_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[0]);;
	Chassis.Current_3508[1] = PID_calc(&right_front_3508_pid, RIGHT_FRONT_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[1]);
	Chassis.Current_3508[2] = PID_calc(&right_back_3508_pid, RIGHT_BACK_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[2]);
	Chassis.Current_3508[3] = PID_calc(&left_back_3508_pid, LEFT_BACK_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[3]);
}


extern uint16_t cms_offline_counter;
void CMS__()
{
//默认
//	if(CMS_Data.cms_status != 0 ){//cms用不了
//		CMS_Data.cms_mode = Noforce;
//	}
	if(CMS_Data.cms_cap_v < 12 || power_heat_data_t.buffer_energy < 30 || cms_offline_counter > 200){ //cms没电了
		CMS_Data.cms_mode = Normal;	
	}else if(	CMS_Data.cms_cap_v > 21 && Chassis.CapKey == 1 ) {//飞坡模式 && CMS_Data.cms_mode == Faster
		CMS_Data.cms_mode = Flying;	
	}else if(CMS_Data.cms_mode == Flying && CMS_Data.cms_cap_v > 15 && Chassis.CapKey == 1) {//飞坡模式 && CMS_Data.cms_mode == Faster
		CMS_Data.cms_mode = Flying;	
	}else if(	CMS_Data.cms_cap_v > 18 && CMS_Data.cms_mode == Normal){
		CMS_Data.cms_mode = Faster;	
	}else if(	CMS_Data.cms_mode == Faster && CMS_Data.cms_cap_v > 15){
		CMS_Data.cms_mode = Faster;	
	}else {
		CMS_Data.cms_mode = Normal;
	}
	
	cms_offline_counter ++;
	
	
	if(CMS_Data.cms_mode == Normal){
		chassis_max_power = robot_state.chassis_power_limit;
	}else if(CMS_Data.cms_mode == Faster){
		chassis_max_power = robot_state.chassis_power_limit + 15;
		if(chassis_max_power > 100){
			chassis_max_power = 100;//复活，给电容充电
		}
	}else if(CMS_Data.cms_mode == Flying){
		chassis_max_power = 200;
	}
}

float initial_total_power = 0;
//fp32 chassis_max_power = 45;

void chassis_power_control(Chassis_t *Chassis)
{

	//uint16_t max_power_limit = robot_state.chassis_power_limit;
	
	float Plimit = 1;
	float input_power = 0;		 // input power from battery (referee system)
	float initial_give_power[4]; // initial power from PID calculation
	float initial_give_power_6020[4];
	//float initial_give_power_6020[4]; // initial power from PID calculation
	initial_total_power = 0;
	fp32 scaled_give_power[4];

	fp32 chassis_power = power_heat_data_t.chassis_power;
	fp32 chassis_power_buffer = power_heat_data_t.buffer_energy;

	fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
	fp32 toque_coefficient_6020 = 9.31e-6f; // (3/25000)*(0.741)/9.55
	fp32 a = 1.23e-07;						 // k1
	fp32 k2 = 1.453e-07;					 // k2
	fp32 constant = 4.581f;

//get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
//	PID_calc(&chassis_power_control->buffer_pid, chassis_power_buffer, 30);
//	input_power = max_power_limit - chassis_power_control->buffer_pid.out; // Input power floating at maximum power

  // set the input power of capacitor controller


//6020先不考虑
	for (uint8_t i = 0; i < 4; i++){ // first get all the initial motor power and total motor power
		initial_give_power[i] = Chassis->Current_3508[i] * toque_coefficient * Chassis->Motor3508[i].speed_rpm +
								k2 * Chassis->Motor3508[i].speed_rpm * Chassis->Motor3508[i].speed_rpm +
								a * Chassis->Current_3508[i] * Chassis->Current_3508[i] + constant;
		  initial_give_power_6020[i] = Chassis->Current_6020[i] * toque_coefficient_6020 * Chassis->Motor6020[i].speed_rpm;
//    initial_give_power_6020[i] = Chassis->Current_6020[i] * toque_coefficient_6020 * Chassis->Motor6020[i].speed_rpm
//								+k2 * Chassis->Motor6020[i].speed_rpm * Chassis->Motor6020[i].speed_rpm +
//								a * Chassis->Current_6020[i] * Chassis->Current_6020[i] ;
//		if (initial_give_power < 0) // negative power not included (transitory)
//			continue;
		
		initial_total_power = initial_total_power + initial_give_power[i] + initial_give_power_6020[i];
		//initial_total_power = initial_total_power + initial_give_power[i];
	}

	if (initial_total_power > chassis_max_power){ // determine if larger than max power
		fp32 power_scale = chassis_max_power / initial_total_power;
		for (uint8_t i = 0; i < 4; i++){
			scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
			if (scaled_give_power[i] < 0){
				continue;
			}

			fp32 b = toque_coefficient * Chassis->Motor3508[i].speed_rpm;
			fp32 c = k2 * Chassis->Motor3508[i].speed_rpm * Chassis->Motor3508[i].speed_rpm - scaled_give_power[i] + constant;

			if (Chassis->Current_3508[i] > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp > 16000)
				{
					Chassis->Current_3508[i] = 16000;
				} else {
					Chassis->Current_3508[i] = temp;
				}
			}
			else
			{
				fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp < -16000)
				{
					Chassis->Current_3508[i] = -16000;
				}
				else
					Chassis->Current_3508[i] = temp;
			}
		}
		
		//保护
		if(power_heat_data_t.buffer_energy<30&&power_heat_data_t.buffer_energy>=20)	Plimit=0.25;
		else if(power_heat_data_t.buffer_energy<20&&power_heat_data_t.buffer_energy>=10)	Plimit=0.125;
		else if(power_heat_data_t.buffer_energy<10&&power_heat_data_t.buffer_energy>=0)	Plimit=0.05;
		else 	Plimit=1;
		for (uint8_t i = 0; i < 4; i++)
		{
			Chassis->Current_3508[i] = Chassis->Current_3508[i]*Plimit;
		}
			
	}
	
}
