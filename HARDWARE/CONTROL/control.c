/************************************
作者：paopao
编写时间：2023/3/24
PWM控制函数
************************************/
#include "control.h"
//陀螺仪PID控制
float Turn_Kp =40,Turn_Kd =2;
float fAcc[3], fGyro[3], fAngle[3];
float Yaw_exp = 0,Yaw_err=0;
//编码器增量式PID控制
float encoder_Kp=1,encoder_Ki=0,encoder_Kd=0;
int previous01_error = 0;
int previous02_error = 0;
//位置标志记录
int A[4] = {19,20,21,17};
int B[4] = {1,20,13,15};
int C[4] = {4,7,12,10};
int D[4] = {5,6,9,10};

/************************************
直线行走函数带自稳
Direection ：forward backword
Speed：0-1000
************************************/
void go_forward(void){
		HAL_GPIO_WritePin(GPIOF, Bin1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, Bin2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, Ain1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Ain2_Pin, GPIO_PIN_SET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, limit(200-PD_control()));// PD向左转为正数，向右转为负数
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, limit(200+PD_control()));
}
/************************************
直线行走函数带自稳
Direection ：forward backword
Speed：0-1000
************************************/
void go_encoder(void){
		HAL_GPIO_WritePin(GPIOF, Bin1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, Bin2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, Ain1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Ain2_Pin, GPIO_PIN_SET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, limit(200+encoder_PID()));// PD向左转为正数，向右转为负数
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, limit(200-encoder_PID()));
}
/***********************************
停止函数

***********************************/
void stop(void){
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
}

/*********************************
角度更新函数
更新角度和角速度作为PID参数
*********************************/
void Read_Angel(void){
	int i;
	WitReadReg(AX, 12);
	if(s_cDataUpdate){
			for(i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}			
			fAngle[2]-=Yaw_err;
			if(fAngle[2]<-180)
				fAngle[2]+=360;
			if(fAngle[2]>180)
				fAngle[2]-=360;
		}
	printf("%f\r\n",fAngle[2]);

}

/**************************************
PD控制函数
Yaw本次偏转角，LYaw上次偏转角
输出函数为PD补偿
**************************************/
float PD_control(void){
	Read_Angel();
	float PD_out;
	PD_out =Turn_Kp*fAngle[2]+Turn_Kd*fGyro[2];
	return PD_out;
}
/*************************************
转向函数
dirtion:left right
*************************************/
void Turn(int direction){
	HAL_GPIO_WritePin(GPIOF, Bin1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, Bin2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, Ain1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, Ain2_Pin, GPIO_PIN_SET);
	__HAL_TIM_SetCounter(&htim4, 0);
	__HAL_TIM_SetCounter(&htim3, 0);
	if(direction == Left){
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 200);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
		while(read_distance(Right)<=1330){}			
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
			Yaw_err+=90;
			if(Yaw_err>=360)
				Yaw_err-=360;
	}
	else{
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 200);
		while(read_distance(Left)<=1330){}
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
			Yaw_err-=90;
			if(Yaw_err<=-360)
				Yaw_err+=360;
	}
	__HAL_TIM_SetCounter(&htim3, 0);
	__HAL_TIM_SetCounter(&htim4, 0);
	
}
/***********************************
限幅函数
将载入到电机上的值限定为(0-1000)
**********************************/
float limit(float x){
	float y=x;
	if(y<0)y=0;
	if(y>1000)y=1000;
	return y;
}
/*************************
向前行走固定距离
0<distance<65536
*************************/
void go_distance(int distance){
	__HAL_TIM_SetCounter(&htim4, 0);
	__HAL_TIM_SetCounter(&htim3, 0);
	while((read_distance(Right)+read_distance(Left))/2<distance)
		go_forward();
	stop();
	__HAL_TIM_SetCounter(&htim4, 0);
	__HAL_TIM_SetCounter(&htim3, 0);
}
/*************************
掉头函数
*************************/
void turn_round(void){
	HAL_GPIO_WritePin(GPIOF, Bin1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, Bin2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, Ain1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, Ain2_Pin, GPIO_PIN_SET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 200);
	while(read_distance(Left)<=1330)
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
	delay_ms(500);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
	__HAL_TIM_SetCounter(&htim4, 1);
	while(read_distance(Right)>=(65536-1330));
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
	Yaw_err+=180;
			if(Yaw_err>=360)
				Yaw_err-=360;
}
/***********************************
封装函数题目2
***********************************/
void question2(void){
//	while(room_flag1==0)room_flag1=room_flag1;
	while(position_flag<2)
				go_forward();
			go_distance(1000);
			stop();
			delay_ms(500);
			Turn(Right);
			delay_ms(500);
			while(position_flag<3)
				go_forward();
			go_distance(1000);
			stop();
			delay_ms(500);
			Turn(Left);
			delay_ms(500);
			while(position_flag<5)
				go_forward();
			go_distance(700);
			stop();
			delay_ms(500);
			Turn(Right);
			delay_ms(500);
			while(position_flag<6)
				go_forward();
			go_distance(1000);
			stop();
			delay_ms(500);
			Turn(Right);
			delay_ms(500);
			while(position_flag<7)
				go_forward();
			go_distance(1000);
			stop();
			delay_ms(500);
			Turn(Left);
			delay_ms(500);
			while(position_flag<10)
				go_forward();
			go_distance(500);
			stop();
			delay_ms(500);
			turn_round();
			delay_ms(500);
			while(position_flag<11)
				go_forward();
			go_distance(1000);
			stop();
			delay_ms(500);
			Turn(Left);
			delay_ms(500);
			while(position_flag<14)
				go_forward();
			go_distance(1000);
			stop();
			delay_ms(500);
			Turn(Left);
			delay_ms(500);
			while(position_flag<15)
				go_forward();
			go_distance(500);
			stop();
			turn_round();
			delay_ms(500);
			while(position_flag<16)
				go_forward();
			go_distance(1000);
			stop();
			delay_ms(500);
			Turn(Left);
			delay_ms(500);
			while(position_flag<17)
				go_forward();
			go_distance(500);
			stop();
			delay_ms(500);
			turn_round();
			delay_ms(500);
			while(position_flag<18)
				go_forward();
			go_distance(1000);
			stop();
			delay_ms(500);
			Turn(Left);
			delay_ms(500);
			while(position_flag<22)
				go_forward();
			go_distance(1000);
			stop();
			delay_ms(500);
			Turn(Left);
			delay_ms(500);
			go_distance(1000);
			room_flag1 = 0;
}
/******************************
编码器PID
******************************/
float encoder_PID(void){
	float PID;
	int SPEED_Left=0,SPEED_Right=0;
	SPEED_Left = Read_Speed(Left);
	SPEED_Right = Read_Speed(Right);
	int speed_error = SPEED_Left-SPEED_Right;
	//debug
	OLED_ShowNum(0,0,SPEED_Left,5,16);
	OLED_ShowNum(0,2,SPEED_Left,5,16);
	OLED_ShowNum(0,4,speed_error,5,16);
	//debug
	PID = encoder_Kp*(speed_error-previous01_error)+
				encoder_Ki*speed_error+
				encoder_Kd*(speed_error-2*previous01_error+previous02_error);
	previous01_error=speed_error;
	previous02_error=previous01_error;
	return PID;
}
