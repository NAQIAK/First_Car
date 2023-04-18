/************************************
���ߣ�paopao
��дʱ�䣺2023/3/24
PWM���ƺ���
************************************/
#include "control.h"
//������PID����
float Turn_Kp =40,Turn_Kd =2;
float fAcc[3], fGyro[3], fAngle[3];
float Yaw_exp = 0,Yaw_err=0;
//����������ʽPID����
float encoder_Kp=1,encoder_Ki=0,encoder_Kd=0;
int previous01_error = 0;
int previous02_error = 0;
//λ�ñ�־��¼
int A[4] = {19,20,21,17};
int B[4] = {1,20,13,15};
int C[4] = {4,7,12,10};
int D[4] = {5,6,9,10};

/************************************
ֱ�����ߺ���������
Direection ��forward backword
Speed��0-1000
************************************/
void go_forward(void){
		HAL_GPIO_WritePin(GPIOF, Bin1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, Bin2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, Ain1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Ain2_Pin, GPIO_PIN_SET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, limit(200-PD_control()));// PD����תΪ����������תΪ����
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, limit(200+PD_control()));
}
/************************************
ֱ�����ߺ���������
Direection ��forward backword
Speed��0-1000
************************************/
void go_encoder(void){
		HAL_GPIO_WritePin(GPIOF, Bin1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, Bin2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, Ain1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Ain2_Pin, GPIO_PIN_SET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, limit(200+encoder_PID()));// PD����תΪ����������תΪ����
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, limit(200-encoder_PID()));
}
/***********************************
ֹͣ����

***********************************/
void stop(void){
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
}

/*********************************
�Ƕȸ��º���
���½ǶȺͽ��ٶ���ΪPID����
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
PD���ƺ���
Yaw����ƫת�ǣ�LYaw�ϴ�ƫת��
�������ΪPD����
**************************************/
float PD_control(void){
	Read_Angel();
	float PD_out;
	PD_out =Turn_Kp*fAngle[2]+Turn_Kd*fGyro[2];
	return PD_out;
}
/*************************************
ת����
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
�޷�����
�����뵽����ϵ�ֵ�޶�Ϊ(0-1000)
**********************************/
float limit(float x){
	float y=x;
	if(y<0)y=0;
	if(y>1000)y=1000;
	return y;
}
/*************************
��ǰ���߹̶�����
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
��ͷ����
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
��װ������Ŀ2
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
������PID
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
