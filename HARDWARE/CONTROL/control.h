#ifndef __PWM_H
#define __PWM_H	

#include "sys.h"


extern float Turn_Kp,Turn_Kd;
extern float fAcc[3], fGyro[3], fAngle[3];
extern float Yaw_exp,Yaw_err;

extern float encoder_Kp,encoder_Ki,encoder_Kd;
extern int previous01_error;
extern int previous02_error;

void go_forward(void);
void go_encoder(void);
void Read_Angel(void);
float PD_control(void);
void Turn(int direction);
float limit(float x);
void stop(void);
void go_distance(int distance);
void turn_round(void);
void question2(void);
float encoder_PID(void);



#endif
