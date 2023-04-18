#include "encoder.h"
#include "sys.h"


int Read_Speed(int Wheel){
	int speed;
	switch(Wheel){
		case 3: 
			speed = __HAL_TIM_GetCounter(&htim3);
			__HAL_TIM_SetCounter(&htim3, 0);
			break;
		case 4: 
			speed = __HAL_TIM_GetCounter(&htim4);
			if(speed != 0)
				speed = 65536-speed;	
			__HAL_TIM_SetCounter(&htim4, 0);
			break;
		default: 
			speed =0;
	}
	return speed;
}
/*************************************
∂¡»°æ‡¿Î


*************************************/
int read_distance(int Wheel){
	int distance;
	switch(Wheel){
		case 3: 
			distance = __HAL_TIM_GetCounter(&htim3);
			break;
		case 4: 
			distance = __HAL_TIM_GetCounter(&htim4);
			if(distance != 0)
				distance = 65536-distance;
			break;
	}
	return distance;
}
