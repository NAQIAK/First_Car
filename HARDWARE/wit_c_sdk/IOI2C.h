#ifndef __IOI2C_H
#define __IOI2C_H
#include "sys.h"

#define IIC_RCC_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

#define IIC_SCL_Pin       GPIO_PIN_8
#define IIC_SCL_GPIO_Port GPIOB
#define IIC_SDA_Pin       GPIO_PIN_9
#define IIC_SDA_GPIO_Port GPIOB//ע����Щ��mpu6050���õ�IICע�����


//IO�������ã���ѯstm32���Ĳο��ֲ�CR�Ĵ�����
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<4;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<4;}

//IO��������	 
#define IIC_SCL    PBout(8) //SCL
#define IIC_SDA    PBout(9) //SDA	 
#define READ_SDA   PBin(9)  //����SDA 


void IIC_Init(void);                			 
void IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(uint8_t txd);			
uint8_t IIC_Read_Byte(unsigned char ack);
uint8_t IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);				

int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);



#endif

//------------------End of File----------------------------
