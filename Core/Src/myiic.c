#include "myiic.h"
 

void delay_xus(__IO uint32_t nTime)
{
    int old_val,new_val,val;
 
    if(nTime > 900)
    {
        for(old_val = 0; old_val < nTime/900; old_val++)
        {
            delay_xus(900);
        }
        nTime = nTime%900;
    }
 
    old_val = SysTick->VAL;
    new_val = old_val - CPU_FREQUENCY_MHZ*nTime;
    if(new_val >= 0)
    {
        do
        {
            val = SysTick->VAL;
        }
        while((val < old_val)&&(val >= new_val));
    }
    else
    {
        new_val +=CPU_FREQUENCY_MHZ*1000;
        do
        {
            val = SysTick->VAL;
        }
        while((val <= old_val)||(val > new_val));
 
    }
}
 
//--------------------------------------------
void SDA_Output(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = MYI2C_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed =GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(MYI2C_SDA_PORT,&GPIO_InitStruct);
}

void SDA_Input(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = MYI2C_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed =GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(MYI2C_SDA_PORT,&GPIO_InitStruct);
}
 
void SCL_Output(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = MYI2C_SCL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed =GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(MYI2C_SCL_PORT,&GPIO_InitStruct);
}

void SCL_Input(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = MYI2C_SCL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed =GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(MYI2C_SCL_PORT,&GPIO_InitStruct);	
}

void I2C_Init(void)
{
	SCL_Output();
	SDA_Output();
	SCL_Dout_HIGH();
	SDA_Dout_HIGH();
}

//����IIC��ʼ�ź�
void I2C_Start(void)
{
	SDA_Output();
	SDA_Dout_HIGH();
	SCL_Dout_HIGH();
	Delay_us(2);
	SDA_Dout_LOW();
	Delay_us(2);
	SCL_Dout_LOW();
}

//����IICֹͣ�ź�
void I2C_Stop(void)
{
	SDA_Output();
	SCL_Dout_LOW();
	SDA_Dout_LOW();
	Delay_us(2);
	SCL_Dout_HIGH();
	SDA_Dout_HIGH();
	Delay_us(2);						   	
}

uint8_t I2C_Wait_Ack(void)
{
//	uint8_t ucErrTime=0;
//	SDA_Input();
//	SDA_Dout_HIGH();Delay_us(1);	   
//	SCL_Dout_HIGH();Delay_us(1);	 
//	while(SDA_Data_IN())
//	{
//		ucErrTime++;
//		if(ucErrTime>250)
//		{
//			I2C_Stop();
//			return 1;
//		}
//	}
//	SCL_Dout_LOW();//ʱ�����0 	   
//	return 0; 
	
	uint8_t i = 0;
	SDA_Input();
	SCL_Dout_HIGH();
	Delay_us(1);
	while(1 == SDA_Data_IN() && i < 250)
	{
		i++;
	}
	SCL_Dout_LOW();
	SDA_Output();
	Delay_us(2);
	return 0;
}

//����ACKӦ��
void I2C_Ack(void)
{
	SCL_Dout_LOW();
	SDA_Output();
	SDA_Dout_LOW();
	Delay_us(2);
	SCL_Dout_HIGH();
	Delay_us(2);
	SCL_Dout_LOW();
}

//������ACKӦ��		  
void I2C_NAck(void)
{
	SCL_Dout_LOW();
	SDA_Output();
	SDA_Dout_HIGH();
	Delay_us(2);
	SCL_Dout_HIGH();
	Delay_us(2);
	SCL_Dout_LOW();
}

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��	
void I2C_Send_Byte(uint8_t txd) 
{                        
	uint8_t t;      
	//����ʱ�ӿ�ʼ���ݴ���
//	SDA_Output(); // ��
	SCL_Dout_LOW();
	Delay_us(2); 	// ��
	for(t=0;t<8;t++)
	{  
//		SDA_Write((txd&0x5e)>>7);		   
//		txd<<=1; 	  
//		Delay_us(5);   //��TEA5767��������ʱ���Ǳ����
//		SCL_Dout_HIGH();
//		Delay_us(5); 	
//		SCL_Dout_LOW();
//		//Delay_us(2);
		if (txd & 0x80)
		{
			SDA_Dout_HIGH();
		}
		else
		{
			SDA_Dout_LOW();
		}
		Delay_us(1);
		SCL_Dout_HIGH();
		txd <<= 1;
		Delay_us(1);
		SCL_Dout_LOW();
		Delay_us(2);
  }
	SDA_Dout_HIGH();
	SCL_Dout_LOW();
	Delay_us(2);
}

//��1���ֽ�
uint8_t I2C_Read_Byte(uint8_t ack)
{
	unsigned char i,receive=0;
	//SDA����Ϊ����
	SDA_Dout_HIGH(); // ��
	SDA_Input();
  for(i=0;i<8;i++ )
	{
		SCL_Dout_LOW();
		Delay_us(1); // 5
		SCL_Dout_HIGH();
		Delay_us(1); // ��
		receive<<=1;
		receive |= SDA_Data_IN();
//		if(SDA_Data_IN())receive++;   
		Delay_us(1);  // 5
		SCL_Dout_LOW(); // ��
		Delay_us(1); // ��
  }					 
//  if(!ack)I2C_NAck();//����nACK
//  else  I2C_Ack(); //����ACK   
  
	return receive;
}

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t I2C_Receive_Byte(uint8_t ack)
{
	unsigned char i,receive=0;
	//SDA����Ϊ���
	SDA_Output();
	SDA_Dout_HIGH();
	SDA_Input();
	SCL_Dout_LOW();
	
  for(i=0;i<8;i++ )
	{
		SCL_Dout_HIGH();
		Delay_us(1); // ��
		receive<<=1;
		if(0 != SDA_Data_IN())
		{
			receive |= 0x01;
		}			
		Delay_us(1);  // 5
		SCL_Dout_LOW(); // ��
		Delay_us(2); // ��
  }					 
  if(!ack)I2C_NAck();//����nACK
  else  I2C_Ack(); //����ACK   
  
	return receive;
}

/**********************************************************************/
// ����������mcp4018��ʼ��
// �����������
// �����������
// �� �� ֵ����
// ��дʱ�䣺2020.8.29
// ��    �ߣ�
// �޸ļ�¼:
/**********************************************************************/
void MCP4018_Init_Myiic(void)
{
	I2C_Init();
}

/**********************************************************************/
// ����������дmcp4018����
// ���������write_data��д������
// �����������
// �� �� ֵ����
// ��дʱ�䣺2020.8.29
// ��    �ߣ�
// �޸ļ�¼:
/**********************************************************************/
void MCP4018_WriteData_Myiic(uint8_t* write_data)
{
	I2C_Start();
	I2C_Send_Byte(0x5e);
	I2C_Wait_Ack();
	I2C_Send_Byte(write_data[0]);
	I2C_Wait_Ack();
	I2C_Stop();
	printf("\r\nwrite mcp4018: 0x%02x", write_data[0]);
}

/**********************************************************************/
// ������������ȡmcp4018����
// ���������void
// �����������
// �� �� ֵ����
// ��дʱ�䣺2020.8.29
// ��    �ߣ�
// �޸ļ�¼:
/**********************************************************************/
void MCP4018_ReadData_Myiic(void)
{
	uint8_t GET_DATA = 0;
	I2C_Start();
	I2C_Send_Byte(0x5e|1);
	I2C_Wait_Ack();
	GET_DATA = I2C_Read_Byte(0);
	I2C_Wait_Ack();
	I2C_Stop();
	printf("\r\nread mcp4018: 0x%02x", GET_DATA);
}

