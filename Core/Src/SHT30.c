#include "SHT30.h"
#include "myiic.h"

uint8_t Buff[6]={0};  // u8

#define EEPROM_I2C_WR	0		/* д����bit */
#define EEPROM_I2C_RD	1		/* ������bit */
float Temperature ;
float Humidity ;


uint8_t sht30_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;
	
	I2C_Start();		/* ���������ź� */

	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	I2C_Send_Byte(_Address | EEPROM_I2C_WR);
	ucAck = I2C_Wait_Ack();	/* ����豸��ACKӦ�� */

	I2C_Stop();			/* ����ֹͣ�ź� */

	return ucAck;
}
void Sht30_Init(void)
{	
	// �ڷ���I2C�豸ǰ�����ȵ��� i2c_CheckDevice() ���I2C�豸�Ƿ��������ú���������GPIO
while (sht30_CheckDevice(0x88+0));  //�����豸��ַ+д�ź� 
	
}


/**
  * @brief  SHT30д����
  * @param  msb
            lsb
  * @retval None
  */
void Sht30_WriteCmd(uint8_t msb,uint8_t lsb)
	{
		I2C_Start();//��ʼ�ź�
    I2C_Send_Byte(0x88+0); //�����豸��ַ+д�ź�
		while (I2C_Wait_Ack());//CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
		I2C_Send_Byte(msb); //���������λ
		while (I2C_Wait_Ack());//CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
		I2C_Send_Byte(lsb); //���������λ
		while (I2C_Wait_Ack());//CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
		I2C_Stop();  // CPU����I2C����ֹͣ�ź�

	 }
	
	 
 
/**
  * @brief  SHT30������
  * @param  None
  * @retval None
  */
void Sht30_ReadData(void)
	{   	
		Sht30_WriteCmd(0x21,0x26);	//���ݲɼ�Ƶ��1 m/s
		I2C_Start();//��ʼ�ź�
		I2C_Send_Byte(0x88+1); //�����豸��ַ+���ź�
		while (I2C_Wait_Ack());//CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
		Buff[0]=I2C_Read_Byte(0);//��ȡ�¶ȸ߰�λ���ҷ���һ��ack
		I2C_Ack();
		Buff[1]=I2C_Read_Byte(0); //�Ͱ�λ
		I2C_Ack();
		Buff[2]=I2C_Read_Byte(0); //У��λ����ACK8λ
		I2C_Ack();
		Buff[3] = I2C_Read_Byte(0);  //ʪ�ȷ���ACK��8λ
		I2C_Ack();
		Buff[4] = I2C_Read_Byte(0);  //ʪ�ȵͰ�λ
		I2C_Ack();
		Buff[5] = I2C_Read_Byte(0);  //У��λ֮����nack����������ֲ�
		I2C_NAck();
		I2C_Stop();  // CPU����I2C����ֹͣ�ź�	
	}	
	
/**
  * @brief  ��ȡ�����ݽ��й�ʽת��
  * @param  None
  * @retval None
  */
void Convert_sht30(void)
{
    Temperature = (float)175*((Buff[0]<<8)+Buff[1])/65535-45;    //����һλ�൱�ڳ�2������8λ*2^8��
    Humidity = (float)100*((Buff[3]<<8)+Buff[4])/65535;

    Buff[0] = 0;
    Buff[1] = 0;
    Buff[2] = 0;
    Buff[3] = 0;
    Buff[4] = 0;
    Buff[5] = 0;
}
	