#include "SHT30.h"
#include "myiic.h"

uint8_t Buff[6]={0};  // u8

#define EEPROM_I2C_WR	0		/* 写控制bit */
#define EEPROM_I2C_RD	1		/* 读控制bit */
float Temperature ;
float Humidity ;


uint8_t sht30_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;
	
	I2C_Start();		/* 发送启动信号 */

	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	I2C_Send_Byte(_Address | EEPROM_I2C_WR);
	ucAck = I2C_Wait_Ack();	/* 检测设备的ACK应答 */

	I2C_Stop();			/* 发送停止信号 */

	return ucAck;
}
void Sht30_Init(void)
{	
	// 在访问I2C设备前，请先调用 i2c_CheckDevice() 检测I2C设备是否正常，该函数会配置GPIO
while (sht30_CheckDevice(0x88+0));  //发送设备地址+写信号 
	
}


/**
  * @brief  SHT30写命令
  * @param  msb
            lsb
  * @retval None
  */
void Sht30_WriteCmd(uint8_t msb,uint8_t lsb)
	{
		I2C_Start();//起始信号
    I2C_Send_Byte(0x88+0); //发送设备地址+写信号
		while (I2C_Wait_Ack());//CPU产生一个时钟，并读取器件的ACK应答信号
		I2C_Send_Byte(msb); //发送命令高位
		while (I2C_Wait_Ack());//CPU产生一个时钟，并读取器件的ACK应答信号
		I2C_Send_Byte(lsb); //发送命令低位
		while (I2C_Wait_Ack());//CPU产生一个时钟，并读取器件的ACK应答信号
		I2C_Stop();  // CPU发起I2C总线停止信号

	 }
	
	 
 
/**
  * @brief  SHT30读数据
  * @param  None
  * @retval None
  */
void Sht30_ReadData(void)
	{   	
		Sht30_WriteCmd(0x21,0x26);	//数据采集频率1 m/s
		I2C_Start();//起始信号
		I2C_Send_Byte(0x88+1); //发送设备地址+读信号
		while (I2C_Wait_Ack());//CPU产生一个时钟，并读取器件的ACK应答信号
		Buff[0]=I2C_Read_Byte(0);//读取温度高八位并且发送一个ack
		I2C_Ack();
		Buff[1]=I2C_Read_Byte(0); //低八位
		I2C_Ack();
		Buff[2]=I2C_Read_Byte(0); //校验位发送ACK8位
		I2C_Ack();
		Buff[3] = I2C_Read_Byte(0);  //湿度发送ACK高8位
		I2C_Ack();
		Buff[4] = I2C_Read_Byte(0);  //湿度低八位
		I2C_Ack();
		Buff[5] = I2C_Read_Byte(0);  //校验位之后发送nack具体见数据手册
		I2C_NAck();
		I2C_Stop();  // CPU发起I2C总线停止信号	
	}	
	
/**
  * @brief  读取的数据进行公式转换
  * @param  None
  * @retval None
  */
void Convert_sht30(void)
{
    Temperature = (float)175*((Buff[0]<<8)+Buff[1])/65535-45;    //左移一位相当于乘2，左移8位*2^8。
    Humidity = (float)100*((Buff[3]<<8)+Buff[4])/65535;

    Buff[0] = 0;
    Buff[1] = 0;
    Buff[2] = 0;
    Buff[3] = 0;
    Buff[4] = 0;
    Buff[5] = 0;
}
	