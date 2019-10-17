#include "icm20608.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"

/*********************************************************************************
			  ___   _     _____  _____  _   _  _____  _____  _   __
			 / _ \ | |   |_   _||  ___|| \ | ||_   _||  ___|| | / /
			/ /_\ \| |     | |  | |__  |  \| |  | |  | |__  | |/ /
			|  _  || |     | |  |  __| | . ` |  | |  |  __| |    \
			| | | || |_____| |_ | |___ | |\  |  | |  | |___ | |\  \
			\_| |_/\_____/\___/ \____/ \_| \_/  \_/  \____/ \_| \_/

 *	******************************************************************************
 *	������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 *	ALIENTEK Pandora STM32L475 IOT������
 *	ICM20608��������
 *	����ԭ��@ALIENTEK
 *	������̳:www.openedv.com
 *	��������:2018/10/27
 *	�汾��V1.0
 *	��Ȩ���У�����ؾ���
 *	Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 *	All rights reserved
 *	******************************************************************************
 *	��ʼ�汾
 *	******************************************************************************/

/**
 * @brief	ICM20608��ʼ������
 *
 * @param   void
 *
 * @return  u8		0,�ɹ�������,ʧ��
 */
u8 ICM20608_Init(void)
{
    u8 res;

    IIC_Init();//��ʼ��IIC����
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80);	//��λICM20608
    delay_ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00);	//����ICM20608
    MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
    MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
    MPU_Set_Rate(50);						//���ò�����50Hz
    MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	//�ر������ж�
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00);	//I2C��ģʽ�ر�
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);	//�ر�FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80);	//INT���ŵ͵�ƽ��Ч

    MPU_Write_Byte(0x1D, 1);	//INT���ŵ͵�ƽ��Ч

    res = MPU_Read_Byte(MPU_DEVICE_ID_REG);

    if(res == 0XAF) //����ID��ȷ
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01);	//����CLKSEL,PLL X��Ϊ�ο�
        MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00);	//���ٶ��������Ƕ�����
        MPU_Set_Rate(50);						//���ò�����Ϊ50Hz
    }
    else return 1;

    return 0;

}
/**
 * @brief	����ICM20608���ٶȴ����������̷�Χ
 *
 * @param   fsr		0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
 *
 * @return  u8		0,�ɹ�������,ʧ��
 */
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); //���������������̷�Χ
}
/**
 * @brief	����ICM20608�����Ǵ����������̷�Χ
 *
 * @param   fsr		0,��2g;1,��4g;2,��8g;3,��16g
 *
 * @return  u8		0,�ɹ�������,ʧ��
 */
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); //���ü��ٶȴ����������̷�Χ
}
/**
 * @brief	����ICM20608�����ֵ�ͨ�˲���
 *
 * @param   lpf		���ֵ�ͨ�˲�Ƶ��(Hz)
 *
 * @return  u8		0,�ɹ�������,ʧ��
 */
u8 MPU_Set_LPF(u16 lpf)
{
    u8 data = 0;

    if(lpf >= 188)data = 1;

    else if(lpf >= 98)data = 2;

    else if(lpf >= 42)data = 3;

    else if(lpf >= 20)data = 4;

    else if(lpf >= 10)data = 5;

    else data = 6;

    return MPU_Write_Byte(MPU_CFG_REG, data); //�������ֵ�ͨ�˲���
}

/**
 * @brief	����ICM20608�Ĳ�����(�ٶ�Fs=1KHz)
 *
 * @param   rate	4~1000(Hz)
 *
 * @return  u8		0,�ɹ�������,ʧ��
 */
u8 MPU_Set_Rate(u16 rate)
{
    u8 data;

    if(rate > 1000)rate = 1000;

    if(rate < 4)rate = 4;

    data = 1000 / rate - 1;
    data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);	//�������ֵ�ͨ�˲���
    return MPU_Set_LPF(rate / 2);	//�Զ�����LPFΪ�����ʵ�һ��
}
/**
 * @brief	�õ��¶�ֵ
 *
 * @param   rate	4~1000(Hz)
 *
 * @return  short	�¶�ֵ(������100��)
 */
short MPU_Get_Temperature(void)
{
    u8 buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((u16)buf[0] << 8) | buf[1];
    temp = 25 + ((double)raw) / 326.8;
    return temp * 100;;
}

/**
 * @brief	�õ�������ֵ(ԭʼֵ)
 *
 * @param   gx,gy,gz	������x,y,z���ԭʼ����(������)
 *
 * @return  u8			0,�ɹ�������,ʧ��
 */
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    u8 buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);

    if(res == 0)
    {
        *gx = ((u16)buf[0] << 8) | buf[1];
        *gy = ((u16)buf[2] << 8) | buf[3];
        *gz = ((u16)buf[4] << 8) | buf[5];
    }

    return res;;
}
/**
 * @brief	�õ����ٶ�ֵ(ԭʼֵ)
 *
 * @param   ax,ay,az	���ٶ�ֵx,y,z���ԭʼ����(������)
 *
 * @return  u8			0,�ɹ�������,ʧ��
 */
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    u8 buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);

    if(res == 0)
    {
        *ax = ((u16)buf[0] << 8) | buf[1];
        *ay = ((u16)buf[2] << 8) | buf[3];
        *az = ((u16)buf[4] << 8) | buf[5];
    }

    return res;;
}
/**
 * @brief	IIC����д
 *
 * @param   addr	������ַ
 * @param   reg		�Ĵ�����ַ
 * @param   len		д�볤��
 * @param   buf		������
 *
 * @return  u8			0,�ɹ�������,ʧ��
 */
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0); //����������ַ+д����

    if(IIC_Wait_Ack())	//�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��

    for(i = 0; i < len; i++)
    {
        IIC_Send_Byte(buf[i]);	//��������

        if(IIC_Wait_Ack())		//�ȴ�ACK
        {
            IIC_Stop();
            return 1;
        }
    }

    IIC_Stop();
    return 0;
}

/**
 * @brief	IIC������
 *
 * @param   addr	������ַ
 * @param   reg		Ҫ��ȡ�ļĴ�����ַ
 * @param   len		Ҫ��ȡ�ĳ���
 * @param   buf		��ȡ�������ݴ洢��
 *
 * @return  u8			0,�ɹ�������,ʧ��
 */
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0); //����������ַ+д����

    if(IIC_Wait_Ack())	//�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 1); //����������ַ+������
    IIC_Wait_Ack();		//�ȴ�Ӧ��

    while(len)
    {
        if(len == 1)*buf = IIC_Read_Byte(0); //������,����nACK

        else *buf = IIC_Read_Byte(1);		//������,����ACK

        len--;
        buf++;
    }

    IIC_Stop();	//����һ��ֹͣ����
    return 0;
}
/**
 * @brief	IICдһ���ֽ�
 *
 * @param   reg		�Ĵ�����ַ
 * @param   data		����
 *
 * @return  u8			0,�ɹ�������,ʧ��
 */
u8 MPU_Write_Byte(u8 reg, u8 data)
{
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 0); //����������ַ+д����

    if(IIC_Wait_Ack())	//�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Send_Byte(data);//��������

    if(IIC_Wait_Ack())	//�ȴ�ACK
    {
        IIC_Stop();
        return 1;
    }

    IIC_Stop();
    return 0;
}
/**
 * @brief	IIC��һ���ֽ�
 *
 * @param   reg		�Ĵ�����ַ
 *
 * @return  u8		����������
 */
u8 MPU_Read_Byte(u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 0); //����������ַ+д����
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 1); //����������ַ+������
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    res = IIC_Read_Byte(0); //��ȡ����,����nACK
    IIC_Stop();			//����һ��ֹͣ����
    return res;
}


