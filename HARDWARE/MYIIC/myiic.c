#include "myiic.h"
#include "delay.h"

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
 *	IIC��������
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
 * @brief	IIC��ʼ������
 *
 * @param   void
 *
 * @return  void
 */
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_GPIOC_CLK_ENABLE();   //ʹ��GPIOCʱ��

    //PH4,5��ʼ������
    GPIO_Initure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP; //�������
    GPIO_Initure.Pull = GPIO_PULLUP;        //����
    GPIO_Initure.Speed = GPIO_SPEED_FAST;   //����
    HAL_GPIO_Init(GPIOC, &GPIO_Initure);

    IIC_SDA(1);
    IIC_SCL(1);
}

/**
 * @brief	����IIC��ʼ�ź�
 *
 * @param   void
 *
 * @return  void
 */
void IIC_Start(void)
{
    SDA_OUT();     //sda�����
    IIC_SDA(1);
    IIC_SCL(1);
    delay_us(4);
    IIC_SDA(0);//START:when CLK is high,DATA change form high to low
	delay_us(4);
    IIC_SCL(0);//ǯסI2C���ߣ�׼�����ͻ��������
}
/**
 * @brief	����IICֹͣ�ź�
 *
 * @param   void
 *
 * @return  void
 */
void IIC_Stop(void)
{
    SDA_OUT();//sda�����
	IIC_SDA(0);
    IIC_SCL(1);
    delay_us(4);
	IIC_SDA(1);//STOP:when CLK is high DATA change form low to high
	delay_us(4);
	IIC_SCL(0);//����I2C���߽����ź�
}

/**
 * @brief	�ȴ�Ӧ���źŵ���
 *
 * @param   void
 *
 * @return  u8		1������Ӧ��ʧ��
 *					0������Ӧ��ɹ�
 */
u8 IIC_Wait_Ack(void)
{
    u8 ucErrTime = 0;
    SDA_IN();      //SDA����Ϊ����
    IIC_SDA(1);
    delay_us(2);
    IIC_SCL(1);
    delay_us(2);

    while(READ_SDA)
    {
        ucErrTime++;

        if(ucErrTime > 250)
        {
            IIC_Stop();
            return 1;
        }
    }

    IIC_SCL(0);//ʱ�����0
    return 0;
}
/**
 * @brief	����ACKӦ��
 *
 * @param   void
 *
 * @return  void
 */
void IIC_Ack(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(0);
    delay_us(2);
    IIC_SCL(1);
    delay_us(2);
    IIC_SCL(0);
}
/**
 * @brief	������ACKӦ��
 *
 * @param   void
 *
 * @return  void
 */
void IIC_NAck(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(1);
    delay_us(2);
    IIC_SCL(1);
    delay_us(2);
    IIC_SCL(0);
}
/**
 * @brief	IIC����һ���ֽ�
 *
 * @param   txd		��Ҫ���͵�����
 *
 * @return  void
 */
void IIC_Send_Byte(u8 txd)
{
    u8 t;
    SDA_OUT();
    IIC_SCL(0);//����ʱ�ӿ�ʼ���ݴ���

    for(t = 0; t < 8; t++)
    {
        IIC_SDA((txd & 0x80) >> 7);
        txd <<= 1;
        IIC_SCL(1);
        delay_us(2);
        IIC_SCL(0);
        delay_us(2);
    }
}
/**
 * @brief	��1���ֽ�����
 *
 * @param   ack		1������ACK		0������nACK
 *
 * @return  u8		���ض�ȡ����
 */
u8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN();//SDA����Ϊ����

    for(i = 0; i < 8; i++)
    {
        IIC_SCL(0);
        delay_us(2);
        IIC_SCL(1);
        receive <<= 1;
        if(READ_SDA)receive++;
        delay_us(2);
    }

    if(!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK

    return receive;
}


