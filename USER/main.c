#include "sys.h"
#include "usart.h"
#include "usmart.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "malloc.h"
#include "w25qxx.h"
#include "sd_card.h"
#include "ff.h"
#include "exfuns.h"
#include "icm20608.h"
#include "inv_icm20608.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/**
 * @brief	����1����1���ֽ�
 *
 * @param   c	Ҫ���͵�����
 *
 * @return  void
 */
void usart1_send_char(u8 c)
{
    while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)==RESET); 
    USART1->TDR = c;
}
/**
 * @brief	�������ݸ�����������λ�����(V2.6�汾)
 *
 * @param  fun		������. 0X01~0X1C
 * @param  data		���ݻ�����,���28�ֽ�!!
 * @param  len		data����Ч���ݸ���
 *
 * @return  void
 */
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;

	if(len>28)return;	//���28�ֽ����� 
	send_buf[len+4]=0;	//У��������
	send_buf[0]=0XAA;	//֡ͷ
	send_buf[1]=0XAA;	//֡ͷ
	send_buf[2]=fun;	//������
	send_buf[3]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[4+i]=data[i];			//��������
	for(i=0;i<len+4;i++)send_buf[4+len]+=send_buf[i];	//����У���
	for(i=0;i<len+5;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1 
}
/**
 * @brief	���ͼ��ٶȴ���������+����������(������֡)
 *
 * @param  aacx,aacy,aacz		x,y,z������������ļ��ٶ�ֵ
 * @param  gyrox,gyroy,gyroz	x,y,z�������������������ֵ 
 *
 * @return  void
 */
void icm20608_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[18]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	tbuf[12]=0;//������ȫ����0����
	tbuf[13]=0;
	tbuf[14]=0;
	tbuf[15]=0;
	tbuf[16]=0;
	tbuf[17]=0;
	usart1_niming_report(0X02,tbuf,18);//������֡,0X02
}
/**
 * @brief	ͨ������1�ϱ���������̬���ݸ�����(״̬֡)
 *
 * @param  roll		�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
 * @param  yaw		�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
 * @param  csb		�������߶�,��λ:cm
 * @param  prs		��ѹ�Ƹ߶�,��λ:mm
 *
 * @return  void
 */
void usart1_report_imu(short roll,short pitch,short yaw)
{
	u8 tbuf[12];   	
	tbuf[0]=(roll>>8)&0XFF;
	tbuf[1]=roll&0XFF;
	tbuf[2]=(pitch>>8)&0XFF;
	tbuf[3]=pitch&0XFF;
	tbuf[4]=(yaw>>8)&0XFF;
	tbuf[5]=yaw&0XFF;
	tbuf[6]=0;
	tbuf[7]=0;
	tbuf[8]=0;
	tbuf[9]=0;
	tbuf[10]=0;
	tbuf[11]=0;
	usart1_niming_report(0X01,tbuf,12);//״̬֡,0X01
}  


int main(void)
{
    u8 t=0,report=1;			//Ĭ�Ͽ����ϱ�

		float pitch,roll,yaw; 		//ŷ����
		short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
		short gyrox,gyroy,gyroz;	//������ԭʼ����
		short temp;					//�¶�
		short tp,tr,ty;
	
    HAL_Init();
    SystemClock_Config();		//��ʼ��ϵͳʱ��Ϊ80M
    delay_init(80); 			//��ʼ����ʱ����    80Mϵͳʱ��
    uart_init(115200);			//��ʼ�����ڣ�������Ϊ500000

    //LED_Init();					//��ʼ��LED
	
		ICM20608_Init();			//��ʼ��ICM20608	
	
	
		FIL  fp;
		UINT bw;


	
	
		W25QXX_Init();
		my_mem_init(SRAM1);
		my_mem_init(SRAM2);
		SD_Init();
		exfuns_init();
		u8 res=0;
    f_mount(fs[0], "0:", 1); 					//����SD��
    res = f_mount(fs[1], "1:", 1); 				//����FLASH.

    if(res == 0X0D) //FLASH����,FAT�ļ�ϵͳ����,���¸�ʽ��FLASH
    {     
        res = f_mkfs("1:", 1, 4096); //��ʽ��FLASH,1,�̷�;1,����Ҫ������,8������Ϊ1����
        if(res == 0)
        {
            f_setlabel((const TCHAR *)"1:ALIENTEK");	//����Flash���̵�����Ϊ��ALIENTEK
        }
        delay_ms(1000);
    }
	


	char str[30];
	while(mpu_mpl_init())
 	{
 		delay_ms(200);
	}

	while(1)
    { 

        if(mpu_mpl_get_data(&pitch,&roll,&yaw)==0)
        {
 
		    MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
		    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
				icm20608_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���ͼ��ٶ�+������ԭʼ����
				usart1_report_imu((int)(roll*100),(int)(pitch*100),(int)(yaw*100));	//������̬��
			if((t%10)==0)
		    { 
				f_open(&fp,"0:data.txt",FA_WRITE|FA_READ);
				f_lseek(&fp,f_size(&fp));

				t=0;
				//LED_B_TogglePin;//DS0��˸ 
			
				sprintf(str,"pitch:%d row:%d yaw:%d\r\n\0",(int)pitch,(int)roll,(int)yaw);
			
				f_write(&fp,str,strlen(str),&bw);
				f_close(&fp);
		 	}
		 }
        t++;
		delay_ms(30);
	 } 	    	
}






