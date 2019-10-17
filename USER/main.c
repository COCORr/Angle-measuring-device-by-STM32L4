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
 * @brief	串口1发送1个字节
 *
 * @param   c	要发送的数据
 *
 * @return  void
 */
void usart1_send_char(u8 c)
{
    while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)==RESET); 
    USART1->TDR = c;
}
/**
 * @brief	传送数据给匿名四轴上位机软件(V2.6版本)
 *
 * @param  fun		功能字. 0X01~0X1C
 * @param  data		数据缓存区,最多28字节!!
 * @param  len		data区有效数据个数
 *
 * @return  void
 */
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;

	if(len>28)return;	//最多28字节数据 
	send_buf[len+4]=0;	//校验数置零
	send_buf[0]=0XAA;	//帧头
	send_buf[1]=0XAA;	//帧头
	send_buf[2]=fun;	//功能字
	send_buf[3]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[4+i]=data[i];			//复制数据
	for(i=0;i<len+4;i++)send_buf[4+len]+=send_buf[i];	//计算校验和
	for(i=0;i<len+5;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
}
/**
 * @brief	发送加速度传感器数据+陀螺仪数据(传感器帧)
 *
 * @param  aacx,aacy,aacz		x,y,z三个方向上面的加速度值
 * @param  gyrox,gyroy,gyroz	x,y,z三个方向上面的陀螺仪值 
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
	tbuf[12]=0;//无数据全部用0代替
	tbuf[13]=0;
	tbuf[14]=0;
	tbuf[15]=0;
	tbuf[16]=0;
	tbuf[17]=0;
	usart1_niming_report(0X02,tbuf,18);//传感器帧,0X02
}
/**
 * @brief	通过串口1上报结算后的姿态数据给电脑(状态帧)
 *
 * @param  roll		横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
 * @param  yaw		航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
 * @param  csb		超声波高度,单位:cm
 * @param  prs		气压计高度,单位:mm
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
	usart1_niming_report(0X01,tbuf,12);//状态帧,0X01
}  


int main(void)
{
    u8 t=0,report=1;			//默认开启上报

		float pitch,roll,yaw; 		//欧拉角
		short aacx,aacy,aacz;		//加速度传感器原始数据
		short gyrox,gyroy,gyroz;	//陀螺仪原始数据
		short temp;					//温度
		short tp,tr,ty;
	
    HAL_Init();
    SystemClock_Config();		//初始化系统时钟为80M
    delay_init(80); 			//初始化延时函数    80M系统时钟
    uart_init(115200);			//初始化串口，波特率为500000

    //LED_Init();					//初始化LED
	
		ICM20608_Init();			//初始化ICM20608	
	
	
		FIL  fp;
		UINT bw;


	
	
		W25QXX_Init();
		my_mem_init(SRAM1);
		my_mem_init(SRAM2);
		SD_Init();
		exfuns_init();
		u8 res=0;
    f_mount(fs[0], "0:", 1); 					//挂载SD卡
    res = f_mount(fs[1], "1:", 1); 				//挂载FLASH.

    if(res == 0X0D) //FLASH磁盘,FAT文件系统错误,重新格式化FLASH
    {     
        res = f_mkfs("1:", 1, 4096); //格式化FLASH,1,盘符;1,不需要引导区,8个扇区为1个簇
        if(res == 0)
        {
            f_setlabel((const TCHAR *)"1:ALIENTEK");	//设置Flash磁盘的名字为：ALIENTEK
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
 
		    MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
				icm20608_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//发送加速度+陀螺仪原始数据
				usart1_report_imu((int)(roll*100),(int)(pitch*100),(int)(yaw*100));	//发送姿态角
			if((t%10)==0)
		    { 
				f_open(&fp,"0:data.txt",FA_WRITE|FA_READ);
				f_lseek(&fp,f_size(&fp));

				t=0;
				//LED_B_TogglePin;//DS0闪烁 
			
				sprintf(str,"pitch:%d row:%d yaw:%d\r\n\0",(int)pitch,(int)roll,(int)yaw);
			
				f_write(&fp,str,strlen(str),&bw);
				f_close(&fp);
		 	}
		 }
        t++;
		delay_ms(30);
	 } 	    	
}






