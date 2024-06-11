#include "ads131.h"
#include "drivers/spi.h"
#include "stm32f20x_40x_spi.h"
#include "stm32f10x_spi.h"
#include "string.h"
#include "globalval.h"

void  ads_write_command(u16 command);

int pd_v;
int V_1248[4]={0};
int ret00=0;
long yun_suan_v[4]={0};
int yuan_1247v[4][10]={0};
extern volatile int	hard_ver;	//0:使用1248版本	1：仅使用ADS131B04	2：使用ADS131B04和DAC8568

#define	S_SPI_DELAY 100//15//10//对于72M时钟，时钟分频值设置为1098（对于36M使用549）

int ADS131_CS1_HIGH()
{
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	return 1;
}
int ADS131_CS1_LOW()
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	return 1;
}
#define	MS_PORT		GPIOB
#define	MS_CLK_PIN		13
#define	MS_DOUT_PIN		14
#define	MS_DIN_PIN		15
#define	MS_CS_PIN		12
	
#define	MS_PORT_RESET		GPIOD
#define MS_RESET_PIN     5  //PD0
	
#define	S_SPI_CSL	MS_PORT->BRR = (1 << MS_CS_PIN)
#define	S_SPI_CSH	MS_PORT->BSRR = (1 << MS_CS_PIN)
		
#define	S_SPI_CLKL	MS_PORT->BRR = (1 << MS_CLK_PIN)
#define	S_SPI_CLKH	MS_PORT->BSRR = (1 << MS_CLK_PIN)
		
#define	S_SPI_MOSIL	MS_PORT->BRR = (1 << MS_DIN_PIN)
#define	S_SPI_MOSIH	MS_PORT->BSRR = (1 << MS_DIN_PIN)
		
#define	S_SPI_MISOH	MS_PORT->BSRR = (1 << MS_DOUT_PIN)
		
#define	S_SPI_MISO	((MS_PORT->IDR & (1 << MS_DOUT_PIN)) != 0)
	
	
#define ADS_CS_0  S_SPI_CSL
#define ADS_CS_1  S_SPI_CSH
	
#define ADS_RESET_1  GPIOD->BSRR = (1 << MS_RESET_PIN)
#define ADS_RESET_0  GPIOD->BRR = (1 << MS_RESET_PIN)
	
	
u8 SPI_ADS131_SendByte(unsigned char in)
{
	int bi = 8;
	int dli = 0;

	if (1)
	{
		S_SPI_CLKL;
		dli = S_SPI_DELAY;
		while (dli--);
		S_SPI_CSL;
		
		S_SPI_CLKH;
		S_SPI_CSL;
		
		while (bi--)
		{
			S_SPI_CLKH;//时钟上升沿，传感器读取数据
			dli = S_SPI_DELAY;
			
			while (dli--);
			if (in & 0x80)
			{
				S_SPI_MOSIH;
			}
			else
			{
				S_SPI_MOSIL;
			}
			in <<= 1;
			dli = S_SPI_DELAY;
			while (dli--);
			S_SPI_CLKL;//时钟下降沿，MCU改变数据
//			dli = S_SPI_DELAY;
//			while (dli--);
		}
		S_SPI_MOSIH;
	}
	return 0;
}

u8 SPI_ADS131_ReadByte()
{
   int bi = 8;
   int dli = 0;
   int out = -1;
   out = 0;
   if (1)
   {
	   for (bi = 0; bi < 8; bi++)
	   {
		   S_SPI_CLKH;//时钟上升沿，传感器改变数据
		   
		   out <<= 1;
		   dli = S_SPI_DELAY;
		   while (dli--);
		   
		   S_SPI_CLKL;//时钟下降沿，MCU读取数据
	   
		   dli = S_SPI_DELAY;
		   while (dli--);
		   
		   if (S_SPI_MISO)
		   {
			   out |= 1;
		   }
		   dli = S_SPI_DELAY;
		   while (dli--);
		   
	   }
   }
   return (out);
}
    
void DRDY131_Interrupt(void);
s32	data_read[8];


void DRDY131_Interrupt(void)
{

	
	EXTI_InitTypeDef exti_pin;
	NVIC_InitTypeDef NVIC_Struct;												//DRDY_131
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);		//DRDY_131
	exti_pin.EXTI_Line = EXTI_Line0;
	exti_pin.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_pin.EXTI_Trigger = EXTI_Trigger_Falling;
	exti_pin.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_pin);				//初始化外部中断0


	NVIC_Struct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Struct);
}

u32 data11=0;
u16 ads_read_reg2(u16 address);

//都是高字节在前
//写入寄存器也是高字节在前  低字节在后
//读出来数据也是高字节在前  低字节在后
void ads_write_reg(u16 address ,u16 data)
{
	int test=0;
	S_SPI_CLKL;
	rt_thread_delay(1);
	S_SPI_CSL;
	test=WREG_131(address,0);
	SPI_ADS131_SendByte((test>>8)&0xff);	
	SPI_ADS131_SendByte(test&0xff); 
	SPI_ADS131_SendByte(0x00);	
	
	SPI_ADS131_SendByte((data>>8)&0xff);
	SPI_ADS131_SendByte(data&0xff);	
	SPI_ADS131_SendByte(0x00);
	
	SPI_ADS131_SendByte(0x00);			
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);			
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);			
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);			
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	S_SPI_CSH;
	rt_thread_delay(10);
}

u16 ads_read_reg(u16 address)
{
	int test=0;
	S_SPI_CLKL;
	rt_thread_delay(1);
	S_SPI_CSL;
	test=RREG_131(address,0);
	SPI_ADS131_SendByte((test>>8)&0xff);	
	SPI_ADS131_SendByte(test&0xff); 
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);			
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);			
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);			
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);			
	S_SPI_CSH;
	rt_thread_delay(10);
	S_SPI_CSL;
	
	while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)==0);
	data11=SPI_ADS131_ReadByte();
	data11=data11<<8;
	data11+=SPI_ADS131_ReadByte();
	rt_kprintf("read reg %04x ======%04x\n",address,data11);
	rt_thread_delay(10);
	S_SPI_CSH;
	return (data11&0xffff);
}
u32 data22=0;
u32 data33=0;
u32 data44=0;
u32 data00=0;
u32 crc00=0;
int cur_channel=0;

int channel_delay=0;
int real_channel_vA[8][20]={0};
int real_channel_vB[8][20]={0};
int real_channel_vC[8][20]={0};

u16 ads_read_data()
{
	int test=0;
	float rest_v[4]=0;
	int i=0;
	
//	while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6));
//	S_SPI_CLKL;
//	rt_thread_delay(1);
//	S_SPI_CSL;

	channel_delay++;
	if(channel_delay>9)
	{
		cur_channel++;
		channel_delay=0;
		if(cur_channel>6)
			cur_channel=0;
	}
//	set_channel_enable(cur_channel+1); 
//先直接测试通道6
	set_channel_enable(6);

	ads_write_command(0);
	rt_thread_delay(10);
	S_SPI_CSL;	
	while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)==0);
	data00=SPI_ADS131_ReadByte();
	data00=data00<<16;
	data00+=(SPI_ADS131_ReadByte()<<8);
	data00+=(SPI_ADS131_ReadByte());
//	rt_kprintf("00 status read data ======%06x \n",data00);

//	while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)==0);
	data11=SPI_ADS131_ReadByte();
	data11=data11<<16;
	data11+=(SPI_ADS131_ReadByte()<<8);
	data11+=(SPI_ADS131_ReadByte());
	
	rest_v[0]=(float)data11/8388608*1200;
//	rt_kprintf("01read data ======%06x %d\n",data11,(int)rest_v);
//	while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)==0);
	data22=SPI_ADS131_ReadByte();
	data22=data22<<16;
	data22+=(SPI_ADS131_ReadByte()<<8);
	data22+=(SPI_ADS131_ReadByte());
	if(data22>8388608)
		data22-=8388608;
	rest_v[1]=(float)data22/8388608*1200;
//	rt_kprintf("02read data ======%06x %d\n",data22,(int)rest_v);
//	while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)==0);
	data33=SPI_ADS131_ReadByte();
	data33=data33<<16;
	data33+=(SPI_ADS131_ReadByte()<<8);
	data33+=(SPI_ADS131_ReadByte());
	if(data33>8388608)
		data33-=8388608;
	
	rest_v[2]=(float)data33/8388608*1200;
//	rt_kprintf("03read data ======%06x %d\n",data33,(int)rest_v);
//	while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)==0);
	data44=SPI_ADS131_ReadByte();
	data44=data44<<16;
	data44+=(SPI_ADS131_ReadByte()<<8);
	data44+=(SPI_ADS131_ReadByte());
	if(data44>8388608)
		data44-=8388608;
	rest_v[3]=(float)data44/8388608*1200;

	crc00=SPI_ADS131_ReadByte();
	crc00=crc00<<16;
	crc00+=(SPI_ADS131_ReadByte()<<8);
	crc00+=SPI_ADS131_ReadByte();
	
	real_channel_vA[cur_channel][channel_delay]=rest_v[0];
	real_channel_vB[cur_channel][channel_delay]=rest_v[1];
	real_channel_vC[cur_channel][channel_delay]=rest_v[2];
	HC_ro_data.data.A0_N1[cur_channel]=(int)rest_v[0];
	
	S_SPI_CSH;
	rt_kprintf("00 status read data ======%06x \n",data00);
	rt_kprintf("01read data ======%06x %d\n",data11,(int)rest_v[0]);
	rt_kprintf("02read data ======%06x %d\n",data22,(int)rest_v[1]);
	rt_kprintf("03read data ======%06x %d\n",data33,(int)rest_v[2]);
	rt_kprintf("04read data ======%06x %d\n",data44,(int)rest_v[3]);	
//	rt_kprintf("crc read data ======%06x\n",crc00);	
	return (data11&0xffff);
}
void  ads_write_command(u16 command)
{
	S_SPI_CLKL;
	rt_thread_delay(1);
	S_SPI_CSL;
	SPI_ADS131_SendByte((command>>8)&0xff);	
	SPI_ADS131_SendByte(command&0xff); 
	SPI_ADS131_SendByte(0x00);
	
	SPI_ADS131_SendByte(0x00);
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);
	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);
	SPI_ADS131_SendByte(0x00);
	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);
	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	
	SPI_ADS131_SendByte(0x00);
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	
	S_SPI_CSH;
	rt_thread_delay(10);
	

}
u16  ads_write_command_get_response(u16 command)
{
	S_SPI_CLKL;
	rt_thread_delay(1);
	S_SPI_CSL;
	SPI_ADS131_SendByte((command>>8)&0xff);	
	SPI_ADS131_SendByte(command&0xff); 
	SPI_ADS131_SendByte(0x00);	
	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);
	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);
	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	SPI_ADS131_SendByte(0x00);	
	
	S_SPI_CSH;
	rt_thread_delay(10);

	S_SPI_CSL;
	while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)==0);
	data11=SPI_ADS131_ReadByte();
	data11=data11<<8;
	data11+=SPI_ADS131_ReadByte();
	rt_kprintf("command %04x return ======%04x\n",command,data11);
	S_SPI_CSH;
	rt_thread_delay(10);
	return (data11&0xffff);

}
#define STAND_BY 0x22
#define WAKE_UP 0x33
#define RESET 0x11

void ads131_reset()
{
//	ads_write_reg(3,0x0f9d);
//	ads_read_reg(3);
	ads_write_command_get_response(RESET);
	ads_write_reg(3,0x0f9d);
	ads_read_reg(3);
	ads_write_command_get_response(STAND_BY);
	ads_write_reg(3,0x0f1d);
	ads_read_reg(3);
	ads_write_reg(02,0x0100);
	ads_read_reg(2);
	ads_read_reg(0);
//	ads_write_command_get_response(0x555);
//	ads_write_reg(02,0x0110);
	ads_read_reg(2);
	
//	ads_write_reg(3,0x0f1d);
//	ads_read_reg(3);
	ads_read_reg(3);
	ads_write_reg(6,0x0d00);
	ads_read_reg(6);
	ads_write_reg(02,0x0110);
	ads_read_reg(2);
//	ads_read_reg2(6);
//	ads_read_reg2(2);
	
//	ads_write_reg(02,0x0110);
//	ads_write_reg(02,0x0110);
//		ads_read_reg(2);

	ads_write_command_get_response(WAKE_UP);

}

	/******************ads131管脚与MCU对应关系*******************
	SCK----------Pb13
	MISO---------Pb14
	MOSI---------Pb15							(SPI)
	CS-----------Pb12
	RESET--------PD5
	DRDY---------PD6
	******************ads1247管脚与MCU对应关系*******************/
void ADS131_SPI_Configuration()
{
 
//  	SPI_InitTypeDef SPI_InitStruct;  
  	GPIO_InitTypeDef GPIO_InitStructure;
	 
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD,ENABLE); 	 //SPI的SCK、MISO、MOSI都是PB
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); 
   
   //CLK
	   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    
	   GPIO_Init(GPIOB, &GPIO_InitStructure);
	   
   //drdy   pd6
	   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    
	   GPIO_Init(GPIOD, &GPIO_InitStructure);
   //DOUT  pb14
	   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  
	   GPIO_Init(GPIOB, &GPIO_InitStructure);
   // DIN pb15  cs  pb12
	   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15|GPIO_Pin_12;
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	   GPIO_Init(GPIOB, &GPIO_InitStructure);

// reset  
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		
		GPIO_SetBits(GPIOD,GPIO_Pin_5);
		
}
	
	

static void ADS131_process(void* parameter)
{
	ADS131_SPI_Configuration();

	rt_thread_delay(60);
//	ADS_CS_0;	 //CS=0
//	ADS_RESET_1;   
//	rt_thread_delay(60);
//	ADS_CS_1;	 //CS=0
//	rt_thread_delay(60);
	ads131_reset();
	while(1)
	{
		ads_read_data();
		rt_thread_delay(1000);	
	}
	
}

void ADS131_init(void)
{
		rt_thread_t thread;

	thread = rt_thread_create("131",		  
							  ADS131_process, RT_NULL,
							  1024,
							  THREAD_1247_PRIORITY, 50);
	if (thread != RT_NULL)
		rt_thread_startup(thread);
	
}

	 

