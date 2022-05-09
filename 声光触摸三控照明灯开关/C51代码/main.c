/**************************************功能说明*************************************
*
* 因为需要用到ADC 而stc89c51系列没有内置ADC 因此选用了STC15F2K60S2
* 该芯片和STC89C51引脚外观都完全一致 不同的是内部资源 可以替换c51
*
* 声音传感器使用的是开关量 接入单片机的一个引脚 有声音的时候值为0
* 光线传感器经过电路处理之后 是一个连续的模拟量 用ADC 通道0来捕获
* 触摸传感器是一个金属片经过300KZH的脉冲信号之后产生的震荡 当人体
* 摸到金属片之后电容改变 另一端的电压继而改变 用ADC通道1捕获
* ADC的值经过转化得到连续值 与TouchZero进行比较判断触发或者未触发
*  
* MainHandle函数里面进行逻辑判断
*
* STC15F2K60S2的ADC做电容感应触摸键.
* 芯片的工作频率为24MHz
*
***********************************************************************************/

#include <reg52.h>
#include <intrins.h>

#define MAIN_Fosc 24000000UL //定义主时钟频率 24M

typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;

#define Timer0_Reload (65536UL - (MAIN_Fosc / 600000)) //Timer 0 重装值 产生300KHZ的波 用于震荡导电片

sfr P1ASF = 0x9D;	  //只写，模拟输入选择
sfr ADC_CONTR = 0xBC; //带AD系列
sfr ADC_RES = 0xBD;	  //带AD系列
sfr ADC_RESL = 0xBE;  //带AD系列
sfr AUXR = 0x8E;
sfr AUXR2 = 0x8F;

/*************	本地常量声明	**************/

#define TOUCH_CHANNEL 2 //ADC通道数

#define ADC_90T (3 << 5)   //ADC时间 90T
#define ADC_180T (2 << 5)  //ADC时间 180T
#define ADC_360T (1 << 5)  //ADC时间 360T
#define ADC_540T 0		   //ADC时间 540T
#define ADC_FLAG (1 << 4)  //软件清0
#define ADC_START (1 << 3) //自动清0

/*************	本地变量声明	**************/

sbit P_LED = P2 ^ 4;   //LED接口
sbit P_SOUND = P2 ^ 7; //声音传感器接口
sbit P_LIGHT = P2 ^ 6; //光敏电阻接口
sbit P_TOUCH = P2 ^ 5; //触摸传感器接口

u16 idata adc[TOUCH_CHANNEL];		  //当前ADC值
u16 idata adc_prev[TOUCH_CHANNEL];	  //上一个ADC值
u16 idata TouchZero[TOUCH_CHANNEL];	  //0点ADC值
u8 idata TouchZeroCnt[TOUCH_CHANNEL]; //0点自动跟踪计数

u16 Counter1;			 //定时器1计数
u16 timerPeriod1 = 10000; //2000ms
u8 cnt_250ms;			 //250ms 一个周期的周期计数

u8 touch_button_state = 0; //触摸按键的状态 0未触发  1触发
u8 light_sensor_state = 0; //光线传感器的状态 0未触发  1触发
u8 voice_sensor_state = 0; //声音传感器的状态 0未触发  1触发

/*************	本地函数声明	**************/
void delay_ms(u8 ms);
void ADC_init(void);
u16 Get_ADC10bitResult(u8 channel);
void AutoZero(void);
u8 check_adc(u8 index);
void MainHandle(void);

/******************** 主函数 **************************/
void main(void)
{
	//u8 i;

	delay_ms(50);

	ET0 = 0; //初始化Timer0输出一个300KHZ时钟
	TR0 = 0;
	AUXR |= 0x80;  //Timer0 set as 1T mode
	AUXR2 |= 0x01; //允许输出时钟
	TMOD = 0;	   //Timer0 set as Timer, 16 bits Auto Reload.
	TH0 = (u8)(Timer0_Reload >> 8);
	TL0 = (u8)Timer0_Reload;
	TR0 = 1;

	TH1 = (65536 - 1000) / 256; // 设置定时器1定时时间1ms
	TH1 = (65536 - 1000) % 256;
	TR1 = 1; // 设置控制寄存器：启动定时器1
	ET1 = 0; // 设置中断允许寄存器
	EA = 1;

	//ADC_init();	  //ADC初始化
	delay_ms(50); //延时50ms

	//通道0 被用来检测光敏电阻的电阻值 TouchZero 改变灵敏度
	adc_prev[0] = 1023;
	TouchZero[0] = 1023;
	TouchZeroCnt[0] = 0;
	//通道1 被用来检测光敏电阻的电阻值 TouchZero 改变灵敏度
	adc_prev[1] = 1023;
	TouchZero[1] = 1023;
	TouchZeroCnt[1] = 0;

	cnt_250ms = 0;

	while (1)
	{
		delay_ms(1); //每隔50ms处理一次按键
		MainHandle(); //主处理函数 逻辑部分在这里处理
//		if (++cnt_250ms >= 5)
//		{
//			cnt_250ms = 0;
//			AutoZero(); //每隔250ms处理一次0点自动跟踪
//		}
	}
}
/**********************************************/

//========================================================================
// 函数: void  delay_ms(unsigned char ms)
// 描述: 延时函数。
// 参数: ms,要延时的ms数, 这里只支持1~255ms. 自动适应主时钟.
// 返回: none.
// 版本: VER1.0
// 日期: 2013-4-1
// 备注:
//========================================================================
void delay_ms(u8 ms)
{
	unsigned int i;
	do
	{
		i = MAIN_Fosc / 13000;
		while (--i)
			;
	} while (--ms);
}

/*************  ADC初始化函数 *****************/
void ADC_init(void)
{
	P1ASF = 0xff;	  //8路ADC
	ADC_CONTR = 0x80; //允许ADC
}

//========================================================================
// 函数: u16	Get_ADC10bitResult(u8 channel)
// 描述: 查询法读一次ADC结果.
// 参数: channel: 选择要转换的ADC.
// 返回: 10位ADC结果.
// 版本: V1.0, 2012-10-22
//========================================================================
u16 Get_ADC10bitResult(u8 channel) //channel = 0~7
{
	ADC_RES = 0;
	ADC_RESL = 0;
	ADC_CONTR = 0x80 | ADC_90T | ADC_START | channel; //触发ADC
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	while ((ADC_CONTR & ADC_FLAG) == 0)
		;												//等待ADC转换结束
	ADC_CONTR = 0x80;									//清除标志
	return (((u16)ADC_RES << 2) | ((u16)ADC_RESL & 3)); //返回ADC结果
}

/********************* 自动0点跟踪函数 *************************/
void AutoZero(void) //250ms调用一次 这是使用相邻2个采样的差的绝对值之和来检测。
{
	u8 i;
	u16 j, k;

	for (i = 0; i < TOUCH_CHANNEL; i++) //处理8个通道
	{
		j = adc[i];
		k = j - adc_prev[i]; //减前一个读数
		F0 = 0;				 //按下
		if (k & 0x8000)
			F0 = 1, k = 0 - k; //释放	求出两次采样的差值
		if (k >= 20)		   //变化比较大
		{
			TouchZeroCnt[i] = 0; //如果变化比较大，则清0计数器
			if (F0)
				TouchZero[i] = j; //如果是释放，并且变化比较大，则直接替代
		}
		else //变化比较小，则蠕动，自动0点跟踪
		{
			if (++TouchZeroCnt[i] >= 20) //连续检测到小变化20次/4 = 5秒.
			{
				TouchZeroCnt[i] = 0;
				TouchZero[i] = adc_prev[i]; //变化缓慢的值作为0点
			}
		}
		adc_prev[i] = j; //保存这一次的采样值
	}
}

/********************* 获取触摸信息函数 50ms调用1次 *************************/
u8 check_adc(u8 index) //判断键按下或释放,有回差控制
{
	u16 delta;
	adc[index] = 1023 - Get_ADC10bitResult(index); //获取ADC值, 转成按下键, ADC值增加
	if (adc[index] < TouchZero[index])
		return 0; //比0点还小的值，则认为是键释放
	delta = adc[index] - TouchZero[index];
	if (delta >= 40)
		return 1; //键按下
	if (delta <= 20)
		return 0; //键释放
	return 2;	  //保持原状态
}

/********************* 键处理 50ms调用1次 *************************/
void MainHandle(void)
{
/*进行逻辑判断*/
if (P_LIGHT == 1) //当没有光照的时候
{
	if (P_SOUND == 0) //有声音
	{
		//timerPeriod1 = 2000; //自动触发的时候 灯亮2S
		P_LED = 0;			 //开灯
		ET1 = 1;			 //开启定时器1 延时2S
	}
}
else //当有光照的时候
{
	//无论有无声响都不点亮
}

if (P_TOUCH == 1) //只要触摸传感器触发
{
	//timerPeriod1 = 10000; //触摸开关触发 灯亮4S
	P_LED = 0;			 //开灯
	ET1 = 1;			 //开启定时器1 中断允许寄存器
}
	
////	i = check_adc(0);
//	i = P_TOUCH;
//	if (i == 0)
//		touch_button_state = 0; //触摸按键的状态 0未触发  1触发
//	if (i == 1)
//		touch_button_state = 1; //触摸按键的状态 0未触发  1触发

////	i = check_adc(1);
//	i = P_LIGHT;
//	if (i == 0)
//		light_sensor_state = 0; //光线传感器的状态 0未触发  1触发
//	if (i == 1)
//		light_sensor_state = 1; //光线传感器的状态 0未触发  1触发

//	if (P_SOUND == 0)
//		voice_sensor_state = 1; //声音传感器的状态 0未触发  1触发
//	if (P_SOUND == 1)
//		voice_sensor_state = 0; //声音传感器的状态 0未触发  1触发

//	/*进行逻辑判断*/
//	if (light_sensor_state == 0) //当没有光照的时候
//	{
//		if (voice_sensor_state == 1) //有声音
//		{
//			timerPeriod1 = 2000; //自动触发的时候 灯亮2S
//			P_LED = 0;			 //开灯
//			ET1 = 1;			 //开启定时器1 延时2S
//		}
//	}
//	else //当有光照的时候
//	{
//		//无论有无声响都不点亮
//	}

//	if (touch_button_state == 1) //只要触摸传感器触发
//	{
//		timerPeriod1 = 4000; //触摸开关触发 灯亮4S
//		P_LED = 0;			 //开灯
//		ET1 = 1;			 //开启定时器1 中断允许寄存器
//	}
}

void Timer1() interrupt 3 //定时器1中断
{
	TH1 = (65536 - 1000) / 256;
	TL1 = (65536 - 1000) % 256;
	Counter1++;
	if (Counter1 == timerPeriod1)
	{
		Counter1 = 0;
		P_LED = 1; //把灯关掉
		ET1 = 0;   //定时时间到触发一次 之后关掉定时器 等下一次开启
	}
}