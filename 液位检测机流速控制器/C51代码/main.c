/**********************************************************************************

* 开机的状态
* 1. 选择总液体容量(ml) 这时数码管四位数显示总液体容量 
*    按键功能 key1 +50(ml) : key2 -50(ml) : key3 下一步
* 2. 选择滴系数 滴系数有10、15、20(滴/ml)3种型号 数码管四位数显示10/15/20 
*    按键功能 key1 +5 : key2 -5 : key3 下一步
* 3. 正常输液状态 前两段数码管显示液体流速 后两段数码管显示已完成百分比进度
*    按键功能 设定的液体流速 key1 +5 : key2 -5 : key3 下一步 这时重新回到第一个状态
*ALARM. 累计大于十次检测到流速小于最小流速,认为液体滴完了进入报警状态 亮灯鸣笛

***********************************************************************************/

#include <reg52.h>
#include <intrins.h>
#include	<tm1638.h>

/* 定义常用变量 */
typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;
#define constrain(amt, low, high) ((amt) < (low) ? (amt = low) : ((amt) > (high) ? (amt = high) : (amt = amt)))
#define limit(amt, low, high) ((amt) < (low) ? (amt = high) : ((amt) > (high) ? (amt = low) : (amt = amt)))
#define timer0Reload 1000
#define timer1Reload 100
#define WORK_STATE_NORMAL 0
#define WORK_STATE_ALARM 1
#define PID_OUTPUT_TO_SERVO_ANGLE 1 //PID输出 转换为 舵机角度

/*********************引脚定义********************/
sbit pin_pwm = P2 ^ 7;
sbit pin_buzzer = P0 ^ 0;
sbit pin_inf = P3 ^ 1;
/* 定义全局变量 */
u8 numDisplay0 = 0;//数码管显示数字
u8 numDisplay1 = 0;//数码管显示数字
u8 numDisplay2 = 0;//数码管显示数字
u8 numDisplay3 = 0;//数码管显示数字
u8 numDisplay4 = 0;//数码管显示数字
u8 numDisplay5 = 0;//数码管显示数字
u8 numDisplay6 = 0;//数码管显示数字
u8 numDisplay7 = 0;//数码管显示数字

u8 servoAngle = 90;	  //舵机初始默认角度
u8 pwmHighTime = 5; //5 - 15 对应0-180度
u8 pwmPeriodCount = 200;//PWM周期 舵机的pwm周期为20ms

u8 myCase = 1; //状态机选择,选择当前的状态
u32 exInt0Count = 0 ,tempExInt0Count = 0;//总液滴滴数
u32 timeMs = 0, tempTimeMs = 0; // 由定时器的到的系统时间 单位 ms
u16 diffTimeFiltered = 0;//滤波后的两次液滴滴下间隔时间
u16 liquidVolume = 100;	  //总液体容积 ml
u16 dropCoefficient = 15; //滴系数 临床常用的输液器 滴系数有10、15、20(滴/ml)3种型号
u8 liquidProgress = 0; //表示液位 也表示输液进度
u16 flowRateSet = 0,tempFlowRateSet = 0;	//点滴的流速设定值
u16 flowRateActual = 0; //点滴的流速真实值，由传感器计算得到
u16 miniFlowRate = 10;	//最小点滴流速 小于这个值就认为点滴当前流速为0，进而报警
u16 consumeLiquid; //已消耗的液体 容量
u8 residualLiquid_precent; //剩余液体 百分比 表示当前进度
u16 residualLiquid = 0;		   //剩余液体 容量
u8 index=0,key=0;
u8 state = 0;
u16 counterTemp = 0;
u16 TempTime = 0;
u8 TrigFlag = 0;
u32 time = 0;
u32 perviousTime = 0;
u32 diffTime = 0;
u32 timeFilterTempValue = 0;
u16 exInt0CountActual;
float tempDropCoefficient = 0.0;
/*********************函数声明********************/
void delay(u16);
void Timer_Init();
void ExInt_Init();
float pid_control(long ref, long set, float Kp, float Ki, float Kd);
void servo_ctrl(flowRateActual, flowRateSet);

/*********************主函数********************/
void main()
{
	Timer_Init(); //定时器初始化
	ExInt_Init(); //外部中断初始化
	EA = 1;
	ET0 = 0;
	ET1 = 1;
	
	init_TM1638();			//初始化TM1638
	
	for(index=0;index<8;index++)
		Write_DATA(index<<1,tab[0]);			//初始化寄存器
	
	Write_allLED(0x00);
	
	while (1)
	{
		
		//写入数码管数据
		Write_DATA(0,tab[numDisplay0]);
		Write_DATA(2,tab[numDisplay1]);		
		Write_DATA(4,tab[numDisplay2]);		
		Write_DATA(6,tab[numDisplay3]);
		Write_DATA(8,tab[numDisplay4]);		
		Write_DATA(10,tab[numDisplay5]);		
		Write_DATA(12,tab[numDisplay6]);
		Write_DATA(14,tab[numDisplay7]);		
		
		//选择总液体容积 ml
		if(myCase==1)
		{
			key = 1;
			if (Read_key() == key)
			{
				while ((Read_key()==key));
				liquidVolume += 100;
				limit(liquidVolume, 0, 2000);
			}
			key = 0;
			if (Read_key() == key)
			{
				while ((Read_key()==key));
				liquidVolume -= 100;
				limit(liquidVolume, 0, 2000);
			}
			key = 3;
			if (Read_key() == key)
			{
				while ((Read_key()==key));
				liquidVolume += 50;
				limit(liquidVolume, 0, 2000);
			}
			key = 2;
			if (Read_key() == key)
			{
				while ((Read_key()==key));
				liquidVolume -= 50;
				limit(liquidVolume, 0, 2000);
			}
			key = 5;
			if (Read_key() == key)
			{
				while ((Read_key()==key));
				liquidVolume += 10;
				limit(liquidVolume, 0, 2000);
			}
			key = 4;
			if (Read_key() == key)
			{
				while ((Read_key()==key));
				liquidVolume -= 10;
				limit(liquidVolume, 0, 2000);
			}
			numDisplay0 = (liquidVolume / 1000) % 10;
			numDisplay1 = (liquidVolume / 100) % 10;
			numDisplay2 = (liquidVolume / 10) % 10;
			numDisplay3 = (liquidVolume / 1) % 10;
			numDisplay4 = 0;
			numDisplay5 = 0;
			numDisplay6 = 0;
			numDisplay7 = 1;
		}
		
		//选择滴系数 临床常用的输液器 滴系数有10、15、20(滴/ml)3种型号
		if(myCase==2)
		{
			key = 1;
			if (Read_key() == key)
			{
				while ((Read_key()==key));
				dropCoefficient += 5;
				limit(dropCoefficient, 10, 20);
			}
			key = 0;
			if (Read_key() == key)
			{
				while ((Read_key()==key));
				dropCoefficient -= 5;
				limit(dropCoefficient, 10, 20);
			}
			numDisplay0 = (dropCoefficient / 1000) % 10;
			numDisplay1 = (dropCoefficient / 100) % 10;
			numDisplay2 = (dropCoefficient / 10) % 10;
			numDisplay3 = (dropCoefficient / 1) % 10;
			numDisplay4 = 0;
			numDisplay5 = 0;
			numDisplay6 = 0;
			numDisplay7 = 2;
		}
		
		//选择点滴的流速设定值
		if(myCase==3)
		{
			key = 1;
			if (Read_key() == key)
			{
				while ((Read_key()==key));
				flowRateSet += 5;
				limit(flowRateSet, 0, 100);
			}
			key = 0;
			if (Read_key() == key)
			{
				while ((Read_key()==key));
				flowRateSet -= 5;
				limit(flowRateSet, 0, 100);
			}
			numDisplay0 = (flowRateSet / 1000) % 10;
			numDisplay1 = (flowRateSet / 100) % 10;
			numDisplay2 = (flowRateSet / 10) % 10;
			numDisplay3 = (flowRateSet / 1) % 10;
			numDisplay4 = 0;
			numDisplay5 = 0;
			numDisplay6 = 0;
			numDisplay7 = 3;
			exInt0CountActual =0;
			liquidProgress=0;
			flowRateActual=0;
			diffTime=0;
			tempFlowRateSet=flowRateSet;
		}
		
		//开启定时器进行流速控制
		if(myCase==4)
		{
			
			
			key = 1;
			if (Read_key() == key)
			{
				while ((Read_key()==key));
				flowRateSet += 5;
				limit(flowRateSet, 0, 100);
			}
			key = 0;
			if (Read_key() == key)
			{
				while ((Read_key()==key));
				flowRateSet -= 5;
				limit(flowRateSet, 0, 100);
			}
			
			numDisplay0 = ((flowRateActual) / 10) % 10;
			numDisplay1 = ((flowRateActual) / 1) % 10;
			numDisplay2 = (liquidProgress / 10) % 10;
			numDisplay3 = (liquidProgress / 1) % 10;
			numDisplay4 = (timeMs / 1000) % 10;
			numDisplay5 = (timeMs / 100) % 10;
			numDisplay6 = (timeMs / 10) % 10;
			numDisplay7 = (timeMs / 1) % 10;
			
			timeMs++;//分秒
			
			//液滴速度 每小时输入的毫升数(ml/h) = (滴/min) x 60(min/h) / 滴系数(滴/ml)
			flowRateActual = (1 * 60 / diffTime) * 60 / dropCoefficient;
			
			if (liquidProgress >= 100) 
			{
				state = WORK_STATE_ALARM;
			}
			else
			{
				if (flowRateActual <= 5) 
					state = WORK_STATE_ALARM;
				else
					state = WORK_STATE_NORMAL;
			}
			


			
		}
		
		//下一步 上一步 切换
		key = 7;
		if (Read_key() == key)
		{
			while ((Read_key()==key));
			myCase++;
			limit(myCase, 1, 4);
		}
		key = 6;
		if (Read_key() == key)
		{
			while ((Read_key()==key));
			myCase--;
			limit(myCase, 1, 4);
		}
		
		switch (state)
		{
		case WORK_STATE_NORMAL:	// 正常状态
//			//液滴速度 每小时输入的毫升数(ml/h) = (滴/min) x 60(min/h) / 滴系数(滴/ml)
//			flowRateActual = (1000 * 60 / diffTimeFiltered) * 60 / dropCoefficient;
			//剩余液体容积(ml)= 液体总容积(ml) - 已消耗的液体容积(ml)
			residualLiquid = liquidVolume - consumeLiquid;
			//输液进度(液位)= 已消耗的液体容积(ml) / 液体总容积(ml)
			liquidProgress = 100 * consumeLiquid / liquidVolume;
			//舵机控制角度PID计算 输入:液滴的真实速度 液滴的设定速度 KP KI KD  输出:舵机角度
			//servoAngle = pid_control(flowRateActual, flowRateSet, 0.5f, 0.00f, 0.0001f) * PID_OUTPUT_TO_SERVO_ANGLE;
			//由舵机控制角度，控制对输液管的压力，达到该表流速的效果
			servo_ctrl(flowRateActual, flowRateSet);
			flowRateSet=tempFlowRateSet;
			pin_buzzer = 1;	   //蜂鸣器响起
			Write_allLED(0x00);
			break;
		case WORK_STATE_ALARM: // 报警状态
			pin_buzzer = 0;	   //蜂鸣器响起
			Write_allLED(0xff);
		  flowRateSet=0;
			servo_ctrl(flowRateActual, flowRateSet);
			break;
		default:
			break;
		}
		

		
		

		
		//tempTimeDs++;
		//timeMs = tempTimeMs*10;
		
//		

//		//判断流速是否小于最小流速，如果是则记录一次
//		flowRateActual < miniFlowRate ? counterTemp++ : counterTemp--;
//		constrain(counterTemp, 0, 10);
//		//只有累计大于十次检测到流速小于最小流速才认为液体滴完了，进入报警状态整个过程持续1s
//		if (counterTemp >= 10)
//		{
//			counterTemp = 0;
//			state = WORK_STATE_ALARM;
//		}
		
	}
}

//#define PWM_GENERATE_FREQUENCY 100 //频率
#define STATE_PROCESS_FREQUENCY 10	 //频率
#define DETECT_FREQUENCY 1		 //频率

u32 t = 0;
static u32 tTime[2];
u16 Counter0;
void Timer0() interrupt 1 //定时器0中断
{
	TH0 = (65536 - timer0Reload) / 256;
	TL0 = (65536 - timer0Reload) % 256;
	timeMs++;
}

void Timer1() interrupt 3 //定时器1中断
{
	TH1 = (65536 - timer1Reload) / 256;
	TL1 = (65536 - timer1Reload) % 256;
	pwmPeriodCount++; // pwmPeriodCount 从0到200循环
	if(pwmPeriodCount>=200) pwmPeriodCount=0;
	(pwmPeriodCount > pwmHighTime) ? (pin_pwm = 0) : (pin_pwm = 1);
}

u16 CounterExint0;
/*******************外部中断**********************/
void EX_INT0() interrupt 0 //外部中断0
{
	//液滴计数，光电传感器触发一次，下降沿触发一次，计为一个液滴
	tempExInt0Count++;
	if(tempExInt0Count>=3)
	{
		perviousTime = time;
		time = timeMs;
		diffTime = time - perviousTime; //计算得到两个液滴之间的时间间隔 ms
		tempExInt0Count=0;
		exInt0CountActual++;
		//已经消耗的毫升数(ml)＝(滴) / 滴系数(滴/ml)。
		consumeLiquid = exInt0CountActual / dropCoefficient;
	}
}


// PID控制器
float pid_control(long ref, long set, float Kp, float Ki, float Kd)
{
	float max_out = 100.0f, max_iout = 100.0f;
	float out, Pout, Iout, Dout;
	float error[3], Dbuf[3];

	error[2] = error[1];
	error[1] = error[0];
	error[0] = set - ref;

	Pout = Kp * error[0];
	Iout += Ki * error[0];
	Dbuf[2] = Dbuf[1];
	Dbuf[1] = Dbuf[0];
	Dbuf[0] = (error[0] - error[1]);
	Dout = Kd * Dbuf[0];
	constrain(Iout, 1.0f, max_iout);
	out = Pout + Iout + Dout;
	constrain(out, 0, max_out);
	return (out);
}

int temp=0;
// 舵机角度控制, pwmHighTime 5-25 angle 0-180
void servo_ctrl(flowRateActual, flowRateSet)
{
	pwmHighTime = 5+flowRateSet/10;
	
//	temp = flowRateActual-flowRateSet;
//	if( (temp < -3000) ) pwmHighTime=5;
//	else if( (temp >  -3000)&&(temp < -1000) ) pwmHighTime=6;
//	else if( (temp >  -1000)&&(temp < -500) ) pwmHighTime=8;
//	else if( (temp >  -500)&&(temp < -0) ) pwmHighTime=9;
//	else if( (temp >  0)&&(temp < 500) ) pwmHighTime=10;
//	else if( (temp >  500)&&(temp < 1000) ) pwmHighTime=12;
//	else if( (temp >  1000)&&(temp < 3000) ) pwmHighTime=14;
//	else if( (temp >  3000) ) pwmHighTime=15;
//	
//	pwmHighTime = angle / 9 + 5;
//	limit(pwmHighTime,5,25);
}


void ExInt_Init()
{
	IT0 = 1; //设置中断类型 1为下降沿 0为低电平
	EX0 = 1; //使能外部中断0
	EA = 1;
	//	IT1 = 1; //设置中断类型 1为下降沿 0为低电平
	//	EX1 = 1; //使能外部中断1
}

void Timer_Init()
{
	EA = 1;						// 设置中断允许寄存器：1.打开中断总开关
	ET0 = 1;					// 设置中断允许寄存器：2.打开定时器0
	ET1 = 1;					// 设置中断允许寄存器：2.打开定时器1
	TMOD = 0x11;				// 设置工作方式寄存器TMOD：工作方式1 -> 0x01
	TH0 = (65536 - timer0Reload) / 256;	// 设置定时器0定时时间100ns
	TL0 = (65536 - timer0Reload) % 256;	//
	TH1 = (65536 - timer1Reload) / 256; // 设置定时器1定时时间1ms
	TH1 = (65536 - timer1Reload) % 256; //
	TR0 = 1;					// 设置控制寄存器：启动定时器0
	TR1 = 1;					// 设置控制寄存器：启动定时器1
}

/******************************************************
 Function    : 延时子程序
 Description : 延时 x 毫秒
*******************************************************/
void delay(u16 xms)
{
	int x, y;
	for (x = xms; x > 0; x--)
	{
		for (y = 125; y > 0; y--)
			;
	}
}
//sbit pin_ex_int0 = P3 ^ 2; //外部中断0
//sbit pin_ex_int1 = P3 ^ 3; //外部中断1

//u32 millis()
//{
//	return timeMs;
//}

//P1口接数码管的7位段选
//sbit wela1 = P2 ^ 0; // 位选
//sbit wela2 = P2 ^ 1; // 位选
//sbit wela3 = P2 ^ 2; // 位选
//sbit wela4 = P2 ^ 3; // 位选
//sbit pin_led = P3 ^ 0;

//u8 code table[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,//共阴数码管段选码表
//				   0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71};
////u8 code table[] = {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, //共阳数码管段选码表
////				   0x80, 0x90, 0x88, 0x83, 0xc6, 0xa1, 0x86, 0x8e};

//u8 shi_1 = 0, ge_1 = 0;//数码管显示前两位
//u8 shi_2 = 0, ge_2 = 0;//数码管显示后两位

//u8 num[8];		//各个数码管显示的值

	
	
//	timeMs++;
//	Counter1++;
//	if (Counter1 >= timerPeriod1)
//	{
//		Counter1 = 0;
//		
//			
//		if ((t - tTime[0]) >= (1000 / PWM_GENERATE_FREQUENCY))
//		{
//			pwmPeriodCount++;
//			limit(pwmPeriodCount, 0, 200);
//			(pwmPeriodCount > pwmHighTime) ? (pin_pwm = 0) : (pin_pwm = 1);
//			tTime[0] = t;
//			
//		
//		
//		}
//	}


	
//	if(TrigFlag == 1)
//	{
//		TempTime++;
//		if(TempTime >= 10)
//		{
//			TempTime=0;
//			TrigFlag=0;
//		}
//		
//	}
	
//	perviousTime = time;
//	time = timeMs;
//	diffTime = time - perviousTime; //计算得到两个液滴之间的时间间隔 ms
	
//	diffTimeFiltered = diffTime;
//	if(diffTimeFiltered>10)
//	{
//	//液滴计数，光电传感器触发一次，下降沿触发一次，计为一个液滴
//	exInt0Count++; 
//	//已经消耗的毫升数(ml)＝(滴) / 滴系数(滴/ml)。
//	consumeLiquid = exInt0Count / dropCoefficient;
//	//液滴速度 每小时输入的毫升数(ml/h) = (滴/min) x 60(min/h) / 滴系数(滴/ml)
//	flowRateActual = (1000 * 60 / diffTimeFiltered) * 60 / dropCoefficient;
//	}
//	
	
//	if ((t - tTime[0]) >= (1000 / PWM_GENERATE_FREQUENCY))
//	{
//		pwmPeriodCount++;
//		limit(pwmPeriodCount, 0, 200);
//		(pwmPeriodCount > pwmHighTime) ? (pin_pwm = 0) : (pin_pwm = 1);
//		tTime[0] = t;
//	}
//	if ((t - tTime[1]) >= (1000 / STATE_PROCESS_FREQUENCY))
//	{
//		switch (state)
//		{
//		case WORK_STATE_NORMAL:			  // 正常状态
////			numDisplay1 = flowRateActual; //前两段数码管显示液体流速
////			numDisplay2 = liquidProgress; //后两段数码管显示已完成百分比进度

//			//液滴速度 每小时输入的毫升数(ml/h) = (滴/min) x 60(min/h) / 滴系数(滴/ml)
//			flowRateActual = (1000 * 60 / diffTimeFiltered) * 60 / dropCoefficient;
//			//剩余液体容积(ml)= 液体总容积(ml) - 已消耗的液体容积(ml)
//			residualLiquid = liquidVolume - consumeLiquid;
//			//输液进度(液位)= 已消耗的液体容积(ml) / 液体总容积(ml)
//			liquidProgress = consumeLiquid / liquidVolume;
//			//舵机控制角度PID计算 输入:液滴的真实速度 液滴的设定速度 KP KI KD  输出:舵机角度
//			servoAngle = pid_control(flowRateActual, flowRateSet, 10.0f, 0.01f, 0.001f) * PID_OUTPUT_TO_SERVO_ANGLE;
//			//由舵机控制角度，控制对输液管的压力，达到该表流速的效果
//			servo_ctrl(servoAngle);

//			break;
//		case WORK_STATE_ALARM: // 报警状态
//			pin_buzzer = 0;	   //蜂鸣器响起
//			Write_allLED(0xff);
//			//pin_led = 0;	   //LED灯亮
//			break;
//		default:
//			break;
//		}
//		tTime[1] = t;
//	}
//	if ((t - tTime[2]) >= (1000 / DETECT_FREQUENCY))
//	{
//		//判断流速是否小于最小流速，如果是则记录一次
//		flowRateActual < miniFlowRate ? counterTemp++ : counterTemp--;
//		constrain(counterTemp, 0, 10);
//		//只有累计大于十次检测到流速小于最小流速才认为液体滴完了，进入报警状态整个过程持续1s
//		if (counterTemp >= 10)
//		{
//			counterTemp = 0;
//			state = WORK_STATE_ALARM;
//		}
//		tTime[2] = t;
//	}

//// 舵机角度控制, pwmHighTime 5-25 angle 0-180
//void servo_ctrl(u16 angle)
//{
//	pwmHighTime = angle / 9 + 5;
//	limit(pwmHighTime,5,25);
//}

//	timeFilterTempValue += diffTime;
//	CounterExint0++;
//	limit(CounterExint0, 0, 9);
//	if (CounterExint0 == 9)
//	{
//		diffTimeFiltered = timeFilterTempValue / 10; //均值滤波，取十个数据算平均值
//		timeFilterTempValue = 0;
//	}
	
//	TrigFlag=1;
//	
//	if(TrigFlag == 0)
//	{

//	TempTime++;
//	if(TempTime >= 100)
//	{
//		TempTime=0;
//		timeMs++;
//	}
	
//	if (pin_irf == 0)
//	{
//		while (pin_irf == 0);
//		
//		
//		dropCoefficient += 5;
//		limit(dropCoefficient, 10, 20);
//	}
	
//	if (pin_inf == 0)
//	{
//		delay(10); // 按键消抖
//		if (pin_inf == 0)
//		{
//			perviousTime = time;
//			time = timeMs;
//			diffTime = time - perviousTime; //计算得到两个液滴之间的时间间隔 ms
//			diffTimeFiltered = diffTime;
//			//液滴计数，光电传感器触发一次，下降沿触发一次，计为一个液滴
//			exInt0Count++; 
//			//已经消耗的毫升数(ml)＝(滴) / 滴系数(滴/ml)。
//			consumeLiquid = exInt0Count / dropCoefficient;
//			//液滴速度 每小时输入的毫升数(ml/h) = (滴/min) x 60(min/h) / 滴系数(滴/ml)
//			flowRateActual = (1000 * 60 / diffTimeFiltered) * 60 / dropCoefficient;
//		}
//		while (!pin_inf);	  // 检测是否松开按键
//		delay(10); // 消抖
//		while (!pin_inf)
//			;
//	}
		
	//tempDropCoefficient = (float)dropCoefficient;

//void EX_INT1() interrupt 2 //外部中断1
//{
//}

//	sbit key1 = P2 ^ 4;
//sbit key2 = P2 ^ 5;
//sbit key3 = P2 ^ 6;