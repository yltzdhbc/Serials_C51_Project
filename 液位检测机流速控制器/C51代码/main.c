/**********************************************************************************

* ������״̬
* 1. ѡ����Һ������(ml) ��ʱ�������λ����ʾ��Һ������ 
*    �������� key1 +50(ml) : key2 -50(ml) : key3 ��һ��
* 2. ѡ���ϵ�� ��ϵ����10��15��20(��/ml)3���ͺ� �������λ����ʾ10/15/20 
*    �������� key1 +5 : key2 -5 : key3 ��һ��
* 3. ������Һ״̬ ǰ�����������ʾҺ������ �������������ʾ����ɰٷֱȽ���
*    �������� �趨��Һ������ key1 +5 : key2 -5 : key3 ��һ�� ��ʱ���»ص���һ��״̬
*ALARM. �ۼƴ���ʮ�μ�⵽����С����С����,��ΪҺ������˽��뱨��״̬ ��������

***********************************************************************************/

#include <reg52.h>
#include <intrins.h>
#include	<tm1638.h>

/* ���峣�ñ��� */
typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;
#define constrain(amt, low, high) ((amt) < (low) ? (amt = low) : ((amt) > (high) ? (amt = high) : (amt = amt)))
#define limit(amt, low, high) ((amt) < (low) ? (amt = high) : ((amt) > (high) ? (amt = low) : (amt = amt)))
#define timer0Reload 1000
#define timer1Reload 100
#define WORK_STATE_NORMAL 0
#define WORK_STATE_ALARM 1
#define PID_OUTPUT_TO_SERVO_ANGLE 1 //PID��� ת��Ϊ ����Ƕ�

/*********************���Ŷ���********************/
sbit pin_pwm = P2 ^ 7;
sbit pin_buzzer = P0 ^ 0;
sbit pin_inf = P3 ^ 1;
/* ����ȫ�ֱ��� */
u8 numDisplay0 = 0;//�������ʾ����
u8 numDisplay1 = 0;//�������ʾ����
u8 numDisplay2 = 0;//�������ʾ����
u8 numDisplay3 = 0;//�������ʾ����
u8 numDisplay4 = 0;//�������ʾ����
u8 numDisplay5 = 0;//�������ʾ����
u8 numDisplay6 = 0;//�������ʾ����
u8 numDisplay7 = 0;//�������ʾ����

u8 servoAngle = 90;	  //�����ʼĬ�ϽǶ�
u8 pwmHighTime = 5; //5 - 15 ��Ӧ0-180��
u8 pwmPeriodCount = 200;//PWM���� �����pwm����Ϊ20ms

u8 myCase = 1; //״̬��ѡ��,ѡ��ǰ��״̬
u32 exInt0Count = 0 ,tempExInt0Count = 0;//��Һ�ε���
u32 timeMs = 0, tempTimeMs = 0; // �ɶ�ʱ���ĵ���ϵͳʱ�� ��λ ms
u16 diffTimeFiltered = 0;//�˲��������Һ�ε��¼��ʱ��
u16 liquidVolume = 100;	  //��Һ���ݻ� ml
u16 dropCoefficient = 15; //��ϵ�� �ٴ����õ���Һ�� ��ϵ����10��15��20(��/ml)3���ͺ�
u8 liquidProgress = 0; //��ʾҺλ Ҳ��ʾ��Һ����
u16 flowRateSet = 0,tempFlowRateSet = 0;	//��ε������趨ֵ
u16 flowRateActual = 0; //��ε�������ʵֵ���ɴ���������õ�
u16 miniFlowRate = 10;	//��С������� С�����ֵ����Ϊ��ε�ǰ����Ϊ0����������
u16 consumeLiquid; //�����ĵ�Һ�� ����
u8 residualLiquid_precent; //ʣ��Һ�� �ٷֱ� ��ʾ��ǰ����
u16 residualLiquid = 0;		   //ʣ��Һ�� ����
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
/*********************��������********************/
void delay(u16);
void Timer_Init();
void ExInt_Init();
float pid_control(long ref, long set, float Kp, float Ki, float Kd);
void servo_ctrl(flowRateActual, flowRateSet);

/*********************������********************/
void main()
{
	Timer_Init(); //��ʱ����ʼ��
	ExInt_Init(); //�ⲿ�жϳ�ʼ��
	EA = 1;
	ET0 = 0;
	ET1 = 1;
	
	init_TM1638();			//��ʼ��TM1638
	
	for(index=0;index<8;index++)
		Write_DATA(index<<1,tab[0]);			//��ʼ���Ĵ���
	
	Write_allLED(0x00);
	
	while (1)
	{
		
		//д�����������
		Write_DATA(0,tab[numDisplay0]);
		Write_DATA(2,tab[numDisplay1]);		
		Write_DATA(4,tab[numDisplay2]);		
		Write_DATA(6,tab[numDisplay3]);
		Write_DATA(8,tab[numDisplay4]);		
		Write_DATA(10,tab[numDisplay5]);		
		Write_DATA(12,tab[numDisplay6]);
		Write_DATA(14,tab[numDisplay7]);		
		
		//ѡ����Һ���ݻ� ml
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
		
		//ѡ���ϵ�� �ٴ����õ���Һ�� ��ϵ����10��15��20(��/ml)3���ͺ�
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
		
		//ѡ���ε������趨ֵ
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
		
		//������ʱ���������ٿ���
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
			
			timeMs++;//����
			
			//Һ���ٶ� ÿСʱ����ĺ�����(ml/h) = (��/min) x 60(min/h) / ��ϵ��(��/ml)
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
		
		//��һ�� ��һ�� �л�
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
		case WORK_STATE_NORMAL:	// ����״̬
//			//Һ���ٶ� ÿСʱ����ĺ�����(ml/h) = (��/min) x 60(min/h) / ��ϵ��(��/ml)
//			flowRateActual = (1000 * 60 / diffTimeFiltered) * 60 / dropCoefficient;
			//ʣ��Һ���ݻ�(ml)= Һ�����ݻ�(ml) - �����ĵ�Һ���ݻ�(ml)
			residualLiquid = liquidVolume - consumeLiquid;
			//��Һ����(Һλ)= �����ĵ�Һ���ݻ�(ml) / Һ�����ݻ�(ml)
			liquidProgress = 100 * consumeLiquid / liquidVolume;
			//������ƽǶ�PID���� ����:Һ�ε���ʵ�ٶ� Һ�ε��趨�ٶ� KP KI KD  ���:����Ƕ�
			//servoAngle = pid_control(flowRateActual, flowRateSet, 0.5f, 0.00f, 0.0001f) * PID_OUTPUT_TO_SERVO_ANGLE;
			//�ɶ�����ƽǶȣ����ƶ���Һ�ܵ�ѹ�����ﵽ�ñ����ٵ�Ч��
			servo_ctrl(flowRateActual, flowRateSet);
			flowRateSet=tempFlowRateSet;
			pin_buzzer = 1;	   //����������
			Write_allLED(0x00);
			break;
		case WORK_STATE_ALARM: // ����״̬
			pin_buzzer = 0;	   //����������
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

//		//�ж������Ƿ�С����С���٣���������¼һ��
//		flowRateActual < miniFlowRate ? counterTemp++ : counterTemp--;
//		constrain(counterTemp, 0, 10);
//		//ֻ���ۼƴ���ʮ�μ�⵽����С����С���ٲ���ΪҺ������ˣ����뱨��״̬�������̳���1s
//		if (counterTemp >= 10)
//		{
//			counterTemp = 0;
//			state = WORK_STATE_ALARM;
//		}
		
	}
}

//#define PWM_GENERATE_FREQUENCY 100 //Ƶ��
#define STATE_PROCESS_FREQUENCY 10	 //Ƶ��
#define DETECT_FREQUENCY 1		 //Ƶ��

u32 t = 0;
static u32 tTime[2];
u16 Counter0;
void Timer0() interrupt 1 //��ʱ��0�ж�
{
	TH0 = (65536 - timer0Reload) / 256;
	TL0 = (65536 - timer0Reload) % 256;
	timeMs++;
}

void Timer1() interrupt 3 //��ʱ��1�ж�
{
	TH1 = (65536 - timer1Reload) / 256;
	TL1 = (65536 - timer1Reload) % 256;
	pwmPeriodCount++; // pwmPeriodCount ��0��200ѭ��
	if(pwmPeriodCount>=200) pwmPeriodCount=0;
	(pwmPeriodCount > pwmHighTime) ? (pin_pwm = 0) : (pin_pwm = 1);
}

u16 CounterExint0;
/*******************�ⲿ�ж�**********************/
void EX_INT0() interrupt 0 //�ⲿ�ж�0
{
	//Һ�μ�������紫��������һ�Σ��½��ش���һ�Σ���Ϊһ��Һ��
	tempExInt0Count++;
	if(tempExInt0Count>=3)
	{
		perviousTime = time;
		time = timeMs;
		diffTime = time - perviousTime; //����õ�����Һ��֮���ʱ���� ms
		tempExInt0Count=0;
		exInt0CountActual++;
		//�Ѿ����ĵĺ�����(ml)��(��) / ��ϵ��(��/ml)��
		consumeLiquid = exInt0CountActual / dropCoefficient;
	}
}


// PID������
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
// ����Ƕȿ���, pwmHighTime 5-25 angle 0-180
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
	IT0 = 1; //�����ж����� 1Ϊ�½��� 0Ϊ�͵�ƽ
	EX0 = 1; //ʹ���ⲿ�ж�0
	EA = 1;
	//	IT1 = 1; //�����ж����� 1Ϊ�½��� 0Ϊ�͵�ƽ
	//	EX1 = 1; //ʹ���ⲿ�ж�1
}

void Timer_Init()
{
	EA = 1;						// �����ж�����Ĵ�����1.���ж��ܿ���
	ET0 = 1;					// �����ж�����Ĵ�����2.�򿪶�ʱ��0
	ET1 = 1;					// �����ж�����Ĵ�����2.�򿪶�ʱ��1
	TMOD = 0x11;				// ���ù�����ʽ�Ĵ���TMOD��������ʽ1 -> 0x01
	TH0 = (65536 - timer0Reload) / 256;	// ���ö�ʱ��0��ʱʱ��100ns
	TL0 = (65536 - timer0Reload) % 256;	//
	TH1 = (65536 - timer1Reload) / 256; // ���ö�ʱ��1��ʱʱ��1ms
	TH1 = (65536 - timer1Reload) % 256; //
	TR0 = 1;					// ���ÿ��ƼĴ�����������ʱ��0
	TR1 = 1;					// ���ÿ��ƼĴ�����������ʱ��1
}

/******************************************************
 Function    : ��ʱ�ӳ���
 Description : ��ʱ x ����
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
//sbit pin_ex_int0 = P3 ^ 2; //�ⲿ�ж�0
//sbit pin_ex_int1 = P3 ^ 3; //�ⲿ�ж�1

//u32 millis()
//{
//	return timeMs;
//}

//P1�ڽ�����ܵ�7λ��ѡ
//sbit wela1 = P2 ^ 0; // λѡ
//sbit wela2 = P2 ^ 1; // λѡ
//sbit wela3 = P2 ^ 2; // λѡ
//sbit wela4 = P2 ^ 3; // λѡ
//sbit pin_led = P3 ^ 0;

//u8 code table[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,//��������ܶ�ѡ���
//				   0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71};
////u8 code table[] = {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, //��������ܶ�ѡ���
////				   0x80, 0x90, 0x88, 0x83, 0xc6, 0xa1, 0x86, 0x8e};

//u8 shi_1 = 0, ge_1 = 0;//�������ʾǰ��λ
//u8 shi_2 = 0, ge_2 = 0;//�������ʾ����λ

//u8 num[8];		//�����������ʾ��ֵ

	
	
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
//	diffTime = time - perviousTime; //����õ�����Һ��֮���ʱ���� ms
	
//	diffTimeFiltered = diffTime;
//	if(diffTimeFiltered>10)
//	{
//	//Һ�μ�������紫��������һ�Σ��½��ش���һ�Σ���Ϊһ��Һ��
//	exInt0Count++; 
//	//�Ѿ����ĵĺ�����(ml)��(��) / ��ϵ��(��/ml)��
//	consumeLiquid = exInt0Count / dropCoefficient;
//	//Һ���ٶ� ÿСʱ����ĺ�����(ml/h) = (��/min) x 60(min/h) / ��ϵ��(��/ml)
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
//		case WORK_STATE_NORMAL:			  // ����״̬
////			numDisplay1 = flowRateActual; //ǰ�����������ʾҺ������
////			numDisplay2 = liquidProgress; //�������������ʾ����ɰٷֱȽ���

//			//Һ���ٶ� ÿСʱ����ĺ�����(ml/h) = (��/min) x 60(min/h) / ��ϵ��(��/ml)
//			flowRateActual = (1000 * 60 / diffTimeFiltered) * 60 / dropCoefficient;
//			//ʣ��Һ���ݻ�(ml)= Һ�����ݻ�(ml) - �����ĵ�Һ���ݻ�(ml)
//			residualLiquid = liquidVolume - consumeLiquid;
//			//��Һ����(Һλ)= �����ĵ�Һ���ݻ�(ml) / Һ�����ݻ�(ml)
//			liquidProgress = consumeLiquid / liquidVolume;
//			//������ƽǶ�PID���� ����:Һ�ε���ʵ�ٶ� Һ�ε��趨�ٶ� KP KI KD  ���:����Ƕ�
//			servoAngle = pid_control(flowRateActual, flowRateSet, 10.0f, 0.01f, 0.001f) * PID_OUTPUT_TO_SERVO_ANGLE;
//			//�ɶ�����ƽǶȣ����ƶ���Һ�ܵ�ѹ�����ﵽ�ñ����ٵ�Ч��
//			servo_ctrl(servoAngle);

//			break;
//		case WORK_STATE_ALARM: // ����״̬
//			pin_buzzer = 0;	   //����������
//			Write_allLED(0xff);
//			//pin_led = 0;	   //LED����
//			break;
//		default:
//			break;
//		}
//		tTime[1] = t;
//	}
//	if ((t - tTime[2]) >= (1000 / DETECT_FREQUENCY))
//	{
//		//�ж������Ƿ�С����С���٣���������¼һ��
//		flowRateActual < miniFlowRate ? counterTemp++ : counterTemp--;
//		constrain(counterTemp, 0, 10);
//		//ֻ���ۼƴ���ʮ�μ�⵽����С����С���ٲ���ΪҺ������ˣ����뱨��״̬�������̳���1s
//		if (counterTemp >= 10)
//		{
//			counterTemp = 0;
//			state = WORK_STATE_ALARM;
//		}
//		tTime[2] = t;
//	}

//// ����Ƕȿ���, pwmHighTime 5-25 angle 0-180
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
//		diffTimeFiltered = timeFilterTempValue / 10; //��ֵ�˲���ȡʮ��������ƽ��ֵ
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
//		delay(10); // ��������
//		if (pin_inf == 0)
//		{
//			perviousTime = time;
//			time = timeMs;
//			diffTime = time - perviousTime; //����õ�����Һ��֮���ʱ���� ms
//			diffTimeFiltered = diffTime;
//			//Һ�μ�������紫��������һ�Σ��½��ش���һ�Σ���Ϊһ��Һ��
//			exInt0Count++; 
//			//�Ѿ����ĵĺ�����(ml)��(��) / ��ϵ��(��/ml)��
//			consumeLiquid = exInt0Count / dropCoefficient;
//			//Һ���ٶ� ÿСʱ����ĺ�����(ml/h) = (��/min) x 60(min/h) / ��ϵ��(��/ml)
//			flowRateActual = (1000 * 60 / diffTimeFiltered) * 60 / dropCoefficient;
//		}
//		while (!pin_inf);	  // ����Ƿ��ɿ�����
//		delay(10); // ����
//		while (!pin_inf)
//			;
//	}
		
	//tempDropCoefficient = (float)dropCoefficient;

//void EX_INT1() interrupt 2 //�ⲿ�ж�1
//{
//}

//	sbit key1 = P2 ^ 4;
//sbit key2 = P2 ^ 5;
//sbit key3 = P2 ^ 6;