/**************************************����˵��*************************************
*
* ��Ϊ��Ҫ�õ�ADC ��stc89c51ϵ��û������ADC ���ѡ����STC15F2K60S2
* ��оƬ��STC89C51������۶���ȫһ�� ��ͬ�����ڲ���Դ �����滻c51
*
* ����������ʹ�õ��ǿ����� ���뵥Ƭ����һ������ ��������ʱ��ֵΪ0
* ���ߴ�����������·����֮�� ��һ��������ģ���� ��ADC ͨ��0������
* ������������һ������Ƭ����300KZH�������ź�֮��������� ������
* ��������Ƭ֮����ݸı� ��һ�˵ĵ�ѹ�̶��ı� ��ADCͨ��1����
* ADC��ֵ����ת���õ�����ֵ ��TouchZero���бȽ��жϴ�������δ����
*  
* MainHandle������������߼��ж�
*
* STC15F2K60S2��ADC�����ݸ�Ӧ������.
* оƬ�Ĺ���Ƶ��Ϊ24MHz
*
***********************************************************************************/

#include <reg52.h>
#include <intrins.h>

#define MAIN_Fosc 24000000UL //������ʱ��Ƶ�� 24M

typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;

#define Timer0_Reload (65536UL - (MAIN_Fosc / 600000)) //Timer 0 ��װֵ ����300KHZ�Ĳ� �����𵴵���Ƭ

sfr P1ASF = 0x9D;	  //ֻд��ģ������ѡ��
sfr ADC_CONTR = 0xBC; //��ADϵ��
sfr ADC_RES = 0xBD;	  //��ADϵ��
sfr ADC_RESL = 0xBE;  //��ADϵ��
sfr AUXR = 0x8E;
sfr AUXR2 = 0x8F;

/*************	���س�������	**************/

#define TOUCH_CHANNEL 2 //ADCͨ����

#define ADC_90T (3 << 5)   //ADCʱ�� 90T
#define ADC_180T (2 << 5)  //ADCʱ�� 180T
#define ADC_360T (1 << 5)  //ADCʱ�� 360T
#define ADC_540T 0		   //ADCʱ�� 540T
#define ADC_FLAG (1 << 4)  //�����0
#define ADC_START (1 << 3) //�Զ���0

/*************	���ر�������	**************/

sbit P_LED = P2 ^ 4;   //LED�ӿ�
sbit P_SOUND = P2 ^ 7; //�����������ӿ�
sbit P_LIGHT = P2 ^ 6; //��������ӿ�
sbit P_TOUCH = P2 ^ 5; //�����������ӿ�

u16 idata adc[TOUCH_CHANNEL];		  //��ǰADCֵ
u16 idata adc_prev[TOUCH_CHANNEL];	  //��һ��ADCֵ
u16 idata TouchZero[TOUCH_CHANNEL];	  //0��ADCֵ
u8 idata TouchZeroCnt[TOUCH_CHANNEL]; //0���Զ����ټ���

u16 Counter1;			 //��ʱ��1����
u16 timerPeriod1 = 10000; //2000ms
u8 cnt_250ms;			 //250ms һ�����ڵ����ڼ���

u8 touch_button_state = 0; //����������״̬ 0δ����  1����
u8 light_sensor_state = 0; //���ߴ�������״̬ 0δ����  1����
u8 voice_sensor_state = 0; //������������״̬ 0δ����  1����

/*************	���غ�������	**************/
void delay_ms(u8 ms);
void ADC_init(void);
u16 Get_ADC10bitResult(u8 channel);
void AutoZero(void);
u8 check_adc(u8 index);
void MainHandle(void);

/******************** ������ **************************/
void main(void)
{
	//u8 i;

	delay_ms(50);

	ET0 = 0; //��ʼ��Timer0���һ��300KHZʱ��
	TR0 = 0;
	AUXR |= 0x80;  //Timer0 set as 1T mode
	AUXR2 |= 0x01; //�������ʱ��
	TMOD = 0;	   //Timer0 set as Timer, 16 bits Auto Reload.
	TH0 = (u8)(Timer0_Reload >> 8);
	TL0 = (u8)Timer0_Reload;
	TR0 = 1;

	TH1 = (65536 - 1000) / 256; // ���ö�ʱ��1��ʱʱ��1ms
	TH1 = (65536 - 1000) % 256;
	TR1 = 1; // ���ÿ��ƼĴ�����������ʱ��1
	ET1 = 0; // �����ж�����Ĵ���
	EA = 1;

	//ADC_init();	  //ADC��ʼ��
	delay_ms(50); //��ʱ50ms

	//ͨ��0 ����������������ĵ���ֵ TouchZero �ı�������
	adc_prev[0] = 1023;
	TouchZero[0] = 1023;
	TouchZeroCnt[0] = 0;
	//ͨ��1 ����������������ĵ���ֵ TouchZero �ı�������
	adc_prev[1] = 1023;
	TouchZero[1] = 1023;
	TouchZeroCnt[1] = 0;

	cnt_250ms = 0;

	while (1)
	{
		delay_ms(1); //ÿ��50ms����һ�ΰ���
		MainHandle(); //�������� �߼����������ﴦ��
//		if (++cnt_250ms >= 5)
//		{
//			cnt_250ms = 0;
//			AutoZero(); //ÿ��250ms����һ��0���Զ�����
//		}
	}
}
/**********************************************/

//========================================================================
// ����: void  delay_ms(unsigned char ms)
// ����: ��ʱ������
// ����: ms,Ҫ��ʱ��ms��, ����ֻ֧��1~255ms. �Զ���Ӧ��ʱ��.
// ����: none.
// �汾: VER1.0
// ����: 2013-4-1
// ��ע:
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

/*************  ADC��ʼ������ *****************/
void ADC_init(void)
{
	P1ASF = 0xff;	  //8·ADC
	ADC_CONTR = 0x80; //����ADC
}

//========================================================================
// ����: u16	Get_ADC10bitResult(u8 channel)
// ����: ��ѯ����һ��ADC���.
// ����: channel: ѡ��Ҫת����ADC.
// ����: 10λADC���.
// �汾: V1.0, 2012-10-22
//========================================================================
u16 Get_ADC10bitResult(u8 channel) //channel = 0~7
{
	ADC_RES = 0;
	ADC_RESL = 0;
	ADC_CONTR = 0x80 | ADC_90T | ADC_START | channel; //����ADC
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	while ((ADC_CONTR & ADC_FLAG) == 0)
		;												//�ȴ�ADCת������
	ADC_CONTR = 0x80;									//�����־
	return (((u16)ADC_RES << 2) | ((u16)ADC_RESL & 3)); //����ADC���
}

/********************* �Զ�0����ٺ��� *************************/
void AutoZero(void) //250ms����һ�� ����ʹ������2�������Ĳ�ľ���ֵ֮������⡣
{
	u8 i;
	u16 j, k;

	for (i = 0; i < TOUCH_CHANNEL; i++) //����8��ͨ��
	{
		j = adc[i];
		k = j - adc_prev[i]; //��ǰһ������
		F0 = 0;				 //����
		if (k & 0x8000)
			F0 = 1, k = 0 - k; //�ͷ�	������β����Ĳ�ֵ
		if (k >= 20)		   //�仯�Ƚϴ�
		{
			TouchZeroCnt[i] = 0; //����仯�Ƚϴ�����0������
			if (F0)
				TouchZero[i] = j; //������ͷţ����ұ仯�Ƚϴ���ֱ�����
		}
		else //�仯�Ƚ�С�����䶯���Զ�0�����
		{
			if (++TouchZeroCnt[i] >= 20) //������⵽С�仯20��/4 = 5��.
			{
				TouchZeroCnt[i] = 0;
				TouchZero[i] = adc_prev[i]; //�仯������ֵ��Ϊ0��
			}
		}
		adc_prev[i] = j; //������һ�εĲ���ֵ
	}
}

/********************* ��ȡ������Ϣ���� 50ms����1�� *************************/
u8 check_adc(u8 index) //�жϼ����»��ͷ�,�лز����
{
	u16 delta;
	adc[index] = 1023 - Get_ADC10bitResult(index); //��ȡADCֵ, ת�ɰ��¼�, ADCֵ����
	if (adc[index] < TouchZero[index])
		return 0; //��0�㻹С��ֵ������Ϊ�Ǽ��ͷ�
	delta = adc[index] - TouchZero[index];
	if (delta >= 40)
		return 1; //������
	if (delta <= 20)
		return 0; //���ͷ�
	return 2;	  //����ԭ״̬
}

/********************* ������ 50ms����1�� *************************/
void MainHandle(void)
{
/*�����߼��ж�*/
if (P_LIGHT == 1) //��û�й��յ�ʱ��
{
	if (P_SOUND == 0) //������
	{
		//timerPeriod1 = 2000; //�Զ�������ʱ�� ����2S
		P_LED = 0;			 //����
		ET1 = 1;			 //������ʱ��1 ��ʱ2S
	}
}
else //���й��յ�ʱ��
{
	//�����������춼������
}

if (P_TOUCH == 1) //ֻҪ��������������
{
	//timerPeriod1 = 10000; //�������ش��� ����4S
	P_LED = 0;			 //����
	ET1 = 1;			 //������ʱ��1 �ж�����Ĵ���
}
	
////	i = check_adc(0);
//	i = P_TOUCH;
//	if (i == 0)
//		touch_button_state = 0; //����������״̬ 0δ����  1����
//	if (i == 1)
//		touch_button_state = 1; //����������״̬ 0δ����  1����

////	i = check_adc(1);
//	i = P_LIGHT;
//	if (i == 0)
//		light_sensor_state = 0; //���ߴ�������״̬ 0δ����  1����
//	if (i == 1)
//		light_sensor_state = 1; //���ߴ�������״̬ 0δ����  1����

//	if (P_SOUND == 0)
//		voice_sensor_state = 1; //������������״̬ 0δ����  1����
//	if (P_SOUND == 1)
//		voice_sensor_state = 0; //������������״̬ 0δ����  1����

//	/*�����߼��ж�*/
//	if (light_sensor_state == 0) //��û�й��յ�ʱ��
//	{
//		if (voice_sensor_state == 1) //������
//		{
//			timerPeriod1 = 2000; //�Զ�������ʱ�� ����2S
//			P_LED = 0;			 //����
//			ET1 = 1;			 //������ʱ��1 ��ʱ2S
//		}
//	}
//	else //���й��յ�ʱ��
//	{
//		//�����������춼������
//	}

//	if (touch_button_state == 1) //ֻҪ��������������
//	{
//		timerPeriod1 = 4000; //�������ش��� ����4S
//		P_LED = 0;			 //����
//		ET1 = 1;			 //������ʱ��1 �ж�����Ĵ���
//	}
}

void Timer1() interrupt 3 //��ʱ��1�ж�
{
	TH1 = (65536 - 1000) / 256;
	TL1 = (65536 - 1000) % 256;
	Counter1++;
	if (Counter1 == timerPeriod1)
	{
		Counter1 = 0;
		P_LED = 1; //�ѵƹص�
		ET1 = 0;   //��ʱʱ�䵽����һ�� ֮��ص���ʱ�� ����һ�ο���
	}
}