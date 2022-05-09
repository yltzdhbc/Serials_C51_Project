#ifndef	_TM1638_H
#define	_TM1638_H

#include <reg52.h>

#define	DATA_COMMAND	0X40
#define	DISP_COMMAND	0x80
#define	ADDR_COMMAND	0XC0

//TM1638ģ�����Ŷ���
sbit	DIO=P1^0;
sbit	CLK=P1^1;
sbit	STB=P1^2;

//�����������ʾ����
unsigned char code tab[]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,
                           0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71};


void TM1638_Write(unsigned char	DATA)			//д���ݺ���
{
	unsigned char i;
	for(i=0;i<8;i++)
	{
		CLK=0;
		if(DATA&0X01)
			DIO=1;
		else
			DIO=0;
		DATA>>=1;
		CLK=1;
	}
}
unsigned char TM1638_Read(void)					//�����ݺ���
{
	unsigned char i;
	unsigned char temp=0;
	DIO=1;	//����Ϊ����
	for(i=0;i<8;i++)
	{
		temp>>=1;
		CLK=0;
		if(DIO)
			temp|=0x80;
		CLK=1;
	}
	return temp;
}
void Write_COM(unsigned char cmd)		//����������
{
	STB=0;
	TM1638_Write(cmd);
	STB=1;
}
unsigned char Read_key(void)
{
	unsigned char c[4],i,key_value=0;
	STB=0;
	TM1638_Write(0x42);		           //����ɨ���� ����
	for(i=0;i<4;i++)		
		c[i]=TM1638_Read();
	STB=1;					           //4���ֽ����ݺϳ�һ���ֽ�
	for(i=0;i<4;i++)
		key_value|=c[i]<<i;
	for(i=0;i<8;i++)
		if((0x01<<i)==key_value)
			break;
	return i;
}
void Write_DATA(unsigned char add,unsigned char DATA)		//ָ����ַд������
{
	Write_COM(0x44);
	STB=0;
	TM1638_Write(0xc0|add);
	TM1638_Write(DATA);
	STB=1;
}
/*
void Write_oneLED(unsigned char num,unsigned char flag)	//��������һ��LED������numΪ��Ҫ���Ƶ�led��ţ�flagΪ0ʱϨ�𣬲�Ϊ0ʱ����
{
	if(flag)
		Write_DATA(2*num+1,1);
	else
		Write_DATA(2*num+1,0);
}  	*/
void Write_allLED(unsigned char LED_flag)					//����ȫ��LED������LED_flag��ʾ����LED״̬
{
	unsigned char i;
	for(i=0;i<8;i++)
		{
			if(LED_flag&(1<<i))
				//Write_DATA(2*i+1,3);
				Write_DATA(2*i+1,1);
			else
				Write_DATA(2*i+1,0);
		}
}

//TM1638��ʼ������
void init_TM1638(void)
{
	unsigned char i;
	Write_COM(0x8b);       //���� (0x88-0x8f)8�����ȿɵ�
	Write_COM(0x40);       //���õ�ַ�Զ���1
	STB=0;		           //
	TM1638_Write(0xc0);    //������ʼ��ַ

	for(i=0;i<16;i++)	   //����16���ֽڵ�����
		TM1638_Write(0x00);
	STB=1;
}
#endif
