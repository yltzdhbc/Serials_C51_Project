
/*************	����˵��	**************
��Ϊ�ڴ�ռ�Ͷ�ʱ����Դ������ ��ͨ��STC89C52RC�Ѿ���������Ҫ
�����ѡ�� STC15F2K60S2 ��оƬ��STC89C51������۶���ȫһ�� ��ͬ�����ڲ���Դ ������ȫ�滻c51

ʹ����3����ʱ��:
��ʱ��0�ж�����ѭ��ˢ��LED�����״̬ ������ʾ����
��ʱ��1�ж�����ѭ����������̵�״̬��������¼���ж���һ����������ȥ��
��ʱ��2�ж��������ڵĲ����ʲ��� �ɼĴ���������

����1ȫ˫���жϷ�ʽ�շ�ͨѶ���� ʹ��22.1184MHZʱ�� ��Ҫ�ı� ���޸������"������ʱ��"��ֵ�����±���
��������Ϊ��115200,8,n,1.

����ѭ���У�
��������������е�״̬ ��ֵ��LED��������
��⴮�ڽ��յ������е�������Ԥ�Ƶ���������Ա� �õ���ǰɨ��������Ӧ�Ĺ������ ���ر�LED�Ʊ�ʾ���Ŵ�
******************************************/
#include "STC15Fxxxx.H"
#include <intrins.h>
#define uchar unsigned char
#define uint  unsigned int
	
#define constrain(amt, low, high) ((amt) < (low) ? (amt = low) : ((amt) > (high) ? (amt = high) : (amt = amt)))
#define limit(amt, low, high) ((amt) < (low) ? (amt = high) : ((amt) > (high) ? (amt = low) : (amt = amt)))

/*************	���س�������	**************/
//#define MAIN_Fosc 22118400L //������ʱ��

#define MAIN_Fosc 12000000L //������ʱ��

#define RX1_Lenth 32	   //���ڽ��ջ��峤��
#define BaudRate1 9600UL //ѡ������
#define BaudRate2 9600UL //ѡ������

#define Timer2_Reload (65536UL - (MAIN_Fosc / 4 / BaudRate2)) //Timer2 300KHZ

/*************	���ر�������	**************/
u8 idata RX1_Buffer[RX1_Lenth]; //���ջ���
u8 TX1_Cnt;						//���ͼ���
u8 RX1_Cnt;						//���ռ���
bit B_TX1_Busy;					//����æ��־

//�������ֵ��Ӧ����ž���
u32 BarCode_Table[24] = {90311017, 90311024, 90311031, 90311048,
						 90311055, 90311062, 90311079, 90311086,
						 90311093, 90311109, 90311116, 90311123,
						 90311130, 90311147, 90311154, 90311161,
						 90311178, 90311185, 90311192, 90311208,
						 90311215, 90311222, 90311239, 90311246};

//
//����Max7219�˿�
sbit Max7219_pinCLK = P2^2;
sbit Max7219_pinCS  = P2^1;
sbit Max7219_pinDIN = P2^0;
						 
u8 LED_LIST_INDEX = 1; //LED�����������
u8 LED_LINE_INDEX = 1; //LED�����������
//LED�������飬6��4�У���Ϊ0����Ϊ1
//u8 Matrix_LED[6][4] = {{1, 1, 1, 1},
//											 {1, 1, 1, 1},
//											 {1, 1, 1, 1},
//											 {1, 1, 1, 1},
//											 {1, 1, 1, 1},
//											 {1, 1, 1, 1}};

u8 Matrix_LED[6][4] = {{0, 0, 0, 0},
											 {0, 0, 0, 0},
											 {0, 0, 0, 0},
											 {0, 0, 0, 0},
											 {0, 0, 0, 0},
											 {0, 0, 0, 0}};
//KEY�����нӿڶ���
sbit PIN_KEY_LIST_1 = P1 ^ 4;
sbit PIN_KEY_LIST_2 = P1 ^ 5;
sbit PIN_KEY_LIST_3 = P1 ^ 6;
sbit PIN_KEY_LIST_4 = P1 ^ 7;
//KEY�����нӿڶ���
//sbit PIN_KEY_LINE_1 = P1 ^ 0;
//sbit PIN_KEY_LINE_2 = P1 ^ 1;
//sbit PIN_KEY_LINE_3 = P1 ^ 2;
//sbit PIN_KEY_LINE_4 = P1 ^ 3;
sbit PIN_KEY_LINE_1 = P0 ^ 0;
sbit PIN_KEY_LINE_2 = P0 ^ 1;
sbit PIN_KEY_LINE_3 = P0 ^ 2;
sbit PIN_KEY_LINE_4 = P0 ^ 3;
											 
sbit PIN_KEY_LINE_5 = P3 ^ 2;
sbit PIN_KEY_LINE_6 = P3 ^ 3;

u8 KEY_LIST_INDEX = 1; //KEY�����������
u8 KEY_LINE_INDEX = 1; //KEY�����������
//�����������飬6��4�У�����Ϊ0��δ����Ϊ1
u8 Matrix_KEY[6][4] = {{1, 1, 1, 1},
											 {1, 1, 1, 1},
											 {1, 1, 1, 1},
											 {1, 1, 1, 1},
											 {1, 1, 1, 1},
											 {1, 1, 1, 1}};
//u8 Matrix_KEY[6][4] = {{0, 0, 0, 0},
//											 {0, 0, 0, 0},
//											 {0, 0, 0, 0},
//											 {0, 0, 0, 0},
//											 {0, 0, 0, 0},
//											 {0, 0, 0, 0}};
/*************	˽�б�������	**************/
u16 i, j, k;	  //�����������ı���
u32 Scan_BarCode; //ɨ�赽�������ֵ
/**********************************************/

//--------------------------------------------
//���ܣ���MAX7219(U3)д���ֽ�
//��ڲ�����DATA 
//���ڲ�������
//˵����
void Write_Max7219_byte(uchar DATA)         
{
    	uchar i;    
		Max7219_pinCS=0;		
	    for(i=8;i>=1;i--)
          {		  
            Max7219_pinCLK=0;
            Max7219_pinDIN=DATA&0x80;
            DATA=DATA<<1;
            Max7219_pinCLK=1;
           }                                 
}
//-------------------------------------------
//���ܣ���MAX7219д������
//��ڲ�����address��dat
//���ڲ�������
//˵����
void Write_Max7219(uchar address,uchar dat)
{ 
   Max7219_pinCS=0;
	 Write_Max7219_byte(address);           //д���ַ��������ܱ��
   Write_Max7219_byte(dat);               //д�����ݣ����������ʾ���� 
	 Max7219_pinCS=1;                        
}

void Init_MAX7219(void)
{
 Write_Max7219(0x09, 0x00);       //���뷽ʽ��BCD��
 Write_Max7219(0x0a, 0x03);       //���� 
 Write_Max7219(0x0b, 0x07);       //ɨ����ޣ�8���������ʾ
 Write_Max7219(0x0c, 0x01);       //����ģʽ��0����ͨģʽ��1
 Write_Max7219(0x0f, 0x00);       //��ʾ���ԣ�1�����Խ�����������ʾ��0
}

void Delay_xms(uint x)
{
 uint i,j;
 for(i=0;i<x;i++)
  for(j=0;j<112;j++);
}


//uchar i,j;
uchar led_line[6] = {0x00,0x00,0x00,0x00,0x00,0x00};

void Delay(uint x)
{
 	uchar i;
	while(x--)
	{
	 	for(i=0;i<120;i++);
	}
}

void putc_to_SerialPort(uchar c)
{
 	SBUF = c;
	while(TI == 0);
	TI = 0;
}

void puts_to_SerialPort(uchar *s)
{
 	while(*s != '\0')
	{
	 	putc_to_SerialPort(*s);
		s++;
		Delay(5);	
	}
}

uchar Receive_Buffer[101];
uchar Buf_Index = 0;
	uchar c = 0;
void main(void)
{
	B_TX1_Busy = 0;
	RX1_Cnt = 0;
	TX1_Cnt = 0;

	/**********��ʱ��0 ��ʱ��1 ����**********/
	EA = 1;						// �����ж�����Ĵ�����1.���ж��ܿ���
	ET0 = 1;					// �����ж�����Ĵ�����2.�򿪶�ʱ��0
	//ET1 = 1;					// �����ж�����Ĵ�����2.�򿪶�ʱ��1
	TMOD = 0x11;				// ���ù�����ʽ�Ĵ���TMOD��������ʽ1 -> 0x01
	TH0 = (65536 - 500) / 256; // ���ö�ʱ��0��ʱʱ��1ms
	TL0 = (65536 - 500) % 256; //
	//TH1 = (65536 - 500) / 256; // ���ö�ʱ��1��ʱʱ��1ms
	//TH1 = (65536 - 500) % 256; //
	TR0 = 1;					// ���ÿ��ƼĴ�����������ʱ��0
	//TR1 = 1;					// ���ÿ��ƼĴ�����������ʱ��1
	EA = 1;		  //����ȫ���ж�
	

//	/**********����1����**********/
//	S1_8bit();			//8λ����
//	S1_USE_P30P31();	//UART1 ʹ��P30 P31��	Ĭ��
//	Timer2_Stop();		//��ֹ��ʱ��2����
//	S1_BRT_UseTimer2(); //������ʹ��Timer2����
//	Timer2_1T();		//Timer2 set as 1T mode
//	TH2 = (u8)(Timer2_Reload >> 8);
//	TL2 = (u8)Timer2_Reload;
//	Timer2_Run(); //����ʱ��2����
//	REN = 1;	  //�������
//	ES = 1;		  //�������ж�

	SCON = 0x50;
	TMOD = 0x20;
	PCON = 0x00;
	TH1  = 0xfd;
	TL1  = 0xfd;
	EA   = 1;
	EX0  = 1;
	IT0  = 1;
	ES   = 1;
	IP   = 0x01;
	TR1  = 1;
	
//	SCON = 0x40;
//	TMOD = 0x20;
//	PCON = 0x00;
//	TH1  = 0xfd;
//	TL1  = 0xfd;
//	TI   = 0;
//	TR1  = 1;
	Delay(200);
	puts_to_SerialPort("Receiving From 8051...\r\n");
	puts_to_SerialPort("------------------------------\r\n");
	Delay(50);
	
	EA = 1;		  //����ȫ���ж�
	
	Init_MAX7219();  

	while (1)
	{
		
//		if(TX1_Cnt != RX1_Cnt)		//�յ�������
//		{
//			if(!B_TX1_Busy)		//���Ϳ���
//			{
//				B_TX1_Busy = 1;		//��־����æ
//				SBUF = RX1_Buffer[TX1_Cnt];	//��һ���ֽ�
//				if(++TX1_Cnt >= RX1_Lenth)	TX1_Cnt = 0;	//�����������
//			}
//		}
		
//		for(i=0;i<100;i++)
//		{
//		 	if(Receive_Buffer[i]==-1) 
//				break;
//			puts_to_SerialPort("Received\r\n");
//			//P0 = DSY_CODE[Receive_Buffer[i]];
//			Delay(200); 
//		}
		
		puts_to_SerialPort("Received\r\n");
		Delay(200);
		
//		putc_to_SerialPort(c+'A');
//		Delay(100);
//		putc_to_SerialPort(' '); //����ط�������ֻ�����ұ�����һ���ո�
//		Delay(100);
//		if(c==25)
//		{
//		 	puts_to_SerialPort("\r\n---------------------------\r\n");
//			Delay(100);
//		}
//		c = (c+1)%26;
//		if(c%10==0)
//		{
//		 	puts_to_SerialPort("\r\n");
//			Delay(100);
//		}
//		
		
		
//		//������ڽ��յ�������ǰ5λ��90311������Ϊ���յ��������ǶԵ�
//		if (RX1_Buffer[0] == 9 && RX1_Buffer[1] == 0 && RX1_Buffer[2] == 3 &&
//			RX1_Buffer[3] == 1 && RX1_Buffer[4] == 1)
//		{
//			//�������ۼ�����
//			Scan_BarCode += RX1_Buffer[0] * 10000000;
//			Scan_BarCode += RX1_Buffer[1] * 1000000;
//			Scan_BarCode += RX1_Buffer[2] * 100000;
//			Scan_BarCode += RX1_Buffer[3] * 10000;
//			Scan_BarCode += RX1_Buffer[4] * 1000;
//			Scan_BarCode += RX1_Buffer[5] * 100;
//			Scan_BarCode += RX1_Buffer[6] * 10;
//			Scan_BarCode += RX1_Buffer[7] * 1;

//			for (k = 0; k < 24; k++)
//			{
//				//�ж���һ��ֵ�붨��õ�������ȣ��������ž���������ӵ����
//				if (BarCode_Table[k] == Scan_BarCode)
//				{
//					Matrix_LED[(k / 4 + 1) - 1][k - 1] = 1; //�ر�LED��
//					Scan_BarCode = 0;						//ɨ�赽�������ֵ����
//				}
//			}
//		}
		
	}
}

void Serial_INT() interrupt 4
{
 	uchar c;
	if(RI==0) 
		return;
	ES = 0;
	RI = 0;
	c  = SBUF;
	if(c>='0' && c<='9')
	{
	 	Receive_Buffer[Buf_Index]=c-'0';
		Receive_Buffer[Buf_Index+1]=-1;
		Buf_Index = (Buf_Index+1)%100;	
	}
	ES = 1;
}


///*********** UART1�жϺ��� ʹ���˶�ʱ��2 **************/
//void UART1_int(void) interrupt UART1_VECTOR
//{
//	if (RI) //�����жϱ�־λ
//	{
//		RI = 0;						//��������жϱ�־λ
//		RX1_Buffer[RX1_Cnt] = SBUF; //����һ���ֽ�
//		if (++RX1_Cnt >= RX1_Lenth)
//			RX1_Cnt = 0; //�����������
//	}

//	if (TI) //�����жϱ�־λ
//	{
//		TI = 0;			//��������жϱ�־λ
//		B_TX1_Busy = 0; //�������æ��־
//	}
//}

#define timer0Period 50
int timer0PeriodCount = 0;
/*********** ��ʱ��0�жϺ��� **************/
void Timer0() interrupt 1
{
	TH0 = (65536 - 500) / 256;
	TL0 = (65536 - 500) % 256;
	timer0PeriodCount++;
	if (timer0PeriodCount >= timer0Period)
	{
		timer0PeriodCount = 0;
		
		for(j=0;j<6;j++)		
		{						 
			for(i=0;i<4;i++)
			{
				//(Matrix_KEY[j][i]==0)?(Matrix_LED[j][i]=1):(Matrix_LED[j][i]=0);
				(Matrix_KEY[j][i]==0)?(Matrix_LED[j][i]=1):(0);
				//if()
				led_line[j] |= (Matrix_LED[j][i] << (7-i));
			}
		}

	  for(i=0;i<6;i++)
			Write_Max7219(i+1,led_line[i]);
		
		Write_Max7219(7,0x00);
		Write_Max7219(8,0x00);
		
		for(i=0;i<6;i++)
			led_line[i] = 0x00;

	}
}

/*********** ��ʱ��1�жϺ��� **************/
void Timer1() interrupt 3
{
	TH1 = (65536 - 500) / 256;
	TL1 = (65536 - 500) % 256;

	KEY_LIST_INDEX++;			 //ѡ����
	limit(KEY_LIST_INDEX, 1, 4); //���Ʊ�����1-4��Χ�ڣ�����4���Ϊ1

	//����ÿ����1һλ�����ѭ��
	switch (KEY_LIST_INDEX)
	{
	case 1:
		PIN_KEY_LIST_1 = 0;
		PIN_KEY_LIST_2 = 1;
		PIN_KEY_LIST_3 = 1;
		PIN_KEY_LIST_4 = 1;
		break;
	case 2:
		PIN_KEY_LIST_1 = 1;
		PIN_KEY_LIST_2 = 0;
		PIN_KEY_LIST_3 = 1;
		PIN_KEY_LIST_4 = 1;
		break;
	case 3:
		PIN_KEY_LIST_1 = 1;
		PIN_KEY_LIST_2 = 1;
		PIN_KEY_LIST_3 = 0;
		PIN_KEY_LIST_4 = 1;
		break;
	case 4:
		PIN_KEY_LIST_1 = 1;
		PIN_KEY_LIST_2 = 1;
		PIN_KEY_LIST_3 = 1;
		PIN_KEY_LIST_4 = 0;
		break;
	default:
		break;
	}

	//����һ��һ�е�ɨ�裬��ȡÿ�ж˿ڵ�ֵ�����ڰ�������������
	Matrix_KEY[1 - 1][KEY_LIST_INDEX - 1] = PIN_KEY_LINE_1;
	Matrix_KEY[2 - 1][KEY_LIST_INDEX - 1] = PIN_KEY_LINE_2;
	Matrix_KEY[3 - 1][KEY_LIST_INDEX - 1] = PIN_KEY_LINE_3;
	Matrix_KEY[4 - 1][KEY_LIST_INDEX - 1] = PIN_KEY_LINE_4;
	Matrix_KEY[5 - 1][KEY_LIST_INDEX - 1] = PIN_KEY_LINE_5;
	Matrix_KEY[6 - 1][KEY_LIST_INDEX - 1] = PIN_KEY_LINE_6;
		 
}





//LED�����нӿڶ���
//sbit PIN_LED_LIST_1 = P3 ^ 4;
//sbit PIN_LED_LIST_2 = P3 ^ 5;
//sbit PIN_LED_LIST_3 = P3 ^ 6;
//sbit PIN_LED_LIST_4 = P3 ^ 7;
////LED�����нӿڶ���
//sbit PIN_LED_LINE_1 = P0 ^ 0;
//sbit PIN_LED_LINE_2 = P0 ^ 1;
//sbit PIN_LED_LINE_3 = P0 ^ 2;
//sbit PIN_LED_LINE_4 = P0 ^ 3;
//sbit PIN_LED_LINE_5 = P0 ^ 4;
//sbit PIN_LED_LINE_6 = P0 ^ 5;

		
//		LED_LIST_INDEX++;			 //ѡ����
//		limit(LED_LIST_INDEX, 1, 4); //���Ʊ�����1-4��Χ�ڣ�����4���Ϊ1

//		//����ÿ����1һλ�����ѭ��
//		switch (LED_LIST_INDEX)
//		{
//		case 1:
//			PIN_LED_LIST_1 = 1;
//			PIN_LED_LIST_2 = 0;
//			PIN_LED_LIST_3 = 0;
//			PIN_LED_LIST_4 = 0;
//			break;
//		case 2:
//			PIN_LED_LIST_1 = 0;
//			PIN_LED_LIST_2 = 1;
//			PIN_LED_LIST_3 = 0;
//			PIN_LED_LIST_4 = 0;
//			break;
//		case 3:
//			PIN_LED_LIST_1 = 0;
//			PIN_LED_LIST_2 = 0;
//			PIN_LED_LIST_3 = 1;
//			PIN_LED_LIST_4 = 0;
//			break;
//		case 4:
//			PIN_LED_LIST_1 = 0;
//			PIN_LED_LIST_2 = 0;
//			PIN_LED_LIST_3 = 0;
//			PIN_LED_LIST_4 = 1;
//			break;
//		default:
//			break;
//		}

//		//��LED�����������һ����ÿ�е�ֵ��������
//		PIN_LED_LINE_1 = Matrix_LED[1 - 1][LED_LIST_INDEX - 1];
//		PIN_LED_LINE_2 = Matrix_LED[2 - 1][LED_LIST_INDEX - 1];
//		PIN_LED_LINE_3 = Matrix_LED[3 - 1][LED_LIST_INDEX - 1];
//		PIN_LED_LINE_4 = Matrix_LED[4 - 1][LED_LIST_INDEX - 1];
//		PIN_LED_LINE_5 = Matrix_LED[5 - 1][LED_LIST_INDEX - 1];
//		PIN_LED_LINE_6 = Matrix_LED[6 - 1][LED_LIST_INDEX - 1];

//		//ʵʱ��ⰴ����״̬���������¶�Ӧ��LED����
//		for (i = 0; i < 6; i++)
//		{
//			for (j = 0; j < 4; j++)
//			{
//				if (Matrix_KEY[i][j] == 0)
//				{
//					Matrix_LED[i][j] = 0;
//				}
//			}
//		}

