
/*************	功能说明	**************
因为内存空间和定时器资源的问题 普通的STC89C52RC已经不满足需要
该设计选用 STC15F2K60S2 该芯片和STC89C51引脚外观都完全一致 不同的是内部资源 可以完全替换c51

使用了3个定时器:
定时器0中断用来循环刷新LED点阵的状态 用作显示功能
定时器1中断用来循环检测矩阵键盘的状态，用来记录和判断哪一个按键按下去了
定时器2中断用作串口的波特率产生 由寄存器来配置

串口1全双工中断方式收发通讯程序 使用22.1184MHZ时钟 如要改变 请修改下面的"定义主时钟"的值并重新编译
串口设置为：115200,8,n,1.

在主循环中：
检测矩阵键盘数组中的状态 赋值给LED矩阵数组
检测串口接收的数组中的数字与预制的条码数组对比 得到当前扫描的条码对应的柜门序号 并关闭LED灯表示柜门打开
******************************************/
#include "STC15Fxxxx.H"
#include <intrins.h>
#define uchar unsigned char
#define uint  unsigned int
	
#define constrain(amt, low, high) ((amt) < (low) ? (amt = low) : ((amt) > (high) ? (amt = high) : (amt = amt)))
#define limit(amt, low, high) ((amt) < (low) ? (amt = high) : ((amt) > (high) ? (amt = low) : (amt = amt)))

/*************	本地常量声明	**************/
//#define MAIN_Fosc 22118400L //定义主时钟

#define MAIN_Fosc 12000000L //定义主时钟

#define RX1_Lenth 32	   //串口接收缓冲长度
#define BaudRate1 9600UL //选择波特率
#define BaudRate2 9600UL //选择波特率

#define Timer2_Reload (65536UL - (MAIN_Fosc / 4 / BaudRate2)) //Timer2 300KHZ

/*************	本地变量声明	**************/
u8 idata RX1_Buffer[RX1_Lenth]; //接收缓冲
u8 TX1_Cnt;						//发送计数
u8 RX1_Cnt;						//接收计数
bit B_TX1_Busy;					//发送忙标志

//条形码的值对应的序号矩阵
u32 BarCode_Table[24] = {90311017, 90311024, 90311031, 90311048,
						 90311055, 90311062, 90311079, 90311086,
						 90311093, 90311109, 90311116, 90311123,
						 90311130, 90311147, 90311154, 90311161,
						 90311178, 90311185, 90311192, 90311208,
						 90311215, 90311222, 90311239, 90311246};

//
//定义Max7219端口
sbit Max7219_pinCLK = P2^2;
sbit Max7219_pinCS  = P2^1;
sbit Max7219_pinDIN = P2^0;
						 
u8 LED_LIST_INDEX = 1; //LED矩阵的列索引
u8 LED_LINE_INDEX = 1; //LED矩阵的行索引
//LED矩阵数组，6行4列，亮为0，灭为1
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
//KEY矩阵列接口定义
sbit PIN_KEY_LIST_1 = P1 ^ 4;
sbit PIN_KEY_LIST_2 = P1 ^ 5;
sbit PIN_KEY_LIST_3 = P1 ^ 6;
sbit PIN_KEY_LIST_4 = P1 ^ 7;
//KEY矩阵行接口定义
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

u8 KEY_LIST_INDEX = 1; //KEY矩阵的列索引
u8 KEY_LINE_INDEX = 1; //KEY矩阵的行索引
//按键矩阵数组，6行4列，按下为0，未按下为1
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
/*************	私有变量声明	**************/
u16 i, j, k;	  //用来做索引的变量
u32 Scan_BarCode; //扫描到的条码的值
/**********************************************/

//--------------------------------------------
//功能：向MAX7219(U3)写入字节
//入口参数：DATA 
//出口参数：无
//说明：
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
//功能：向MAX7219写入数据
//入口参数：address、dat
//出口参数：无
//说明：
void Write_Max7219(uchar address,uchar dat)
{ 
   Max7219_pinCS=0;
	 Write_Max7219_byte(address);           //写入地址，即数码管编号
   Write_Max7219_byte(dat);               //写入数据，即数码管显示数字 
	 Max7219_pinCS=1;                        
}

void Init_MAX7219(void)
{
 Write_Max7219(0x09, 0x00);       //译码方式：BCD码
 Write_Max7219(0x0a, 0x03);       //亮度 
 Write_Max7219(0x0b, 0x07);       //扫描界限；8个数码管显示
 Write_Max7219(0x0c, 0x01);       //掉电模式：0，普通模式：1
 Write_Max7219(0x0f, 0x00);       //显示测试：1；测试结束，正常显示：0
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

	/**********定时器0 定时器1 配置**********/
	EA = 1;						// 设置中断允许寄存器：1.打开中断总开关
	ET0 = 1;					// 设置中断允许寄存器：2.打开定时器0
	//ET1 = 1;					// 设置中断允许寄存器：2.打开定时器1
	TMOD = 0x11;				// 设置工作方式寄存器TMOD：工作方式1 -> 0x01
	TH0 = (65536 - 500) / 256; // 设置定时器0定时时间1ms
	TL0 = (65536 - 500) % 256; //
	//TH1 = (65536 - 500) / 256; // 设置定时器1定时时间1ms
	//TH1 = (65536 - 500) % 256; //
	TR0 = 1;					// 设置控制寄存器：启动定时器0
	//TR1 = 1;					// 设置控制寄存器：启动定时器1
	EA = 1;		  //允许全局中断
	

//	/**********串口1配置**********/
//	S1_8bit();			//8位数据
//	S1_USE_P30P31();	//UART1 使用P30 P31口	默认
//	Timer2_Stop();		//禁止定时器2计数
//	S1_BRT_UseTimer2(); //波特率使用Timer2产生
//	Timer2_1T();		//Timer2 set as 1T mode
//	TH2 = (u8)(Timer2_Reload >> 8);
//	TL2 = (u8)Timer2_Reload;
//	Timer2_Run(); //允许定时器2计数
//	REN = 1;	  //允许接收
//	ES = 1;		  //允许串行中断

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
	
	EA = 1;		  //允许全局中断
	
	Init_MAX7219();  

	while (1)
	{
		
//		if(TX1_Cnt != RX1_Cnt)		//收到过数据
//		{
//			if(!B_TX1_Busy)		//发送空闲
//			{
//				B_TX1_Busy = 1;		//标志发送忙
//				SBUF = RX1_Buffer[TX1_Cnt];	//发一个字节
//				if(++TX1_Cnt >= RX1_Lenth)	TX1_Cnt = 0;	//避免溢出处理
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
//		putc_to_SerialPort(' '); //这个地方‘’间只能有且必须有一个空格
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
		
		
//		//如果串口接收到的数据前5位是90311，则认为接收到的数据是对的
//		if (RX1_Buffer[0] == 9 && RX1_Buffer[1] == 0 && RX1_Buffer[2] == 3 &&
//			RX1_Buffer[3] == 1 && RX1_Buffer[4] == 1)
//		{
//			//将数据累加起来
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
//				//判断哪一个值与定义好的数组相等，数组的序号就是这个柜子的序号
//				if (BarCode_Table[k] == Scan_BarCode)
//				{
//					Matrix_LED[(k / 4 + 1) - 1][k - 1] = 1; //关闭LED灯
//					Scan_BarCode = 0;						//扫描到的条码的值清零
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


///*********** UART1中断函数 使用了定时器2 **************/
//void UART1_int(void) interrupt UART1_VECTOR
//{
//	if (RI) //接收中断标志位
//	{
//		RI = 0;						//清除接收中断标志位
//		RX1_Buffer[RX1_Cnt] = SBUF; //保存一个字节
//		if (++RX1_Cnt >= RX1_Lenth)
//			RX1_Cnt = 0; //避免溢出处理
//	}

//	if (TI) //发送中断标志位
//	{
//		TI = 0;			//清除发送中断标志位
//		B_TX1_Busy = 0; //清除发送忙标志
//	}
//}

#define timer0Period 50
int timer0PeriodCount = 0;
/*********** 定时器0中断函数 **************/
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

/*********** 定时器1中断函数 **************/
void Timer1() interrupt 3
{
	TH1 = (65536 - 500) / 256;
	TL1 = (65536 - 500) % 256;

	KEY_LIST_INDEX++;			 //选择列
	limit(KEY_LIST_INDEX, 1, 4); //限制变量在1-4范围内，超出4则变为1

	//将列每次置1一位，如此循环
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

	//按键一列一列的扫描，读取每列端口的值储存在按键矩阵数组中
	Matrix_KEY[1 - 1][KEY_LIST_INDEX - 1] = PIN_KEY_LINE_1;
	Matrix_KEY[2 - 1][KEY_LIST_INDEX - 1] = PIN_KEY_LINE_2;
	Matrix_KEY[3 - 1][KEY_LIST_INDEX - 1] = PIN_KEY_LINE_3;
	Matrix_KEY[4 - 1][KEY_LIST_INDEX - 1] = PIN_KEY_LINE_4;
	Matrix_KEY[5 - 1][KEY_LIST_INDEX - 1] = PIN_KEY_LINE_5;
	Matrix_KEY[6 - 1][KEY_LIST_INDEX - 1] = PIN_KEY_LINE_6;
		 
}





//LED点阵列接口定义
//sbit PIN_LED_LIST_1 = P3 ^ 4;
//sbit PIN_LED_LIST_2 = P3 ^ 5;
//sbit PIN_LED_LIST_3 = P3 ^ 6;
//sbit PIN_LED_LIST_4 = P3 ^ 7;
////LED点阵行接口定义
//sbit PIN_LED_LINE_1 = P0 ^ 0;
//sbit PIN_LED_LINE_2 = P0 ^ 1;
//sbit PIN_LED_LINE_3 = P0 ^ 2;
//sbit PIN_LED_LINE_4 = P0 ^ 3;
//sbit PIN_LED_LINE_5 = P0 ^ 4;
//sbit PIN_LED_LINE_6 = P0 ^ 5;

		
//		LED_LIST_INDEX++;			 //选择列
//		limit(LED_LIST_INDEX, 1, 4); //限制变量在1-4范围内，超出4则变为1

//		//将列每次置1一位，如此循环
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

//		//将LED矩阵数组里的一列中每行的值付给引脚
//		PIN_LED_LINE_1 = Matrix_LED[1 - 1][LED_LIST_INDEX - 1];
//		PIN_LED_LINE_2 = Matrix_LED[2 - 1][LED_LIST_INDEX - 1];
//		PIN_LED_LINE_3 = Matrix_LED[3 - 1][LED_LIST_INDEX - 1];
//		PIN_LED_LINE_4 = Matrix_LED[4 - 1][LED_LIST_INDEX - 1];
//		PIN_LED_LINE_5 = Matrix_LED[5 - 1][LED_LIST_INDEX - 1];
//		PIN_LED_LINE_6 = Matrix_LED[6 - 1][LED_LIST_INDEX - 1];

//		//实时监测按键的状态，按键按下对应的LED灯亮
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

