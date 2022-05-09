/*实现功能:对颜色传感器输出RGB
使用芯片：STC89C52 
晶振：12MHZ
编译环境：Keil
作者：ylt*/
#include <reg52.h>
#define uchar unsigned char
#define uint unsigned int
//==============LCD1602接口连接方法=====================
/*-----------------------------------------------------
       |DB0-----P2.0 | DB4-----P2.4 | RW-------P0.6    |
       |DB1-----P2.1 | DB5-----P2.5 | RS-------P0.7    |
       |DB2-----P2.2 | DB6-----P2.6 | E--------P0.5    |
       |DB3-----P2.3 | DB7-----P2.7 | 
    ---------------------------------------------------*/
//================================================*/
#define LCM_Data P0   //LCD1602数据接口
#define Busy 0x80     //用于检测LCM状态字中的Busy标识
sbit LCM_RW = P2 ^ 1; //读写控制输入端，LCD1602的第五脚
sbit LCM_RS = P2 ^ 0; //寄存器选择输入端，LCD1602的第四脚
sbit LCM_E = P2 ^ 2;  //使能信号输入端,LCD1602的第6脚

sbit PIN_RELAY1 = P1 ^ 7 ;//继电器端口
sbit PIN_RELAY2 = P1 ^ 6 ;//继电器端口
sbit PIN_RELAY3 = P1 ^ 5 ;//继电器端口

//=================颜色传感模块连接=====================
/*-----------------------------------------------------
       |EO-----GND
       |S0-----VCC | S2-----P1.0 | OUT-------P3.5 
       |S1-----VCC | S3-----P1.1 | 
  ---------------------------------------------------*/
sbit tcs230_s2 = P1 ^ 0; //TCS230 S2接单片机P1.0
sbit tcs230_s3 = P1 ^ 1; //TCS230 S3接单片机P1.1
sbit tcs230_en = P1 ^ 2; //TCS230 EN(E0)接GND
//**************函数声明***************************************
void WriteDataLCM(uchar WDLCM);                     //LCD模块写数据
void WriteCommandLCM(uchar WCLCM, BuysC);           //LCD模块写指令
uchar ReadStatusLCM(void);                          //读LCD模块的忙标
void DisplayOneChar(uchar X, uchar Y, uchar ASCII); //在第X+1行的第Y+1位置显示一个字符
void LCMInit(void);                                 //LCD初始
void DelayMs(uint Ms);                              //1MS基准延时程序
void baipingheng();                                 //白平衡子程序
void celiang();                                     //实际颜色程序
uint ryz, gyz, byz;                                 //分别定义红色因子 绿色因子 蓝色因子
uint rb, gb, bb;                                    //RGB值
uchar tab1[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

//*********分选装置子函数***********
void classify()
{
	
	if((rb>bb)&&(rb>gb))//max=redValue;
	{
		PIN_RELAY2 = 1;//复位
		PIN_RELAY3 = 1;//复位
		PIN_RELAY1 = 0;//低电平触发
	}
	else if((gb>rb)&&(gb>bb))//max=greenValue;
	{
		PIN_RELAY1 = 1;//复位
		PIN_RELAY3 = 1;//复位
		PIN_RELAY2 = 0;//低电平触发
	}
	else if((bb>rb)&&(bb>gb))//max=blueValue;
	{
		PIN_RELAY1 = 1;//复位
		PIN_RELAY2 = 1;//复位
		PIN_RELAY3 = 0;//低电平触发
	}
}

//***********************主程序******************************
main()
{
  TMOD = 0x51;   //设定T0以工作方式1定时10毫秒
  LCMInit();     //LCD初始
  baipingheng(); //上电时先白平衡一次
  while (1)
  {
		classify();
    celiang();                                 //颜色测试
    DisplayOneChar(0, 0, 'R');                 //以十进制显示RGB中红色的分值
    DisplayOneChar(0, 1, rb / 100 + 0x30);     //显示百位数据
    DisplayOneChar(0, 2, rb / 10 % 10 + 0x30); //显示十位数据
    DisplayOneChar(0, 3, rb % 10 + 0x30);      //显示个位数据
    DisplayOneChar(0, 5, 'G');                 //以十进制显示RGB中绿色的分值
    DisplayOneChar(0, 6, gb / 100 + 0x30);     //显示百位数据
    DisplayOneChar(0, 7, gb / 10 % 10 + 0x30);
    DisplayOneChar(0, 8, gb % 10 + 0x30);
    DisplayOneChar(0, 10, 'B'); //以十进制显示RGB中蓝色的分值
    DisplayOneChar(0, 11, bb / 100 + 0x30);
    DisplayOneChar(0, 12, bb / 10 % 10 + 0x30);
    DisplayOneChar(0, 13, bb % 10 + 0x30);
    //*****在LCD1602的第二行以16进制显示RGB*******************
    DisplayOneChar(1, 1, tab1[rb / 16]);
    DisplayOneChar(1, 2, tab1[rb % 16]);
    DisplayOneChar(1, 3, 'H');
    DisplayOneChar(1, 6, tab1[gb / 16]);
    DisplayOneChar(1, 7, tab1[rb % 16]);
    DisplayOneChar(1, 8, 'H');
    DisplayOneChar(1, 11, tab1[bb / 16]);
    DisplayOneChar(1, 12, tab1[bb % 16]);
    DisplayOneChar(1, 13, 'H');
    DelayMs(50); //每隔0.25秒测试一次颜色
  }
}
//******************************************************
//白平衡子程序
void celiang()
{
  //*********求R值************************************
  TH0 = (65536 - 10000) / 256;
  TL0 = (65536 - 10000) % 256;
  TH1 = 0;
  TL1 = 0;
  tcs230_s2 = 0;
  tcs230_s3 = 0; //选择红色滤光器
  tcs230_en = 0;
  TR0 = 1; //10毫秒开始计时
  TR1 = 1; //开始计数
  while (TF0 == 0)
    ;      //等待定时器溢出
  TF0 = 0; //清楚定时器0溢出标志
  TR0 = 0; //关闭定时0
  TR1 = 0;
  rb = (unsigned long)(TH1 * 256 + TL1) * 255 / ryz;
  if (rb > 255)
    rb = 255; //判断RGB值是否合法
  //***********求B值**************************************
  TH0 = (65536 - 10000) / 256;
  TL0 = (65536 - 10000) % 256;
  TH1 = 0;
  TL1 = 0;
  tcs230_s2 = 0;
  tcs230_s3 = 1; //选择蓝色滤光器
  TR0 = 1;       //10毫秒开始计时
  TR1 = 1;       //开始计数
  while (TF0 == 0)
    ;      //等待定时器溢出
  TF0 = 0; //清楚定时器0溢出标志
  TR0 = 0; //关闭定时0
  TR1 = 0;
  bb = (unsigned long)(TH1 * 256 + TL1) * 255 / byz;
  if (bb > 255)
    bb = 255; //判断RGB值是否合法
  //***********求G值**************************************
  TH0 = (65536 - 10000) / 256;
  TL0 = (65536 - 10000) % 256;
  TH1 = 0;
  TL1 = 0;
  tcs230_s2 = 1;
  tcs230_s3 = 1; //选择绿色滤光器
  TR0 = 1;       //10毫秒开始计时
  TR1 = 1;       //开始计数
  while (TF0 == 0)
    ;      //等待定时器溢出
  TF0 = 0; //清楚定时器0溢出标志
  TR0 = 0; //关闭定时0
  TR1 = 0;
  tcs230_en = 1;
  gb = (unsigned long)(TH1 * 256 + TL1) * 255 / gyz;
  if (gb > 255)
    gb = 255; //判断RGB值是否合法
}
//******************************************************
//白平衡子程序
void baipingheng()
{
  //**************求取红色因子***********************
  TH0 = (65536 - 10000) / 256;
  TL0 = (65536 - 10000) % 256;
  TH1 = 0;
  TL1 = 0;
  tcs230_s2 = 0;
  tcs230_s3 = 0; //选择红色滤光器
  tcs230_en = 0;
  TR0 = 1; //10毫秒开始计时
  TR1 = 1; //开始计数
  while (TF0 == 0)
    ;      //等待定时器溢出
  TF0 = 0; //清楚定时器0溢出标志
  TR0 = 0; //关闭定时0
  TR1 = 0;
  ryz = TH1 * 256 + TL1; //其实这里的比例因子应该为255/(TH1*256+TL1)
  //**************求取蓝色因子***********************
  TH0 = (65536 - 10000) / 256;
  TL0 = (65536 - 10000) % 256;
  TH1 = 0;
  TL1 = 0;
  tcs230_s2 = 0;
  tcs230_s3 = 1; //选择蓝色滤光器
  TR0 = 1;       //10毫秒开始计时
  TR1 = 1;       //开始计数
  while (TF0 == 0)
    ;      //等待定时器溢出
  TF0 = 0; //清楚定时器0溢出标志
  TR0 = 0; //关闭定时0
  TR1 = 0;
  byz = TH1 * 256 + TL1; //其实这里的比例因子应该为255/(TH1*256+TL1)
  //**************求绿红色因子***********************
  TH0 = (65536 - 10000) / 256;
  TL0 = (65536 - 10000) % 256;
  TH1 = 0;
  TL1 = 0;
  tcs230_s2 = 1;
  tcs230_s3 = 1; //选择绿色滤光器
  TR0 = 1;       //10毫秒开始计时
  TR1 = 1;       //开始计数
  while (TF0 == 0)
    ;      //等待定时器溢出
  TF0 = 0; //清楚定时器0溢出标志
  TR0 = 0; //关闭定时0
  TR1 = 0;
  tcs230_en = 1;
  gyz = TH1 * 256 + TL1; //其实这里的比例因子应该为255/(TH1*256+TL1)
}
/*======================================================================
 LCM初始化
======================================================================*/
void LCMInit(void)
{
  LCM_Data = 0;
  WriteCommandLCM(0x38, 0); //三次显示模式设置，不检测忙信号
  DelayMs(5);
  WriteCommandLCM(0x38, 0);
  DelayMs(5);
  WriteCommandLCM(0x38, 0);
  DelayMs(5);
  WriteCommandLCM(0x38, 1); //显示模式设置,开始要求每次检测忙信号
  WriteCommandLCM(0x08, 1); //关闭显示
  WriteCommandLCM(0x01, 1); //显示清屏
  WriteCommandLCM(0x06, 1); // 显示光标移动设置
  WriteCommandLCM(0x0C, 1); // 显示开及光标设置
  DelayMs(100);
}
//==============================LCD1602显示子程序================================================
// 写数据函数: E =高脉冲 RS=1 RW=0
//======================================================================*/
void WriteDataLCM(uchar WDLCM)
{
  ReadStatusLCM(); //检测忙
  LCM_Data = WDLCM;
  LCM_RS = 1;
  LCM_RW = 0;
  LCM_E = 0; //若晶振速度太高可以在这后加小的延时
  LCM_E = 0; //延时
  LCM_E = 1;
}
/*====================================================================
  写指令函数: E=高脉冲 RS=0 RW=0
======================================================================*/
void WriteCommandLCM(uchar WCLCM, BuysC) //BuysC为0时忽略忙检测
{
  if (BuysC)
    ReadStatusLCM(); //根据需要检测忙
  LCM_Data = WCLCM;
  LCM_RS = 0;
  LCM_RW = 0;
  LCM_E = 0;
  LCM_E = 0;
  LCM_E = 1;
}
/*====================================================================
  正常读写操作之前必须检测LCD控制器状态:E=1 RS=0 RW=1;
  DB7: 0 LCD控制器空闲，1 LCD控制器忙。
  读状态
======================================================================*/
uchar ReadStatusLCM(void)
{
  LCM_Data = 0xFF;
  LCM_RS = 0;
  LCM_RW = 1;
  LCM_E = 0;
  LCM_E = 0;
  LCM_E = 1;
  while (LCM_Data & Busy)
    ; //检测忙信号
  return (LCM_Data);
}
/*======================================================================
功 能:     在1602 指定位置显示一个字符:第一行位置0~15,第二行16~31
说 明:     第 X 行,第 y 列  注意:字符串不能长于16个字符
======================================================================*/
void DisplayOneChar(uchar X, uchar Y, uchar ASCII)
{
  X &= 0x1;
  Y &= 0xF; //限制Y不能大于15，X不能大于1
  if (X)
    Y |= 0x40;           //当要显示第二行时地址码+0x40;
  Y |= 0x80;             // 算出指令码
  WriteCommandLCM(Y, 0); //这里不检测忙信号，发送地址码
  WriteDataLCM(ASCII);
}
/*====================================================================
  设定延时时间:x*1ms
====================================================================*/
void DelayMs(uint Ms)
{
  uint i, TempCyc;
  for (i = 0; i < Ms; i++)
  {
    TempCyc = 250;
    while (TempCyc--)
      ;
  }
}
