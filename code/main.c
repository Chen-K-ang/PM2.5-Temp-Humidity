#include <reg52.h>
#include <intrins.h>
#include "ADC0832.h"
#include "SERVO.h"
#define uchar unsigned char		// 以后unsigned char就可以用uchar代替
#define uint  unsigned int		// 以后unsigned int 就可以用uint 代替

sfr ISP_DATA  = 0xe2;			// 数据寄存器
sfr ISP_ADDRH = 0xe3;			// 地址寄存器高八位
sfr ISP_ADDRL = 0xe4;			// 地址寄存器低八位
sfr ISP_CMD   = 0xe5;			// 命令寄存器
sfr ISP_TRIG  = 0xe6;			// 命令触发寄存器
sfr ISP_CONTR = 0xe7;			// 命令寄存器

sbit LcdRs_P   = P1^2;    // 1602液晶的RS管脚       
sbit LcdRw_P   = P1^3;    // 1602液晶的RW管脚 
sbit LcdEn_P   = P1^4;    // 1602液晶的EN管脚
sbit KeySet_P  = P3^2;		// “设置”按键的管脚
sbit KeyDown_P = P3^3;		// “减”按键的管脚
sbit KeyUp_P   = P3^4;		// “加”按键的管脚 
sbit Buzzer_P  = P1^5;		// 蜂鸣器
sbit DHT11_P   = P1^1;	 	// 温湿度传感器DHT11数据接入
sbit LedTH_P   = P2^0;		// 温度过高报警指示灯
sbit LedTL_P   = P2^1;		// 温度过低报警指示灯
sbit LedHH_P   = P2^2;		// 湿度过高报警指示灯
sbit LedHL_P   = P2^3;		// 湿度过低报警指示灯
sbit LedPM_P   = P2^4;		// PM2.5过高报警指示灯

uchar temp;								// 保存温度
uchar humi;								// 保存湿度
uint  pm;									// 保存PM2.5

uchar gIndex=0;						// 串口接收索引
uint  Value[20]={0};			// 串口数据缓存区

uchar AlarmTL;						// 温度下限报警值
uchar AlarmTH;						// 温度上限报警值
uchar AlarmHL; 						// 湿度下限报警值
uchar AlarmHH;						// 湿度上限报警值
uint  AlarmPM;						// PM2.5报警值



/*********************************************************/
// 单片机内部EEPROM不使能
/*********************************************************/
void ISP_Disable()
{
	ISP_CONTR = 0;
	ISP_ADDRH = 0;
	ISP_ADDRL = 0;
}


/*********************************************************/
// 从单片机内部EEPROM读一个字节，从0x2000地址开始
/*********************************************************/
unsigned char EEPROM_Read(unsigned int add)
{
	ISP_DATA  = 0x00;
	ISP_CONTR = 0x83;
	ISP_CMD   = 0x01;
	ISP_ADDRH = (unsigned char)(add>>8);
	ISP_ADDRL = (unsigned char)(add&0xff);
	// 对STC89C51系列来说，每次要写入0x46，再写入0xB9,ISP/IAP才会生效
	ISP_TRIG  = 0x46;	   
	ISP_TRIG  = 0xB9;
	_nop_();
	ISP_Disable();
	return (ISP_DATA);
}


/*********************************************************/
// 往单片机内部EEPROM写一个字节，从0x2000地址开始
/*********************************************************/
void EEPROM_Write(unsigned int add,unsigned char ch)
{
	ISP_CONTR = 0x83;
	ISP_CMD   = 0x02;
	ISP_ADDRH = (unsigned char)(add>>8);
	ISP_ADDRL = (unsigned char)(add&0xff);
	ISP_DATA  = ch;
	ISP_TRIG  = 0x46;
	ISP_TRIG  = 0xB9;
	_nop_();
	ISP_Disable();
}


/*********************************************************/
// 擦除单片机内部EEPROM的一个扇区
// 写8个扇区中随便一个的地址，便擦除该扇区，写入前要先擦除
/*********************************************************/
void Sector_Erase(unsigned int add)	  
{
	ISP_CONTR = 0x83;
	ISP_CMD   = 0x03;
	ISP_ADDRH = (unsigned char)(add>>8);
	ISP_ADDRL = (unsigned char)(add&0xff);
	ISP_TRIG  = 0x46;
	ISP_TRIG  = 0xB9;
	_nop_();
	ISP_Disable();
}


/*********************************************************/
// 毫秒级的延时函数，time是要延时的毫秒数
/*********************************************************/
void DelayMs(uint time)
{
	uint i,j;
	for(i=0;i<time;i++)
		for(j=0;j<112;j++);
}


/*********************************************************/
// 10us级延时程序
/*********************************************************/
void Delay10us()
{
	_nop_();	// 执行一条指令，延时1微秒
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	_nop_();
}


/*********************************************************/
// 读取DHT11单总线上的一个字节
/*********************************************************/
uchar DhtReadByte(void)
{   
	bit bit_i; 
	uchar j;
	uchar dat=0;

	for(j=0;j<8;j++)    
	{
		while(!DHT11_P);	// 等待低电平结束	
		Delay10us();			// 延时
		Delay10us();
		Delay10us();
		if(DHT11_P==1)		// 判断数据线是高电平还是低电平
		{
			bit_i=1; 
			while(DHT11_P);
		} 
		else
		{
			bit_i=0;
		}
		dat<<=1;		   		// 将该位移位保存到dat变量中
		dat|=bit_i;    
	}
	return(dat);  
}


/*********************************************************/
// 读取DHT11的一帧数据，湿高、湿低(0)、温高、温低(0)、校验码
/*********************************************************/
void ReadDhtData()
{    	 
	uchar HumiHig;		// 湿度高检测值
	uchar HumiLow;		// 湿度低检测值 
	uchar TemHig;			// 温度高检测值
	uchar TemLow;			// 温度低检测值
	uchar check;			// 校验字节 
	
	DHT11_P=0;				// 主机拉低
	DelayMs(20);			// 保持20毫秒
	DHT11_P=1;				// DATA总线由上拉电阻拉高

	Delay10us();	 		// 延时等待30us
	Delay10us();
	Delay10us();

	while(!DHT11_P);	// 等待DHT的低电平结束
	while(DHT11_P);		// 等待DHT的高电平结束

	//进入数据接收状态
	HumiHig = DhtReadByte(); 	// 湿度高8位
	HumiLow = DhtReadByte(); 	// 湿度低8为，总为0
	TemHig  = DhtReadByte(); 	// 温度高8位 
	TemLow  = DhtReadByte(); 	// 温度低8为，总为0 
	check   = DhtReadByte();	// 8位校验码，其值等于读出的四个字节相加之和的低8位

	DHT11_P=1;				// 拉高总线

	if(check==HumiHig + HumiLow + TemHig + TemLow) 		// 如果收到的数据无误
	{
		temp=TemHig; 			// 将温度的检测结果赋值给全局变量temp
		humi=HumiHig;			// 将湿度的检测结果赋值给全局变量humi
	}
}



/*********************************************************/
// 1602液晶写命令函数，cmd就是要写入的命令
/*********************************************************/
void LcdWriteCmd(uchar cmd)
{ 
	LcdRs_P = 0;
	LcdRw_P = 0;
	LcdEn_P = 0;
	P0=cmd;
	DelayMs(2);
	LcdEn_P = 1;    
	DelayMs(2);
	LcdEn_P = 0;	
}


/*********************************************************/
// 1602液晶写数据函数，dat就是要写入的数据
/*********************************************************/
void LcdWriteData(uchar dat)
{
	LcdRs_P = 1; 
	LcdRw_P = 0;
	LcdEn_P = 0;
	P0=dat;
	DelayMs(2);
	LcdEn_P = 1;    
	DelayMs(2);
	LcdEn_P = 0;
}


/*********************************************************/
// 1602液晶初始化函数
/*********************************************************/
void LcdInit()
{
	LcdWriteCmd(0x38);        // 16*2显示，5*7点阵，8位数据口
	LcdWriteCmd(0x0C);        // 开显示，不显示光标
	LcdWriteCmd(0x06);        // 地址加1，当写入数据后光标右移
	LcdWriteCmd(0x01);        // 清屏
}


/*********************************************************/
// 液晶光标定位函数
/*********************************************************/
void LcdGotoXY(uchar line,uchar column)
{
	// 第一行
	if(line==0)        
		LcdWriteCmd(0x80+column); 
	 // 第二行
	if(line==1)        
		LcdWriteCmd(0x80+0x40+column); 
}


/*********************************************************/
// 液晶输出数字（PM2.5的）
/*********************************************************/
void LcdPrintNum1(uint num)
{
	LcdWriteData(num/100+48);					// 百位
	LcdWriteData(num%100/10+48);			// 十位
	LcdWriteData(num%10+48); 					// 个位
}


/*********************************************************/
// 液晶输出数字（温湿度的）
/*********************************************************/
void LcdPrintNum2(uchar num)
{
	LcdWriteData(num/10+48);					// 十位
	LcdWriteData(num%10+48); 					// 个位
}



/*********************************************************/
// 液晶输出字符串函数
/*********************************************************/
void LcdPrintStr(uchar *str)
{
	while(*str!='\0')
		LcdWriteData(*str++);
}


/*********************************************************/
// 液晶显示内容初始化
/*********************************************************/
void LcdShowInit()
{
	LcdGotoXY(0,0);										// 液晶光标定位到第0行第0列
	LcdPrintStr("PM2.5:    ug/m3 ");	// 显示内容
	LcdGotoXY(1,0);										// 液晶光标定位到第1行第0列
	LcdPrintStr("T:   C   H:  %RH");	// 显示内容
	LcdGotoXY(1,4);										// 温度单位摄氏度上面的圆圈符号
	LcdWriteData(0xdf);	
}

/*********************************************************/
// 按键扫描
/*********************************************************/
void KeyScanf()
{
	if(KeySet_P==0)		// 判断设置按键是否被按下
	{
		EA=0;
		
		/*将液晶显示改为设置温度的页面****************************************************/
		LcdWriteCmd(0x01);				  	
		LcdGotoXY(0,0);
		LcdPrintStr("Temperature Set ");
		LcdGotoXY(1,0);
		LcdPrintStr("      -    C    ");
		LcdGotoXY(1,10);	 					
		LcdWriteData(0xdf);			

		LcdGotoXY(1,4);	 					// 在液晶上填充温度的下限值	
		LcdPrintNum2(AlarmTL);	
		LcdGotoXY(1,7);	 					// 在液晶上填充温度的上限值
		LcdPrintNum2(AlarmTH);

		LcdGotoXY(1,5);	 					// 光标定位到第1行第5列
		LcdWriteCmd(0x0f);				// 光标闪烁
		
		DelayMs(10);	  					// 去除按键按下的抖动
		while(!KeySet_P);	 				// 等待按键释放
		DelayMs(10);					  	// 去除按键松开的抖动

		/*设置温度的下限值****************************************************************/
		while(KeySet_P)						// “设置键”没有被按下，则一直处于温度下限的设置
		{
			if(KeyDown_P==0)				// 判断 “减按键“ 是否被按下		
			{
				if(AlarmTL>0)					// 只有当温度下限值大于0时，才能减1
					AlarmTL--;
				LcdGotoXY(1,4);	 			// 重新刷新显示更改后的温度下限值	
				LcdPrintNum2(AlarmTL);  		
				LcdGotoXY(1,5);				// 重新定位闪烁的光标位置
				DelayMs(350);					// 延时
			}
			if(KeyUp_P==0)		  		// 判断 “加按键“ 是否被按下
			{
				if(AlarmTL<99)	   		// 只有当温度下限值小于99时，才能加1
					AlarmTL++;
				LcdGotoXY(1,4);	 	 		// 重新刷新显示更改后的温度下限值
				LcdPrintNum2(AlarmTL);
				LcdGotoXY(1,5);				// 重新定位闪烁的光标位置
				DelayMs(350);					// 延时
			}	
		}

		LcdGotoXY(1,8);
		DelayMs(10);	  					// 去除按键按下的抖动
		while(!KeySet_P);	 				// 等待按键释放
		DelayMs(10);					  	// 去除按键松开的抖动

		/*设置温度的上限值****************************************************************/	
		while(KeySet_P)	  				// “设置键”没有被按下，则一直处于温度上限的设置
		{
			if(KeyDown_P==0)				// 判断 “减按键“ 是否被按下
			{
				if(AlarmTH>0)  				// 只有当温度上限值大于0时，才能减1			
					AlarmTH--;
				LcdGotoXY(1,7);	 	  	// 重新刷新显示更改后的温度上限值
				LcdPrintNum2(AlarmTH);
				LcdGotoXY(1,8);				// 重新定位闪烁的光标位置
				DelayMs(350);					// 延时
			}
			if(KeyUp_P==0)			   	// 判断 “加按键“ 是否被按下
			{
				if(AlarmTH<99)	 			// 只有当温度上限值小于99时，才能加1
					AlarmTH++;
				LcdGotoXY(1,7);				// 重新刷新显示更改后的温度上限值 	
				LcdPrintNum2(AlarmTH);
				LcdGotoXY(1,8);				// 重新定位闪烁的光标位置
				DelayMs(350);					// 延时
			}								 
		}

		/*将液晶显示改为设置湿度的页面****************************************************/
		LcdWriteCmd(0x01);				  	
		LcdGotoXY(0,0);
		LcdPrintStr("  Humidity Set  ");
		LcdGotoXY(1,0);
		LcdPrintStr("      -   %RH   ");		

		LcdGotoXY(1,4);	 					// 在液晶上填充湿度的下限值	
		LcdPrintNum2(AlarmHL);	
		LcdGotoXY(1,7);	 					// 在液晶上填充湿度的上限值
		LcdPrintNum2(AlarmHH);

		LcdGotoXY(1,5);	 					// 光标定位到第1行第5列
		
		DelayMs(10);	  					// 去除按键按下的抖动
		while(!KeySet_P);	 				// 等待按键释放
		DelayMs(10);
		
		/*设置湿度的下限值****************************************************************/
		while(KeySet_P)				 		// “设置键”没有被按下，则一直处于湿度下限的设置
		{
			if(KeyDown_P==0)				// 判断 “减按键“ 是否被按下
			{
				if(AlarmHL>0)	 				// 只有当湿度下限值大于0时，才能减1
					AlarmHL--;
				LcdGotoXY(1,4);				// 重新刷新显示更改后的湿度下限值 	
				LcdPrintNum2(AlarmHL);
				LcdGotoXY(1,5);				// 重新定位闪烁的光标位置
				DelayMs(350);
			}
			if(KeyUp_P==0)			  	// 判断 “加按键“ 是否被按下
			{
				if(AlarmHL<99)	  		// 只有当湿度下限值小于99时，才能加1
					AlarmHL++;
				LcdGotoXY(1,4);	 		 	// 重新刷新显示更改后的湿度下限值
				LcdPrintNum2(AlarmHL);
				LcdGotoXY(1,5);	  		// 重新定位闪烁的光标位置
				DelayMs(350);					// 延时
			}	
		}

		LcdGotoXY(1,8);
		DelayMs(10);	  					// 去除按键按下的抖动
		while(!KeySet_P);	 				// 等待按键释放
		DelayMs(10);					  	// 去除按键松开的抖动
		
		/*设置湿度的上限值****************************************************************/
		while(KeySet_P)				   	// “设置键”没有被按下，则一直处于湿度上限的设置
		{
			if(KeyDown_P==0)		 		// 判断 “减按键“ 是否被按下
			{
				if(AlarmHH>0)			  	// 只有当湿度上限值大于0时，才能减1
					AlarmHH--;
				LcdGotoXY(1,7);	 		 	// 重新刷新显示更改后的湿度上限值
				LcdPrintNum2(AlarmHH);
				LcdGotoXY(1,8);		   	// 重新定位闪烁的光标位置
				DelayMs(350);
			}
			if(KeyUp_P==0)				 	// 判断 “加按键“ 是否被按下
			{
				if(AlarmHH<99)				// 只有当湿度上限值小于99时，才能加1
					AlarmHH++;
				LcdGotoXY(1,7);	 			// 重新刷新显示更改后的湿度上限值	
				LcdPrintNum2(AlarmHH);
				LcdGotoXY(1,8);	 			// 重新定位闪烁的光标位置
				DelayMs(350);					// 延时
			}	
		}

		/*将液晶显示改为设置PM2.5的页面****************************************************/
		LcdWriteCmd(0x01);				  	// 设置界面的显示框架
		LcdGotoXY(0,0);
		LcdPrintStr("   PM2.5 Set    ");
		LcdGotoXY(1,0);
		LcdPrintStr("        ug/m3   ");
		LcdGotoXY(1,4);								// 显示当前的报警值
		LcdPrintNum1(AlarmPM);				

		LcdGotoXY(1,6);	 							// 光标定位到第1行第6列
		DelayMs(10);	  							// 去除按键按下的抖动
		while(!KeySet_P);	 						// 等待按键释放
		DelayMs(10);
		
		while(KeySet_P)				 				// “设置键”没有被按下，则一直处于光强下限的设置
		{
			if(KeyDown_P==0)						// 判断 “减按键“ 是否被按下
			{
				if(AlarmPM>1)							// 只有gAlarmPM大于1才能减1								
					AlarmPM--;				
				LcdGotoXY(1,4);						// 液晶光标定位
				LcdPrintNum1(AlarmPM);		// 刷新改变后的报警值
				LcdGotoXY(1,6);
				DelayMs(200);							// 延时一下
			}
			if(KeyUp_P==0)			  			// 判断 “加按键“ 是否被按下
			{
			if(AlarmPM<999)							// 只有gAlarmPM小于999才能加1
					AlarmPM++;				
				LcdGotoXY(1,4);						// 液晶光标定位
				LcdPrintNum1(AlarmPM);		// 刷新改变后的报警值
				LcdGotoXY(1,6);
				DelayMs(200);							// 延时一下
			}	
		}

		/*完成设置，退出前的处理**********************************************************/
		LcdWriteCmd(0x0C);	  						// 取消光标闪烁
		LcdShowInit();										// 液晶显示为检测界面的

		DelayMs(10);	  									// 去除按键按下的抖动
		while(!KeySet_P);	 								// 等待按键释放
		DelayMs(10);					  					// 去除按键松开的抖动

		Sector_Erase(0x2000);			 				// 存储之前必须先擦除
		EEPROM_Write(0x2000,AlarmTL);			// 把温度下限存入到EEPROM的0x2000这个地址
		EEPROM_Write(0x2001,AlarmTH);			// 把温度上限存入到EEPROM的0x2001这个地址
		EEPROM_Write(0x2002,AlarmHL);			// 把湿度下限存入到EEPROM的0x2002这个地址
		EEPROM_Write(0x2003,AlarmHH);			// 把湿度上限存入到EEPROM的0x2003这个地址
		EEPROM_Write(0x2004,AlarmPM/100);	// 把PM2.5存入到EEPROM的0x2004和0x2005这两个地址
		EEPROM_Write(0x2005,AlarmPM%100);
		
		EA=1;
	}	
}


/*********************************************************/
// 报警判断
/*********************************************************/
void AlarmJudge(void)
{
	/*温度*/
	if(temp>AlarmTH)				// 温度是否过高
	{
		LedTH_P=0;
		LedTL_P=1;
	}
	else if(temp<AlarmTL)		// 温度是否过低
	{
		LedTL_P=0;
		LedTH_P=1;
	}
	else										// 温度正常
	{
		LedTH_P=1;
		LedTL_P=1;
	}

	/*湿度*/
	if(humi>AlarmHH)	   		// 湿度是否过高
	{
		LedHH_P=0;
		LedHL_P=1;
		LED_H = 1;
	}
	else if(humi<AlarmHL)		// 湿度是否过低
	{
		LedHL_P=0;
		LedHH_P=1;
		LED_H = 0;
	}
	else				   					// 湿度正常
	{
		LedHH_P=1;
		LedHL_P=1;
		LED_H = 1;
	}
	
	/*PM2.5*/
	if (pm > AlarmPM) {
		IN1 = 1;
		IN2 = 0;
		servo_pwm_val =  200 * (pm - AlarmPM) / (500 - AlarmPM);
		LedPM_P = 0;
	} else {
		servo_pwm_val = 0;
		IN1 = 0;
		IN2 = 0;
		LedPM_P = 1;
	}

	/*蜂鸣器*/
	if((LedHH_P==0)||(LedHL_P==0)||(LedTH_P==0)||(LedTL_P==0)||(LedPM_P==0)) 	// 蜂鸣器判断，只要至少1个报警灯亮，蜂鸣器就报警
		Buzzer_P=0;
	else	
		Buzzer_P=1;
}


/*********************************************************/
// 报警值初始化
/*********************************************************/
void AlarmInit(void)
{
	AlarmTL=EEPROM_Read(0x2000);	// 从EEPROM的0x2000这个地址读取温度的报警下限
	AlarmTH=EEPROM_Read(0x2001);	// 从EEPROM的0x2001这个地址读取温度的报警上限
	AlarmHL=EEPROM_Read(0x2002);	// 从EEPROM的0x2002这个地址读取湿度的报警下限	
	AlarmHH=EEPROM_Read(0x2003);	// 从EEPROM的0x2003这个地址读取湿度的报警上限
	AlarmPM=EEPROM_Read(0x2004)*100+EEPROM_Read(0x2005);		// 读取PM2.5报警值
	
	if((AlarmTL==0)||(AlarmTL>100))	// 如果温度下限报警值读出来异常（等于0或大于100），则重新赋值
		AlarmTL=20;
	if((AlarmTH==0)||(AlarmTH>100))	// 如果温度上限报警值读出来异常（等于0或大于100），则重新赋值
		AlarmTH=35;
	if((AlarmHL==0)||(AlarmHL>100))	// 如果温度下限报警值读出来异常（等于0或大于100），则重新赋值
		AlarmHL=40;
	if((AlarmHH==0)||(AlarmHH>100))	// 如果温度上限报警值读出来异常（等于0或大于100），则重新赋值
		AlarmHH=85;
	if((AlarmPM==0)||(AlarmPM>1300))	// 如果读取到的报警值异常，则重新赋值
		AlarmPM=200;
}


/*********************************************************/
// 主函数
/*********************************************************/
void main(void)
{
	/****pm2.5参数****/
	float ad_show_pm = 0;
	unsigned char ad_result_pm = 0, ad_sampling_cnt = 0, pm_str[5] = {0};
	float ad_filiter_pm = 0, ad_pm = 0;
	
	/**/
	unsigned char motor_count = 0;
	
	uchar i;

	ADC0832_init(); // ad采集pm2.5初始化
	LcdInit();	// 液晶功能初始化
	LcdShowInit();	// 液晶显示初始化
	AlarmInit();	// 报警值初始化
	SERVO_time1_init();

	while(1)
	{
		/****PM2.5的读取****/
		ad_result_pm = ADC0832_conv(0x01);
		ad_filiter_pm += ad_result_pm;
		ad_sampling_cnt++;
		if (ad_sampling_cnt > 3) {
			ad_sampling_cnt = 0;
			/****瞬时流量处理与显示****/
			ad_pm = ad_filiter_pm * 0.25;
			ad_filiter_pm = 0;
			ad_show_pm = ad_pm / 256 * 500;
			pm = (uint)(ad_show_pm);
			LcdGotoXY(0,7);			// 液晶定位到第0行第8列
			LcdPrintNum1(pm);		// 显示测量结果
		}
		
		/*温湿度读取*/
		EA=0;
		ReadDhtData(); 							// 检测温湿度数据;
		EA=1;
		LcdGotoXY(1,2);	 						// 定位到要显示温度的地方
		LcdPrintNum2(temp);					// 显示温度值
		LcdGotoXY(1,11);						// 定位到要显示湿度的地方
		LcdPrintNum2(humi);					// 显示湿度值
		
		// 报警判断
		AlarmJudge();						

		/*按键扫描和延时*/
		for(i=0;i<30;i++)
		{
			KeyScanf();			// 按键判断
			DelayMs(10);
		}
	}
}


