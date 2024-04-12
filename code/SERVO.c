#include "SERVO.h"

/*舵机pwm使用定时器初始化*/
void SERVO_time1_init(void)//100微秒@12.000MHz
{
	SERVO_IO = 0;

	TMOD &= 0x0F; //设置定时器模式
	TMOD |= 0x10; //设置定时器模式
	TL1 = 0x9C;   //设置定时初值
	TH1 = 0xFF;   //设置定时初值
	TF1 = 0;      //清除TF1标志
	ET1 = 1;      //打开定时器1中断
	EA = 1;       //打开总中断
	TR1 = 1;      //定时器1开始计时
}

unsigned char servo_pwm_val = 0;
/*T1中断*/
void SERVO_pwm_ISR(void) interrupt 3
{
	static unsigned char count = 0;
	TR1 = 0; 
	TH1 = 0xFF; // 0.1ms
	TL1 = 0x9C;
	
	if (count <= servo_pwm_val)
		SERVO_IO = 1;
	else
		SERVO_IO = 0;
	count++;
	if (count >= 200) //T = 20ms清零
		count = 0;
	
	TR1 = 1; //开启T0
}

