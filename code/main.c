#include <reg52.h>
#include <intrins.h>
#include "ADC0832.h"
#include "SERVO.h"
#define uchar unsigned char		// �Ժ�unsigned char�Ϳ�����uchar����
#define uint  unsigned int		// �Ժ�unsigned int �Ϳ�����uint ����

sfr ISP_DATA  = 0xe2;			// ���ݼĴ���
sfr ISP_ADDRH = 0xe3;			// ��ַ�Ĵ����߰�λ
sfr ISP_ADDRL = 0xe4;			// ��ַ�Ĵ����Ͱ�λ
sfr ISP_CMD   = 0xe5;			// ����Ĵ���
sfr ISP_TRIG  = 0xe6;			// ������Ĵ���
sfr ISP_CONTR = 0xe7;			// ����Ĵ���

sbit LcdRs_P   = P1^2;    // 1602Һ����RS�ܽ�       
sbit LcdRw_P   = P1^3;    // 1602Һ����RW�ܽ� 
sbit LcdEn_P   = P1^4;    // 1602Һ����EN�ܽ�
sbit KeySet_P  = P3^2;		// �����á������Ĺܽ�
sbit KeyDown_P = P3^3;		// �����������Ĺܽ�
sbit KeyUp_P   = P3^4;		// ���ӡ������Ĺܽ� 
sbit Buzzer_P  = P1^5;		// ������
sbit DHT11_P   = P1^1;	 	// ��ʪ�ȴ�����DHT11���ݽ���
sbit LedTH_P   = P2^0;		// �¶ȹ��߱���ָʾ��
sbit LedTL_P   = P2^1;		// �¶ȹ��ͱ���ָʾ��
sbit LedHH_P   = P2^2;		// ʪ�ȹ��߱���ָʾ��
sbit LedHL_P   = P2^3;		// ʪ�ȹ��ͱ���ָʾ��
sbit LedPM_P   = P2^4;		// PM2.5���߱���ָʾ��

uchar temp;								// �����¶�
uchar humi;								// ����ʪ��
uint  pm;									// ����PM2.5

uchar gIndex=0;						// ���ڽ�������
uint  Value[20]={0};			// �������ݻ�����

uchar AlarmTL;						// �¶����ޱ���ֵ
uchar AlarmTH;						// �¶����ޱ���ֵ
uchar AlarmHL; 						// ʪ�����ޱ���ֵ
uchar AlarmHH;						// ʪ�����ޱ���ֵ
uint  AlarmPM;						// PM2.5����ֵ



/*********************************************************/
// ��Ƭ���ڲ�EEPROM��ʹ��
/*********************************************************/
void ISP_Disable()
{
	ISP_CONTR = 0;
	ISP_ADDRH = 0;
	ISP_ADDRL = 0;
}


/*********************************************************/
// �ӵ�Ƭ���ڲ�EEPROM��һ���ֽڣ���0x2000��ַ��ʼ
/*********************************************************/
unsigned char EEPROM_Read(unsigned int add)
{
	ISP_DATA  = 0x00;
	ISP_CONTR = 0x83;
	ISP_CMD   = 0x01;
	ISP_ADDRH = (unsigned char)(add>>8);
	ISP_ADDRL = (unsigned char)(add&0xff);
	// ��STC89C51ϵ����˵��ÿ��Ҫд��0x46����д��0xB9,ISP/IAP�Ż���Ч
	ISP_TRIG  = 0x46;	   
	ISP_TRIG  = 0xB9;
	_nop_();
	ISP_Disable();
	return (ISP_DATA);
}


/*********************************************************/
// ����Ƭ���ڲ�EEPROMдһ���ֽڣ���0x2000��ַ��ʼ
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
// ������Ƭ���ڲ�EEPROM��һ������
// д8�����������һ���ĵ�ַ���������������д��ǰҪ�Ȳ���
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
// ���뼶����ʱ������time��Ҫ��ʱ�ĺ�����
/*********************************************************/
void DelayMs(uint time)
{
	uint i,j;
	for(i=0;i<time;i++)
		for(j=0;j<112;j++);
}


/*********************************************************/
// 10us����ʱ����
/*********************************************************/
void Delay10us()
{
	_nop_();	// ִ��һ��ָ���ʱ1΢��
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
// ��ȡDHT11�������ϵ�һ���ֽ�
/*********************************************************/
uchar DhtReadByte(void)
{   
	bit bit_i; 
	uchar j;
	uchar dat=0;

	for(j=0;j<8;j++)    
	{
		while(!DHT11_P);	// �ȴ��͵�ƽ����	
		Delay10us();			// ��ʱ
		Delay10us();
		Delay10us();
		if(DHT11_P==1)		// �ж��������Ǹߵ�ƽ���ǵ͵�ƽ
		{
			bit_i=1; 
			while(DHT11_P);
		} 
		else
		{
			bit_i=0;
		}
		dat<<=1;		   		// ����λ��λ���浽dat������
		dat|=bit_i;    
	}
	return(dat);  
}


/*********************************************************/
// ��ȡDHT11��һ֡���ݣ�ʪ�ߡ�ʪ��(0)���¸ߡ��µ�(0)��У����
/*********************************************************/
void ReadDhtData()
{    	 
	uchar HumiHig;		// ʪ�ȸ߼��ֵ
	uchar HumiLow;		// ʪ�ȵͼ��ֵ 
	uchar TemHig;			// �¶ȸ߼��ֵ
	uchar TemLow;			// �¶ȵͼ��ֵ
	uchar check;			// У���ֽ� 
	
	DHT11_P=0;				// ��������
	DelayMs(20);			// ����20����
	DHT11_P=1;				// DATA������������������

	Delay10us();	 		// ��ʱ�ȴ�30us
	Delay10us();
	Delay10us();

	while(!DHT11_P);	// �ȴ�DHT�ĵ͵�ƽ����
	while(DHT11_P);		// �ȴ�DHT�ĸߵ�ƽ����

	//�������ݽ���״̬
	HumiHig = DhtReadByte(); 	// ʪ�ȸ�8λ
	HumiLow = DhtReadByte(); 	// ʪ�ȵ�8Ϊ����Ϊ0
	TemHig  = DhtReadByte(); 	// �¶ȸ�8λ 
	TemLow  = DhtReadByte(); 	// �¶ȵ�8Ϊ����Ϊ0 
	check   = DhtReadByte();	// 8λУ���룬��ֵ���ڶ������ĸ��ֽ����֮�͵ĵ�8λ

	DHT11_P=1;				// ��������

	if(check==HumiHig + HumiLow + TemHig + TemLow) 		// ����յ�����������
	{
		temp=TemHig; 			// ���¶ȵļ������ֵ��ȫ�ֱ���temp
		humi=HumiHig;			// ��ʪ�ȵļ������ֵ��ȫ�ֱ���humi
	}
}



/*********************************************************/
// 1602Һ��д�������cmd����Ҫд�������
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
// 1602Һ��д���ݺ�����dat����Ҫд�������
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
// 1602Һ����ʼ������
/*********************************************************/
void LcdInit()
{
	LcdWriteCmd(0x38);        // 16*2��ʾ��5*7����8λ���ݿ�
	LcdWriteCmd(0x0C);        // ����ʾ������ʾ���
	LcdWriteCmd(0x06);        // ��ַ��1����д�����ݺ�������
	LcdWriteCmd(0x01);        // ����
}


/*********************************************************/
// Һ����궨λ����
/*********************************************************/
void LcdGotoXY(uchar line,uchar column)
{
	// ��һ��
	if(line==0)        
		LcdWriteCmd(0x80+column); 
	 // �ڶ���
	if(line==1)        
		LcdWriteCmd(0x80+0x40+column); 
}


/*********************************************************/
// Һ��������֣�PM2.5�ģ�
/*********************************************************/
void LcdPrintNum1(uint num)
{
	LcdWriteData(num/100+48);					// ��λ
	LcdWriteData(num%100/10+48);			// ʮλ
	LcdWriteData(num%10+48); 					// ��λ
}


/*********************************************************/
// Һ��������֣���ʪ�ȵģ�
/*********************************************************/
void LcdPrintNum2(uchar num)
{
	LcdWriteData(num/10+48);					// ʮλ
	LcdWriteData(num%10+48); 					// ��λ
}



/*********************************************************/
// Һ������ַ�������
/*********************************************************/
void LcdPrintStr(uchar *str)
{
	while(*str!='\0')
		LcdWriteData(*str++);
}


/*********************************************************/
// Һ����ʾ���ݳ�ʼ��
/*********************************************************/
void LcdShowInit()
{
	LcdGotoXY(0,0);										// Һ����궨λ����0�е�0��
	LcdPrintStr("PM2.5:    ug/m3 ");	// ��ʾ����
	LcdGotoXY(1,0);										// Һ����궨λ����1�е�0��
	LcdPrintStr("T:   C   H:  %RH");	// ��ʾ����
	LcdGotoXY(1,4);										// �¶ȵ�λ���϶������ԲȦ����
	LcdWriteData(0xdf);	
}

/*********************************************************/
// ����ɨ��
/*********************************************************/
void KeyScanf()
{
	if(KeySet_P==0)		// �ж����ð����Ƿ񱻰���
	{
		EA=0;
		
		/*��Һ����ʾ��Ϊ�����¶ȵ�ҳ��****************************************************/
		LcdWriteCmd(0x01);				  	
		LcdGotoXY(0,0);
		LcdPrintStr("Temperature Set ");
		LcdGotoXY(1,0);
		LcdPrintStr("      -    C    ");
		LcdGotoXY(1,10);	 					
		LcdWriteData(0xdf);			

		LcdGotoXY(1,4);	 					// ��Һ��������¶ȵ�����ֵ	
		LcdPrintNum2(AlarmTL);	
		LcdGotoXY(1,7);	 					// ��Һ��������¶ȵ�����ֵ
		LcdPrintNum2(AlarmTH);

		LcdGotoXY(1,5);	 					// ��궨λ����1�е�5��
		LcdWriteCmd(0x0f);				// �����˸
		
		DelayMs(10);	  					// ȥ���������µĶ���
		while(!KeySet_P);	 				// �ȴ������ͷ�
		DelayMs(10);					  	// ȥ�������ɿ��Ķ���

		/*�����¶ȵ�����ֵ****************************************************************/
		while(KeySet_P)						// �����ü���û�б����£���һֱ�����¶����޵�����
		{
			if(KeyDown_P==0)				// �ж� ���������� �Ƿ񱻰���		
			{
				if(AlarmTL>0)					// ֻ�е��¶�����ֵ����0ʱ�����ܼ�1
					AlarmTL--;
				LcdGotoXY(1,4);	 			// ����ˢ����ʾ���ĺ���¶�����ֵ	
				LcdPrintNum2(AlarmTL);  		
				LcdGotoXY(1,5);				// ���¶�λ��˸�Ĺ��λ��
				DelayMs(350);					// ��ʱ
			}
			if(KeyUp_P==0)		  		// �ж� ���Ӱ����� �Ƿ񱻰���
			{
				if(AlarmTL<99)	   		// ֻ�е��¶�����ֵС��99ʱ�����ܼ�1
					AlarmTL++;
				LcdGotoXY(1,4);	 	 		// ����ˢ����ʾ���ĺ���¶�����ֵ
				LcdPrintNum2(AlarmTL);
				LcdGotoXY(1,5);				// ���¶�λ��˸�Ĺ��λ��
				DelayMs(350);					// ��ʱ
			}	
		}

		LcdGotoXY(1,8);
		DelayMs(10);	  					// ȥ���������µĶ���
		while(!KeySet_P);	 				// �ȴ������ͷ�
		DelayMs(10);					  	// ȥ�������ɿ��Ķ���

		/*�����¶ȵ�����ֵ****************************************************************/	
		while(KeySet_P)	  				// �����ü���û�б����£���һֱ�����¶����޵�����
		{
			if(KeyDown_P==0)				// �ж� ���������� �Ƿ񱻰���
			{
				if(AlarmTH>0)  				// ֻ�е��¶�����ֵ����0ʱ�����ܼ�1			
					AlarmTH--;
				LcdGotoXY(1,7);	 	  	// ����ˢ����ʾ���ĺ���¶�����ֵ
				LcdPrintNum2(AlarmTH);
				LcdGotoXY(1,8);				// ���¶�λ��˸�Ĺ��λ��
				DelayMs(350);					// ��ʱ
			}
			if(KeyUp_P==0)			   	// �ж� ���Ӱ����� �Ƿ񱻰���
			{
				if(AlarmTH<99)	 			// ֻ�е��¶�����ֵС��99ʱ�����ܼ�1
					AlarmTH++;
				LcdGotoXY(1,7);				// ����ˢ����ʾ���ĺ���¶�����ֵ 	
				LcdPrintNum2(AlarmTH);
				LcdGotoXY(1,8);				// ���¶�λ��˸�Ĺ��λ��
				DelayMs(350);					// ��ʱ
			}								 
		}

		/*��Һ����ʾ��Ϊ����ʪ�ȵ�ҳ��****************************************************/
		LcdWriteCmd(0x01);				  	
		LcdGotoXY(0,0);
		LcdPrintStr("  Humidity Set  ");
		LcdGotoXY(1,0);
		LcdPrintStr("      -   %RH   ");		

		LcdGotoXY(1,4);	 					// ��Һ�������ʪ�ȵ�����ֵ	
		LcdPrintNum2(AlarmHL);	
		LcdGotoXY(1,7);	 					// ��Һ�������ʪ�ȵ�����ֵ
		LcdPrintNum2(AlarmHH);

		LcdGotoXY(1,5);	 					// ��궨λ����1�е�5��
		
		DelayMs(10);	  					// ȥ���������µĶ���
		while(!KeySet_P);	 				// �ȴ������ͷ�
		DelayMs(10);
		
		/*����ʪ�ȵ�����ֵ****************************************************************/
		while(KeySet_P)				 		// �����ü���û�б����£���һֱ����ʪ�����޵�����
		{
			if(KeyDown_P==0)				// �ж� ���������� �Ƿ񱻰���
			{
				if(AlarmHL>0)	 				// ֻ�е�ʪ������ֵ����0ʱ�����ܼ�1
					AlarmHL--;
				LcdGotoXY(1,4);				// ����ˢ����ʾ���ĺ��ʪ������ֵ 	
				LcdPrintNum2(AlarmHL);
				LcdGotoXY(1,5);				// ���¶�λ��˸�Ĺ��λ��
				DelayMs(350);
			}
			if(KeyUp_P==0)			  	// �ж� ���Ӱ����� �Ƿ񱻰���
			{
				if(AlarmHL<99)	  		// ֻ�е�ʪ������ֵС��99ʱ�����ܼ�1
					AlarmHL++;
				LcdGotoXY(1,4);	 		 	// ����ˢ����ʾ���ĺ��ʪ������ֵ
				LcdPrintNum2(AlarmHL);
				LcdGotoXY(1,5);	  		// ���¶�λ��˸�Ĺ��λ��
				DelayMs(350);					// ��ʱ
			}	
		}

		LcdGotoXY(1,8);
		DelayMs(10);	  					// ȥ���������µĶ���
		while(!KeySet_P);	 				// �ȴ������ͷ�
		DelayMs(10);					  	// ȥ�������ɿ��Ķ���
		
		/*����ʪ�ȵ�����ֵ****************************************************************/
		while(KeySet_P)				   	// �����ü���û�б����£���һֱ����ʪ�����޵�����
		{
			if(KeyDown_P==0)		 		// �ж� ���������� �Ƿ񱻰���
			{
				if(AlarmHH>0)			  	// ֻ�е�ʪ������ֵ����0ʱ�����ܼ�1
					AlarmHH--;
				LcdGotoXY(1,7);	 		 	// ����ˢ����ʾ���ĺ��ʪ������ֵ
				LcdPrintNum2(AlarmHH);
				LcdGotoXY(1,8);		   	// ���¶�λ��˸�Ĺ��λ��
				DelayMs(350);
			}
			if(KeyUp_P==0)				 	// �ж� ���Ӱ����� �Ƿ񱻰���
			{
				if(AlarmHH<99)				// ֻ�е�ʪ������ֵС��99ʱ�����ܼ�1
					AlarmHH++;
				LcdGotoXY(1,7);	 			// ����ˢ����ʾ���ĺ��ʪ������ֵ	
				LcdPrintNum2(AlarmHH);
				LcdGotoXY(1,8);	 			// ���¶�λ��˸�Ĺ��λ��
				DelayMs(350);					// ��ʱ
			}	
		}

		/*��Һ����ʾ��Ϊ����PM2.5��ҳ��****************************************************/
		LcdWriteCmd(0x01);				  	// ���ý������ʾ���
		LcdGotoXY(0,0);
		LcdPrintStr("   PM2.5 Set    ");
		LcdGotoXY(1,0);
		LcdPrintStr("        ug/m3   ");
		LcdGotoXY(1,4);								// ��ʾ��ǰ�ı���ֵ
		LcdPrintNum1(AlarmPM);				

		LcdGotoXY(1,6);	 							// ��궨λ����1�е�6��
		DelayMs(10);	  							// ȥ���������µĶ���
		while(!KeySet_P);	 						// �ȴ������ͷ�
		DelayMs(10);
		
		while(KeySet_P)				 				// �����ü���û�б����£���һֱ���ڹ�ǿ���޵�����
		{
			if(KeyDown_P==0)						// �ж� ���������� �Ƿ񱻰���
			{
				if(AlarmPM>1)							// ֻ��gAlarmPM����1���ܼ�1								
					AlarmPM--;				
				LcdGotoXY(1,4);						// Һ����궨λ
				LcdPrintNum1(AlarmPM);		// ˢ�¸ı��ı���ֵ
				LcdGotoXY(1,6);
				DelayMs(200);							// ��ʱһ��
			}
			if(KeyUp_P==0)			  			// �ж� ���Ӱ����� �Ƿ񱻰���
			{
			if(AlarmPM<999)							// ֻ��gAlarmPMС��999���ܼ�1
					AlarmPM++;				
				LcdGotoXY(1,4);						// Һ����궨λ
				LcdPrintNum1(AlarmPM);		// ˢ�¸ı��ı���ֵ
				LcdGotoXY(1,6);
				DelayMs(200);							// ��ʱһ��
			}	
		}

		/*������ã��˳�ǰ�Ĵ���**********************************************************/
		LcdWriteCmd(0x0C);	  						// ȡ�������˸
		LcdShowInit();										// Һ����ʾΪ�������

		DelayMs(10);	  									// ȥ���������µĶ���
		while(!KeySet_P);	 								// �ȴ������ͷ�
		DelayMs(10);					  					// ȥ�������ɿ��Ķ���

		Sector_Erase(0x2000);			 				// �洢֮ǰ�����Ȳ���
		EEPROM_Write(0x2000,AlarmTL);			// ���¶����޴��뵽EEPROM��0x2000�����ַ
		EEPROM_Write(0x2001,AlarmTH);			// ���¶����޴��뵽EEPROM��0x2001�����ַ
		EEPROM_Write(0x2002,AlarmHL);			// ��ʪ�����޴��뵽EEPROM��0x2002�����ַ
		EEPROM_Write(0x2003,AlarmHH);			// ��ʪ�����޴��뵽EEPROM��0x2003�����ַ
		EEPROM_Write(0x2004,AlarmPM/100);	// ��PM2.5���뵽EEPROM��0x2004��0x2005��������ַ
		EEPROM_Write(0x2005,AlarmPM%100);
		
		EA=1;
	}	
}


/*********************************************************/
// �����ж�
/*********************************************************/
void AlarmJudge(void)
{
	/*�¶�*/
	if(temp>AlarmTH)				// �¶��Ƿ����
	{
		LedTH_P=0;
		LedTL_P=1;
	}
	else if(temp<AlarmTL)		// �¶��Ƿ����
	{
		LedTL_P=0;
		LedTH_P=1;
	}
	else										// �¶�����
	{
		LedTH_P=1;
		LedTL_P=1;
	}

	/*ʪ��*/
	if(humi>AlarmHH)	   		// ʪ���Ƿ����
	{
		LedHH_P=0;
		LedHL_P=1;
		LED_H = 1;
	}
	else if(humi<AlarmHL)		// ʪ���Ƿ����
	{
		LedHL_P=0;
		LedHH_P=1;
		LED_H = 0;
	}
	else				   					// ʪ������
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

	/*������*/
	if((LedHH_P==0)||(LedHL_P==0)||(LedTH_P==0)||(LedTL_P==0)||(LedPM_P==0)) 	// �������жϣ�ֻҪ����1�������������������ͱ���
		Buzzer_P=0;
	else	
		Buzzer_P=1;
}


/*********************************************************/
// ����ֵ��ʼ��
/*********************************************************/
void AlarmInit(void)
{
	AlarmTL=EEPROM_Read(0x2000);	// ��EEPROM��0x2000�����ַ��ȡ�¶ȵı�������
	AlarmTH=EEPROM_Read(0x2001);	// ��EEPROM��0x2001�����ַ��ȡ�¶ȵı�������
	AlarmHL=EEPROM_Read(0x2002);	// ��EEPROM��0x2002�����ַ��ȡʪ�ȵı�������	
	AlarmHH=EEPROM_Read(0x2003);	// ��EEPROM��0x2003�����ַ��ȡʪ�ȵı�������
	AlarmPM=EEPROM_Read(0x2004)*100+EEPROM_Read(0x2005);		// ��ȡPM2.5����ֵ
	
	if((AlarmTL==0)||(AlarmTL>100))	// ����¶����ޱ���ֵ�������쳣������0�����100���������¸�ֵ
		AlarmTL=20;
	if((AlarmTH==0)||(AlarmTH>100))	// ����¶����ޱ���ֵ�������쳣������0�����100���������¸�ֵ
		AlarmTH=35;
	if((AlarmHL==0)||(AlarmHL>100))	// ����¶����ޱ���ֵ�������쳣������0�����100���������¸�ֵ
		AlarmHL=40;
	if((AlarmHH==0)||(AlarmHH>100))	// ����¶����ޱ���ֵ�������쳣������0�����100���������¸�ֵ
		AlarmHH=85;
	if((AlarmPM==0)||(AlarmPM>1300))	// �����ȡ���ı���ֵ�쳣�������¸�ֵ
		AlarmPM=200;
}


/*********************************************************/
// ������
/*********************************************************/
void main(void)
{
	/****pm2.5����****/
	float ad_show_pm = 0;
	unsigned char ad_result_pm = 0, ad_sampling_cnt = 0, pm_str[5] = {0};
	float ad_filiter_pm = 0, ad_pm = 0;
	
	/**/
	unsigned char motor_count = 0;
	
	uchar i;

	ADC0832_init(); // ad�ɼ�pm2.5��ʼ��
	LcdInit();	// Һ�����ܳ�ʼ��
	LcdShowInit();	// Һ����ʾ��ʼ��
	AlarmInit();	// ����ֵ��ʼ��
	SERVO_time1_init();

	while(1)
	{
		/****PM2.5�Ķ�ȡ****/
		ad_result_pm = ADC0832_conv(0x01);
		ad_filiter_pm += ad_result_pm;
		ad_sampling_cnt++;
		if (ad_sampling_cnt > 3) {
			ad_sampling_cnt = 0;
			/****˲ʱ������������ʾ****/
			ad_pm = ad_filiter_pm * 0.25;
			ad_filiter_pm = 0;
			ad_show_pm = ad_pm / 256 * 500;
			pm = (uint)(ad_show_pm);
			LcdGotoXY(0,7);			// Һ����λ����0�е�8��
			LcdPrintNum1(pm);		// ��ʾ�������
		}
		
		/*��ʪ�ȶ�ȡ*/
		EA=0;
		ReadDhtData(); 							// �����ʪ������;
		EA=1;
		LcdGotoXY(1,2);	 						// ��λ��Ҫ��ʾ�¶ȵĵط�
		LcdPrintNum2(temp);					// ��ʾ�¶�ֵ
		LcdGotoXY(1,11);						// ��λ��Ҫ��ʾʪ�ȵĵط�
		LcdPrintNum2(humi);					// ��ʾʪ��ֵ
		
		// �����ж�
		AlarmJudge();						

		/*����ɨ�����ʱ*/
		for(i=0;i<30;i++)
		{
			KeyScanf();			// �����ж�
			DelayMs(10);
		}
	}
}


