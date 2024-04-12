#include "ADC0832.h"

void ADC0832_init(void)
{
	ADC0832_CS  = 1;
	ADC0832_CLK = 0;
	ADC0832_DI0 = 1;
}

unsigned char ADC0832_conv(unsigned char channel)
{
	unsigned char i;
	unsigned char ad_result1 = 0, ad_result2 = 0;
	
	ADC0832_CS  = 0;
	ADC0832_CLK = 0;
	ADC0832_DI0 = 1;
	_nop_();
	ADC0832_CLK = 1;
	_nop_();
	ADC0832_CLK = 0;
	
	//choose mode    DIO = 1 means SGL; eithor means DIF
	ADC0832_DI0 = 1;
	_nop_();
	ADC0832_CLK = 1;
	_nop_();
	ADC0832_CLK = 0;
	
	//choose channel
	if (channel == 0x00)
		ADC0832_DI0 = 0;
	else
		ADC0832_DI0 = 1;
	_nop_();
	ADC0832_CLK = 1;
	_nop_();
	ADC0832_CLK = 0;
	
	//receive date
	ADC0832_DI0  = 1;
	for (i = 0; i < 8; i++) {
		ADC0832_CLK = 1;
		_nop_();
		ADC0832_CLK = 0;
		ad_result1 = ad_result1 << 1;
		if (ADC0832_DI0)
			ad_result1 = ad_result1 | 0x01;
	}
	
	for (i = 0; i < 8; i++) {
		ad_result2 = ad_result2 >> 1;
		if (ADC0832_DI0)
			ad_result2 = ad_result2 | 0x80;
		ADC0832_CLK = 1;
		_nop_();
		ADC0832_CLK = 0;
	}
	ADC0832_CLK = 1;
	ADC0832_CS  = 1;
	ADC0832_DI0 = 1;
	
	return ((ad_result1 == ad_result2) ? ad_result1 : 0);
	
}




