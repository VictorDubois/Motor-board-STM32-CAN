/*
	MCP3002 is Arduino Library for communicating with MCP3002 Analog to digital converter.
	Based on the MCP3008 Library created by Uros Petrevski, Nodesign.net 2013
	Released into the public domain.


	ported from Python code originaly written by Adafruit learning system for rPI :
	http://learn.adafruit.com/send-raspberry-pi-data-to-cosm/python-script
*/

#ifndef CurrentReader_H_
#define CurrentReader_H_
#include "stm32g4xx_hal.h"
#define CURRENT_READER_OFFLINE 2048

class CurrentReader
{
  public:
    CurrentReader(){};
    ~CurrentReader(){};

  public:
    virtual int readCurrent(int adcnum);
    void setCurrent(int adcnum, int16_t a_current){};
    virtual constexpr float getOneAmp();
    constexpr float getOneMilliAmp() { return getOneAmp()/1000.f; };
};

class CurrentReaderMCP3002: public CurrentReader
{
  public:
	CurrentReaderMCP3002(){};
    int readCurrent(int adcnum);
    constexpr float getOneAmp() {
    	// MCP3002
    	// 105 <=> 360mA
    	// 69 <=> 240mA
    	// 53 <=> 186mA
    	// 77 <=> 267mA
    	// 89 <=> 301mA
    	// 137 <=> 480mA
    	constexpr float l_one_volt = 774.f; // MCP3002 adc value for one volt
    	return 0.377f*l_one_volt; // Volt to amp conversion of LMD18200
    }
  private:
	int readADC(int a_adc_num);
};

class CurrentReaderAdc: public CurrentReader
{
public:
	CurrentReaderAdc(){};
	CurrentReaderAdc(ADC_HandleTypeDef* a_ADC_Handle);
    int readCurrent(int adcnum);
    constexpr float getOneAmp() {
    	// 20 <=> 200mA
		constexpr float l_one_volt = 265.f; // STM32 adc value for one volt
		return 0.377f*l_one_volt; // Volt to amp conversion of LMD18200
	}
private:
	int readADC(int a_adc_num);
	ADC_HandleTypeDef* m_ADC_Handle;
};

class CurrentReaderCan: public CurrentReader
{
public:
	CurrentReaderCan(){};
    void setCurrent(int adcnum, int16_t a_current)
    {
    	if (adcnum)
    	{
    		m_current_left = a_current;
    	}
    	else
    	{
    		m_current_right = a_current;
    	}
    }
    int readCurrent(int adcnum);
    constexpr float getOneAmp() {
    	//4000 = 0.80A sur deux moteurs
    	//2000 = 0.25A sur deux moteurs,
    	//1050 = 0.133A sur deux moteurs
    	//300  = 0.09A sur deux moteurs (arrêt)
    	return 1700.f; // 1700 arbitrary C620 value that lead to reasonable threshold for 10A. @TODO calibrate
    };
private:
    int16_t m_current_left = 0;
    int16_t m_current_right = 0;
};


#endif /* CurrentReader_H_ */
