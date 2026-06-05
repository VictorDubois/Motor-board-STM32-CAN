/*
	MCP3002 is Arduino Library for communicating with MCP3002 Analog to digital converter.
	Based on the MCP3008 Library created by Uros Petrevski, Nodesign.net 2013
	Released into the public domain.


	ported from Python code originaly written by Adafruit learning system for rPI :
	http://learn.adafruit.com/send-raspberry-pi-data-to-cosm/python-script
*/

// Mesures :
// MCP3002
// 105 <=> 360mA
// 69 <=> 240mA
// 53 <=> 186mA
// 77 <=> 267mA
// 89 <=> 301mA
// 137 <=> 480mA

// STM32 ADC
// 20 <=> 200mA
// "ONE_VOLT" factor = measure/("ONE_AMP" factor * real current)
#define USE_C620_CURRENT

#ifdef USE_MCP3002
	#define ONE_VOLT        774.f // MCP3002 adc value for one volt
#else
	#define ONE_VOLT        265.f // STM32 adc value for one volt
#endif

#ifdef USE_C620_CURRENT
//4000 = 0.80A sur deux moteurs
//2000 = 0.25A sur deux moteurs,
//1050 = 0.133A sur deux moteurs
//300  = 0.09A sur deux moteurs (arrêt)
	#define ONE_AMP         1700.f // 1700 arbitrary C620 value that lead to reasonable threshold for 10A. @TODO calibrate
#else
	#define ONE_AMP         (0.377f*ONE_VOLT) // Volt to amp conversion of LMD18200
#endif

#define ONE_MILLIAMP (ONE_AMP / 1000.f)

#ifndef CurrentReader_H_
#define CurrentReader_H_
#include "stm32g4xx_hal.h"
#define CURRENT_READER_OFFLINE 2048

class CurrentReader
{
  public:
    CurrentReader();
#ifndef USE_MCP3002
    CurrentReader(ADC_HandleTypeDef* a_hadc2);
  private:
    ADC_HandleTypeDef* m_hadc2;
#endif

#ifdef USE_C620_CURRENT
    int16_t m_current_left = 0;
    int16_t m_current_right = 0;
#endif
  public:
    int readADC(int adcnum);
    int readCurrent(int adcnum);
#ifdef USE_C620_CURRENT
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
;
#endif
  private:
    GPIO_TypeDef* m_miso_gpio_bank;
	uint16_t m_miso_gpio;
	GPIO_TypeDef* m_mosi_gpio_bank;
	uint16_t m_mosi_gpio;
	GPIO_TypeDef* m_clk_bank;
	uint16_t m_clk_gpio;
	GPIO_TypeDef* m_cs_gpio_bank;
	uint16_t m_cs_gpio;
};

#endif /* CurrentReader_H_ */
