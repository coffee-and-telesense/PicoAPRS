/*
 * device.h
 *
 *  Created on: Aug 2, 2024
 *      Author: arie
 */



#include "constants.h"

#ifndef DEVICE_CONFIGURATION
#define DEVICE_CONFIGURATION

// CPU settings
#ifndef TARGET_CPU
    #define TARGET_CPU m328p
#endif

#ifndef F_CPU
    #define F_CPU 16000000
#endif

#ifndef FREQUENCY_CORRECTION
    #define FREQUENCY_CORRECTION 0
#endif
a
// Sampling & timer setup
#define CONFIG_AFSK_DAC_SAMPLERATE 9600

#define PLACEHOLDER_DAC_PORT
#define PLACEHOLDER_DAC_DDR
#define PLACEHOLDER_LED_PORT
#define PLACEHOLDER_LED_DDR
#define PLACEHOLDER_ADC_PORT
#define PLACEHOLDER_ADC_DDR

// Port settings
#if TARGET_CPU == m328p
    #define DAC_PORT PLACEHOLDER_DAC_PORT
    #define DAC_DDR  PLACEHOLDER_DAC_DDR
    #define LED_PORT PLACEHOLDER_LED_PORT
    #define LED_DDR  PLACEHOLDER_LED_DDR
    #define ADC_PORT PLACEHOLDER_ADC_PORT
    #define ADC_DDR  PLACEHOLDER_ADC_DDR
#endif

#endif

