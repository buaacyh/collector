/*
 * adc.c
 *
 *  Created on: 2017Äê9ÔÂ13ÈÕ
 *      Author: saber
 */
#include "comm.h"
#include "adc.h"

static uint16_t resultsBuffer[8];
static volatile float normalizedADCRes[8];

void ADC14_init(){
    /* Setting reference voltage to 2.5  and enabling reference */
    MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    MAP_REF_A_enableReferenceVoltage();

    /* Initializing ADC (MCLK/1/1) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);

    /* Configuring GPIOs for Analog In */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
                GPIO_PIN5 | GPIO_PIN4 | GPIO_PIN3 | GPIO_PIN2 | GPIO_PIN1
                        | GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
                GPIO_PIN7 | GPIO_PIN6, GPIO_TERTIARY_MODULE_FUNCTION);


    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM7 (A0 - A7)  with no repeat)
     * with internal 2.5v reference */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM7, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM0,
                ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                ADC_INPUT_A0, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM1,
                ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                ADC_INPUT_A1, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM2,
                ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                ADC_INPUT_A2, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM3,
                ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                ADC_INPUT_A3, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM4,
                ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                ADC_INPUT_A4, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM5,
                ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                ADC_INPUT_A5, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM6,
                ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                ADC_INPUT_A6, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM7,
                ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                ADC_INPUT_A7, false);

    /* Enabling the interrupt when a conversion on channel 7 (end of sequence)
     *  is complete and enabling conversions */
    MAP_ADC14_enableInterrupt(ADC_INT7);

    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();

    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();
}
/*
 * get_adc14_value
 * return the value for voltage ref 2.5V, the range of value is 0--2.5
 * */
float get_adc14_value(uint8_t adc_no){
    if(adc_no<8){
        return normalizedADCRes[adc_no];
    }
    else{
        return 0;
    }
}

/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM7. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer */
void ADC14_IRQHandler(void)
{
    uint64_t status;
    int i;

    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    if(status & ADC_INT7)
    {
        MAP_ADC14_getMultiSequenceResult(resultsBuffer);
        for(i = 0; i < 8; i ++)
        {
            normalizedADCRes[i] = (resultsBuffer[i] * 2.5) / 16384;
        }
    }
    i++;
}
