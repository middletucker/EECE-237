/***************************************************************************//**
 * @file
 * @brief Blink PWM examples functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
//pmw includes
#include "sl_pwm.h"
#include "sl_pwm_instances.h"
#include "sl_sleeptimer.h"

//adc includes
#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"


//lcd includes
#include "glib.h"

static GLIB_Context_t glibContext;
static int currentLine = 0;

/******************************************************************************
 * ADC Code
 *****************************************************************************/
#define adcFreq   16000000

static volatile uint32_t sample;
static volatile uint32_t millivolts;

/**************************************************************************//**
 * @brief  Initialize ADC function
 *****************************************************************************/
void initADC (void)
{
  // Enable ADC0 clock
  CMU_ClockEnable(cmuClock_ADC0, true);

  // Declare init structs
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  // Modify init structs and initialize
  init.prescale = ADC_PrescaleCalc(adcFreq, 0); // Init to max ADC clock for Series 1

  initSingle.diff       = false;        // single ended
  initSingle.reference  = adcRef2V5;    // internal 2.5V reference
  initSingle.resolution = adcRes12Bit;  // 12-bit resolution
  initSingle.acqTime    = adcAcqTime4;  // set acquisition time to meet minimum requirement

  // Select ADC input. See README for corresponding EXP header pin.
  initSingle.posSel = adcPosSelAPORT2XCH9;
  init.timebase = ADC_TimebaseCalc(0);

  ADC_Init(ADC0, &init);
  ADC_InitSingle(ADC0, &initSingle);
}

/**************************************************************************//**
 * @brief  ADC Main function
 *****************************************************************************/
float adc_measure()
{
  CHIP_Init();

  initADC();

  // Infinite loop
  while(1)
  {
    // Start ADC conversion
    ADC_Start(ADC0, adcStartSingle);

    // Wait for conversion to be complete
    while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));

    // Get ADC result
    sample = ADC_DataSingleGet(ADC0);

    // Calculate input voltage in mV
    return millivolts = (sample * 2500) / 4096;
  }
}


/****************************************************************************
 * PWM Code
 ****************************************************************************/
uint8_t pwm_lut[] = {
  0, 1, 1, 1, 2, 2, 2, 2, 2, 2,
  2, 3, 3, 3, 3, 3, 4, 4, 4, 4,
  5, 5, 5, 5, 6, 6, 6, 7, 7, 7,
  8, 8, 8, 9, 9, 10, 10, 10, 11, 11,
  12, 12, 13, 13, 14, 15, 15, 16, 17, 17,
  18, 19, 19, 20, 21, 22, 23, 23, 24, 25,
  26, 27, 28, 29, 30, 31, 32, 34, 35, 36,
  37, 39, 40, 41, 43, 44, 46, 48, 49, 51,
  53, 54, 56, 58, 60, 62, 64, 66, 68, 71,
  73, 75, 78, 80, 83, 85, 88, 91, 94, 97,
  100,
};


void blink_pwm_init(void)
{
  // Enable PWM output
  sl_pwm_start(&sl_pwm_led0);
  sl_pwm_start(&sl_pwm_led1);
}

void blink_pwm_process_action(void)
{
  //int volts[50];

  int volts_scale=millivolts*10;

  //fade leds based on voltage value
  sl_pwm_set_duty_cycle(&sl_pwm_led0, volts_scale);
  sl_pwm_set_duty_cycle(&sl_pwm_led1, volts_scale);

  /*volts[millivolts] = millivolts;
      sprintf(volts, "%ld mV\r\n", millivolts);
      GLIB_drawStringOnLine(&glibContext,
                              volts,
                              currentLine = 7,
                              GLIB_ALIGN_LEFT,
                              10,
                              10,
                              true);
                              */


  /*for (uint8_t i = 0; i < 100; i++) {
    sl_pwm_set_duty_cycle(&sl_pwm_led0, pwm_lut[i]);
    sl_sleeptimer_delay_millisecond(6);
    if (i == 0) {
      sl_sleeptimer_delay_millisecond(190);
    }
  }
  for (uint8_t i = 100; i > 0; i--) {
    sl_pwm_set_duty_cycle(&sl_pwm_led0, pwm_lut[i]);
    sl_sleeptimer_delay_millisecond(6);
    if (i == 100) {
      sl_sleeptimer_delay_millisecond(190);
    }
  }*/
}
