/***************************************************************************//**
 * @file
 * @brief i2cspm bare metal examples functions
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
//i2c includes
#include <string.h>
#include <stdio.h>
#include "i2cspm.h"
#include "em_emu.h"
#include "sl_i2cspm.h"
#include "sl_i2cspm_instances.h"
#include "sl_sleeptimer.h"
#include "sl_simple_led.h"
#include "sl_simple_led_instances.h"
#include "sl_atomic.h"

//lcd includes
#include "sl_board_control.h"
//#include "sl_simple_button_instances.h"
#include "em_assert.h"
#include "glib.h"
#include "dmd.h"

//wdog include
#include "em_wdog.h"

WDOG_Init_TypeDef wdog=WDOG_INIT_DEFAULT;

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_cryotimer.h"
//#include "bsp.h"

// Note: change this to one of the defined periods in em_cryotimer.h
// Wakeup events occur every 2048 prescaled clock cycles
#define CRYOTIMER_PERIOD    cryotimerPeriod_2k

// Note: change this to one of the defined prescalers in em_cryotimer.h
// The clock is divided by one
#define CRYOTIMER_PRESCALE  cryotimerPresc_1
/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

// The limits are set relative to the initial temperature
#define TEMPERATURE_BAND_C               4

// SI7021_Config_Settings Si7021 Configuration Settings
#define SI7021_I2C_DEVICE                (sl_i2cspm_sensor) /**< I2C device used to control the Si7021  */
#define SI7021_I2C_BUS_ADDRESS           0x40               /**< I2C bus address                        */
#define SI7021_DEVICE_ID                 0x15               /**< Si7021 device ID                       */

// Si7021 command macro definitions
#define SI7021_CMD_MEASURE_RH            0xE5               /**< Measure Relative Humidity, Hold Master Mode */
#define SI7021_CMD_MEASURE_RH_NO_HOLD    0xF5               /**< Measure Relative Humidity, No Hold Master Mode */
#define SI7021_CMD_MEASURE_TEMP          0xE3               /**< Measure Temperature, Hold Master Mode */
#define SI7021_CMD_MEASURE_TEMP_NO_HOLD  0xF3               /**< Measure Temperature, No Hold Master Mode */
#define SI7021_CMD_READ_TEMP             0xE0               /**< Read Temperature Value from Previous RH Measurement */
#define SI7021_CMD_RESET                 0xFE               /**< Reset */
#define SI7021_CMD_WRITE_USER_REG1       0xE6               /**< Write RH/T User Register 1 */
#define SI7021_CMD_READ_USER_REG1        0xE7               /**< Read RH/T User Register 1 */
#define SI7021_CMD_WRITE_HEATER_CTRL     0x51               /**< Write Heater Control Register */
#define SI7021_CMD_READ_HEATER_CTRL      0x11               /**< Read Heater Control Register */
#define SI7021_CMD_READ_ID_BYTE1         { 0xFA, 0x0F }       /**< Read Electronic ID 1st Byte */
#define SI7021_CMD_READ_ID_BYTE2         { 0xFC, 0xC9 }       /**< Read Electronic ID 2nd Byte */
#define SI7021_CMD_READ_FW_REV           { 0x84, 0xB8 }       /**< Read Firmware Revision */

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/

static uint32_t relative_humidity;
static int32_t temperature;
static int32_t start_temperature;
static int32_t high_limit, low_limit;
static bool read_sensor_data = false;
static sl_sleeptimer_timer_handle_t delay_timer;

static GLIB_Context_t glibContext;
static int currentLine = 0;

static int xcoor_temp = 7;
static int xcoor_humid = 7;

/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/

/***************************************************************************//**
 * Function to perform I2C transactions on the Si7021
 *
 * This function is used to perform I2C transactions on the Si7021
 * including read, write and continued write read operations.
 ******************************************************************************/


static I2C_TransferReturn_TypeDef SI7021_transaction(uint16_t flag,
                                                     uint8_t *writeCmd,
                                                     size_t writeLen,
                                                     uint8_t *readCmd,
                                                     size_t readLen)
{
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;

  seq.addr = SI7021_I2C_BUS_ADDRESS << 1;
  seq.flags = flag;

  switch (flag) {
    // Send the write command from writeCmd
    case I2C_FLAG_WRITE:
      seq.buf[0].data = writeCmd;
      seq.buf[0].len  = writeLen;

      break;

    // Receive data into readCmd of readLen
    case I2C_FLAG_READ:
      seq.buf[0].data = readCmd;
      seq.buf[0].len  = readLen;

      break;

    // Send the write command from writeCmd
    // and receive data into readCmd of readLen
    case I2C_FLAG_WRITE_READ:
      seq.buf[0].data = writeCmd;
      seq.buf[0].len  = writeLen;

      seq.buf[1].data = readCmd;
      seq.buf[1].len  = readLen;

      break;

    default:
      return i2cTransferUsageFault;
  }

  // Perform the transfer and return status from the transfer
  ret = I2CSPM_Transfer(SI7021_I2C_DEVICE, &seq);

  return ret;
}

/***************************************************************************//**
 * Function to decode Relative Humidity from the read value
 *
 * This function decodes RH data using the formula provided in the Si7021
 * datasheet. Returns the value in Percent.
 ******************************************************************************/
uint32_t decode_rh(uint8_t* read_register)
{
  uint32_t rhValue;

  // Formula to decode read RH from the Si7021 Datasheet
  rhValue = ((uint32_t) read_register[0] << 8) + (read_register[1] & 0xfc);
  rhValue = (((rhValue) * 125) >> 16) - 6;

  return rhValue;
}

/***************************************************************************//**
 * Function to decode Temperature from the read Temperature value
 *
 * This function decodes Temperature using the formula provided in the Si7021
 * datasheet. Returns the value in Celsius.
 ******************************************************************************/
uint32_t decode_temp(uint8_t* read_register)
{
  uint32_t tempValue;
  float actual_temp;
  uint32_t rounded_temp;

  // Formula to decode read Temperature from the Si7021 Datasheet
  tempValue = ((uint32_t) read_register[0] << 8) + (read_register[1] & 0xfc);
  actual_temp = (((tempValue) * 175.72) / 65536) - 46.85;

  // Round the temperature to an integer value
  rounded_temp = (uint32_t)(actual_temp + 0.5 - (actual_temp < 0));

  return rounded_temp;
}

/***************************************************************************//**
 * Function to measure humidity and temperature from the Si7021
 *
 * This function measures current humidity and temperature using the Si7021,
 * returning values using the pointer inputs.
 ******************************************************************************/
static void SI7021_measure(uint32_t *rhData, int32_t *tData)
{
  I2C_TransferReturn_TypeDef ret;
  uint8_t cmd;
  uint8_t readData[2];
  uint32_t timeout;

  // Start no-hold measurement by writing command to si7021
  cmd = SI7021_CMD_MEASURE_RH_NO_HOLD;
  ret = SI7021_transaction(I2C_FLAG_WRITE, &cmd, 2, NULL, 0);
  EFM_ASSERT(ret == i2cTransferDone);

  // Wait for data to become ready
  timeout = 300;
  while (timeout--) {
    ret = SI7021_transaction(I2C_FLAG_READ, NULL, 0, readData, 2);
    if (ret == i2cTransferDone) {
      break;
    } else if (ret == i2cTransferNack) {
      // Si7021 returns a Nack if data not ready
      continue;
    }
  }

  EFM_ASSERT(timeout > 0);

  // Data is ready, decode the RH value
  *rhData = decode_rh(readData);

  // Read the temperature measured during RH measurement
  cmd = SI7021_CMD_READ_TEMP;
  ret = SI7021_transaction(I2C_FLAG_WRITE_READ, &cmd, 1, readData, 2);
  EFM_ASSERT(ret == i2cTransferDone);

  *tData = decode_temp(readData);
}

/***************************************************************************//**
 * Callback function for the periodic timer.
 *
 * This function is called when the periodic timer goes off. It sets the
 * read_sensor_data flag.
 ******************************************************************************/
static void timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;
  bool local_read_sensor_data = true;
  sl_atomic_store(read_sensor_data, local_read_sensor_data);
}

/***************************************************************************//**
 * Helper function to Initialize the periodic timer
 *
 * This function starts a periodic timer which sets a flag, regulating the
 * sampling rate of the temperature sensor.
 ******************************************************************************/
static void initialise_timer()
{
  uint32_t ticks = sl_sleeptimer_ms_to_tick(1000);
  sl_sleeptimer_start_periodic_timer(&delay_timer, ticks, timer_callback, NULL, 0, 0);
}

/***************************************************************************//**
 * Helper function to Initialize the upper and lower limits
 *
 * This function sets the limits based on the temperature when the application
 * is started.
 ******************************************************************************/
void initialise_temp_limits()
{
  // Get current temperature
  SI7021_measure(&relative_humidity, &start_temperature);

  // Set temperature limits based on initial temperature.
  low_limit = start_temperature - (TEMPERATURE_BAND_C / 2);
  high_limit = start_temperature + (TEMPERATURE_BAND_C / 2);
}

/***************************************************************************//**
 * Helper function to turn on led 0 or 1 based on current temperature
 *
 * This function turns on led 0 or 1 based on the current temperature and
 * lower and upper limits.
 ******************************************************************************/
void set_leds(int32_t temp)
{
  if (temp > high_limit) {
    // For higher temperature, turn led0 on and turn led1 off
    sl_led_turn_on(&sl_led_led0);
    sl_led_turn_off(&sl_led_led1);
    printf("Turning LED0 on!\r\n");
  } else if (temp < low_limit) {
    // For higher temperature, turn led1 on and turn led0 off
    sl_led_turn_off(&sl_led_led0);
    sl_led_turn_on(&sl_led_led1);
    printf("Turning LED1 on!\r\n");

  }
  else if (temp >= low_limit & temp <= high_limit)
    {
      sl_led_turn_off(&sl_led_led0);
      sl_led_turn_off(&sl_led_led1);
    }

}

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/*******************************************************************************
 * Initialize example.
 ******************************************************************************/
void i2cspm_app_init(void)
{
  I2C_TransferReturn_TypeDef ret;
  uint8_t cmdReadId2[2] = SI7021_CMD_READ_ID_BYTE2;
  uint8_t deviceId[8];

  // Wait for sensor to become ready
  sl_sleeptimer_delay_millisecond(80);

  // Check for device presence  and compare device ID
  ret = SI7021_transaction(I2C_FLAG_WRITE_READ, cmdReadId2, 2, deviceId, 8);

  // Make sure transfer was successful
  EFM_ASSERT(ret == i2cTransferDone);

  // Check the Received Device ID
  EFM_ASSERT(deviceId[0] == SI7021_DEVICE_ID);

  // Initialize LED PWM module
  initialise_temp_limits();

  // Initialize Periodic timer
  initialise_timer();

  // stdout is redirected to VCOM in project configuration
  printf("Welcome to the I2CSPM example application\r\n");
}
void initCryotimer(void)
{
  // Enable cryotimer clock
  CMU_ClockEnable(cmuClock_CRYOTIMER, true);

  // Initialize cryotimer
  CRYOTIMER_Init_TypeDef init = CRYOTIMER_INIT_DEFAULT;
  init.osc = cryotimerOscULFRCO;   // Use the ULFRCO
  init.presc = CRYOTIMER_PRESCALE; // Set the prescaler
  init.period = CRYOTIMER_PERIOD;  // Set when wakeup events occur
  init.enable = true;              // Start the cryotimer after initialization is done
  CRYOTIMER_Init(&init);

  // Enable cryotimer interrupts
  CRYOTIMER_IntEnable(CRYOTIMER_IEN_PERIOD);
  NVIC_EnableIRQ(CRYOTIMER_IRQn);
}

/***************************************************************************//**
 * Ticking function
 ******************************************************************************/
void i2cspm_app_process_action(void)
{

  //EMU_EnterEM1();

  int temp[50];
  int humid[50];


  bool local_read_sensor_data=1;

  //sl_atomic_load(local_read_sensor_data, read_sensor_data);

  if (local_read_sensor_data) {
    // Measure the current humidity and temperature
    SI7021_measure(&relative_humidity, &temperature);

    // Print the current humidity and temperature to vcom
    printf("\r\n");
    printf("Relative Humidity = %ld%%\r\n", relative_humidity);
    printf("Temperature = %ld C\r\n", temperature);

    //GLIB_drawPixel(GLIB_Context_t *pContext, int32_t x, int32_t y);
    // display temp section
    if(xcoor_temp < 128)
      {
       GLIB_drawPixel(&glibContext, xcoor_temp, temperature + 74);
        xcoor_temp++;
      }
    temp[temperature] = temperature;
    sprintf(temp, "%ld C\r\n", temperature);
    GLIB_drawStringOnLine(&glibContext,
                            temp,
                            currentLine = 7,
                            GLIB_ALIGN_RIGHT,
                            1,
                            1,
                            true);



    //display humidity section
    if(xcoor_humid < 128)
          {
            GLIB_drawPixel(&glibContext, xcoor_humid, relative_humidity);
            xcoor_humid++;
          }
    humid[relative_humidity] = relative_humidity;
        sprintf(humid, "%ld%%\r\n", relative_humidity);
        GLIB_drawStringOnLine(&glibContext,
                                humid,
                                currentLine = 0,
                                GLIB_ALIGN_RIGHT,
                                1,
                                1,
                                true);

    DMD_updateDisplay();
    // Set appropriate LEDs (led0 or 1) based on temperature
    set_leds(temperature);

    // Reset the flag
    local_read_sensor_data = false;
    sl_atomic_store(read_sensor_data, local_read_sensor_data);
  }
}

//lcd
void memlcd_app_process_action(void)
{
  return;
}

/******************************************************************************
 * Global LCD
 * ***************************************************************************/
//Global***********************************************************************
void memlcd_app_init(void)
{
  uint32_t status;

  /* Enable the memory lcd */
  status = sl_board_enable_display();
  EFM_ASSERT(status == SL_STATUS_OK);

  /* Initialize the DMD support for memory lcd display */
  status = DMD_init(0);
  EFM_ASSERT(status == DMD_OK);

  /* Initialize the glib context */
  status = GLIB_contextInit(&glibContext);
  EFM_ASSERT(status == GLIB_OK);

  glibContext.backgroundColor = White;
  glibContext.foregroundColor = Black;

  /* Fill lcd with background color */
  GLIB_clear(&glibContext);

  /* Use Narrow font */
  GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNarrow6x8);

  //create axis for lcd
    GLIB_drawStringOnLine(&glibContext,
                                "________________",
                                currentLine = 11,
                                GLIB_ALIGN_LEFT,
                                5,
                                9,
                                true);
    GLIB_drawStringOnLine(&glibContext,
                                   "________________",
                                   currentLine = 5,
                                   GLIB_ALIGN_LEFT,
                                   5,
                                   9,
                                   true);
    currentLine = 0;
    for (int i; i < 20; i++)
      {
        GLIB_drawStringOnLine(&glibContext,
                                           "|",
                                           currentLine,
                                           GLIB_ALIGN_LEFT,
                                           5,
                                           0,
                                           true);
        currentLine++;
      }

  /* Draw text on the memory lcd display*/
  GLIB_drawStringOnLine(&glibContext,
                        "Temp[F]",
                        currentLine = 7,
                        GLIB_ALIGN_LEFT,
                        3,
                        3,
                        true);
  GLIB_drawStringOnLine(&glibContext,
                        "Humidity[%]",
                        currentLine = 0,
                        GLIB_ALIGN_LEFT,
                        1,
                        1,
                        true);

  GLIB_drawStringOnLine(&glibContext,
                          "Time[s]",
                          currentLine = 6,
                          GLIB_ALIGN_RIGHT,
                          1,
                          1,
                          true);
  GLIB_drawStringOnLine(&glibContext,
                            "Time[s]",
                            currentLine = 12,
                            GLIB_ALIGN_RIGHT,
                            1,
                            1,
                            true);


  DMD_updateDisplay();
}

void CRYOTIMER_IRQHandler(void)
{
  // Acknowledge the interrupt
  uint32_t flags = CRYOTIMER_IntGet();
  CRYOTIMER_IntClear(flags);
  // Put a barrier here to ensure interrupts are not retriggered. See note above
  __DSB();
  //EMU_EnterEM1();


  //WDOG_Feed();
}

