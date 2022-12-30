/*
 * Driver.h
 *
 *  Created on: Nov 28, 2021
 *      Author: kcastlet
 */

#ifndef DRIVER_H_
#define DRIVER_H_

typedef enum _LEDMODE {ToDo} LEDMODE;  // ToDo  can't be empty for test compile
typedef enum _LEDSTATE { HIGH, LOW} LEDSTATE;
typedef enum _LEDERROR {OK, ERROR} LEDERROR;  /* ToDo */
typedef struct _LEDCONFIG{
  /* ToDo */
} LEDCONFIG;

typedef struct _LED{
  /* ToDo */
} LED;

/* Implement this set using the SL library */
LEDERROR LED0Config_open(LEDCONFIG *ledHandle);
LEDMODE  LED0Config_read(LEDCONFIG *ledHandle);
LEDERROR LED0Config_write(LEDCONFIG *ledHandle,LEDMODE value);
LEDERROR LED0Config_close(LEDCONFIG *ledHandle);

LEDERROR LED0_open(LED *ledHandle);
LEDSTATE LED0_read(LED *ledHandle);
LEDERROR LED0_write(LED *ledHandle,LEDSTATE value);
LEDERROR LED0_close(LED *ledHandle);

/* Implement this set using the device hardware directly */
LEDERROR LED1Config_open(LEDCONFIG *ledHandle);
LEDMODE  LED1Config_read(LEDCONFIG *ledHandle);
LEDERROR LED1Config_write(LEDCONFIG *ledHandle,LEDMODE value);
LEDERROR LED1Config_close(LEDCONFIG *ledHandle);

LEDERROR LED1_open(LED *ledHandle);
LEDSTATE LED1_read(LED *ledHandle);
LEDERROR LED1_write(LED *ledHandle,LEDSTATE value);
LEDERROR LED1_close(LED *ledHandle);

#endif /* DRIVER_H_ */
