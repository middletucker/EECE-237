/*
 * Driver.c
 *
 *  Created on: Nov 28, 2021
 *      Author: kcastlet
 */

#include "Driver.h"

LEDERROR LED0Config_open(LEDCONFIG *ledHandle) {
  // ToDo
  return ERROR;
}
LEDMODE  LED0Config_read(LEDCONFIG *ledHandle) {
  // ToDo
  return ToDo;
}
LEDERROR LED0Config_write(LEDCONFIG *ledHandle,LEDMODE value){
  // ToDo
  return ERROR;
}
LEDERROR LED0Config_close(LEDCONFIG *ledHandle) {
  // ToDo
  return ERROR;
}

LEDERROR LED0_open(LED *ledHandle){
  // ToDo
  return ERROR;
}
LEDSTATE LED0_read(LED *ledHandle){
// ToDo
return LOW;
}
LEDERROR LED0_write(LED *ledHandle,LEDSTATE value){
// ToDo
return ERROR;
}
LEDERROR LED0_close(LED *ledHandle) {
// ToDo
return ERROR;
}

/* Implement this set using the device hardware directly */
LEDERROR LED1Config_open(LEDCONFIG *ledHandle) {
  // ToDo
  return ERROR;
}
LEDMODE  LED1Config_read(LEDCONFIG *ledHandle) {
  // ToDo
  return ToDo;
}
LEDERROR LED1Config_write(LEDCONFIG *ledHandle,LEDMODE value){
  // ToDo
  return ERROR;
}
LEDERROR LED1Config_close(LEDCONFIG *ledHandle) {
  // ToDo
  return ERROR;
}

LEDERROR LED1_open(LED *ledHandle){
  // ToDo
  return ERROR;
}
LEDSTATE LED1_read(LED *ledHandle){
// ToDo
return LOW;
}
LEDERROR LED1_write(LED *ledHandle,LEDSTATE value){
// ToDo
return ERROR;
}
LEDERROR LED1_close(LED *ledHandle) {
// ToDo
return ERROR;
}
