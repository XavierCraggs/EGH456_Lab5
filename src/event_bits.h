#ifndef EVENT_BITS_H
#define EVENT_BITS_H

#include "FreeRTOS.h"
#include "event_groups.h"

/* Shared event group handle */
extern EventGroupHandle_t xSensorEventGroup;

/* Event bit definitions */
#define EVENT_HIGH_THRESHOLD    ( 1UL << 0 )  /* 0x01 - lux above high limit */
#define EVENT_LOW_THRESHOLD     ( 1UL << 1 )  /* 0x02 - lux below low limit  */
#define EVENT_SENSOR_MISSED     ( 1UL << 2 )  /* 0x04 - conversion not ready */
#define EVENT_QUEUE_FULL        ( 1UL << 3 )  /* 0x08 - queue send failed    */
#define EVENT_BTN_TOGGLE_PLOT   ( 1UL << 4 )  /* 0x10 - SW1 pressed          */
#define EVENT_DISPLAY_HOLD      ( 1UL << 5 )  /* 0x20 - SW2 pressed          */

#endif /* EVENT_BITS_H */