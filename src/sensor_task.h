#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t xSensorQueue;

typedef struct
{
    uint32_t   ui32Sequence;    // detect dropped messages
    TickType_t ui32Timestamp;   // tick when sample was taken
    float      fRawLux;         // direct sensor reading
    float      fFilteredLux;    // after moving average
    uint32_t   ui32Missed;      // missed sample counter
} SensorMsg_t;


void vCreateSensorTasks(void);

#endif /* SENSOR_TASK_H */