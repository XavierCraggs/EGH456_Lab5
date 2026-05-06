#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "event_bits.h"

#include "sensor_task.h"
#include "drivers/opt3001.h"
#include "drivers/i2cOptDriver.h"
#include "utils/uartstdio.h"
#include "drivers/rtos_hw_drivers.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

/*******************************************************

* Sensor task implementation.

*******************************************************/

/* Config*/
#define SENSOR_QUEUE_LENGTH (5U)
#define FILTER_WINDOW_SIZE (8U)

#define LUX_HIGH_THRESHOLD      500.0f
#define LUX_LOW_THRESHOLD       10.0f

/* Queue handle */
QueueHandle_t xSensorQueue = NULL;

/* Task handle */
TaskHandle_t xSensorTaskHandle = NULL;

/*Counters*/
static uint32_t ui32SequenceNum = 0U;
static uint32_t ui32MissedSamples = 0U;

/*******************************************************/

 /* Moving average filter state */

static float fLuxHistory[FILTER_WINDOW_SIZE] = {0};
static uint32_t ui32FilterIndex = 0;
static uint32_t ui32FilterCount = 0;

static float prvMovingAverage(float fNewValue)
{
    float fSum = 0.0f;
    uint32_t i;

    fLuxHistory[ui32FilterIndex] = fNewValue;
    ui32FilterIndex = (ui32FilterIndex + 1) % FILTER_WINDOW_SIZE;

    if (ui32FilterCount < FILTER_WINDOW_SIZE)
    {
        ui32FilterCount++;
    }

    for (i = 0; i < ui32FilterCount; i++)
    {
        fSum += fLuxHistory[i];
    }

    return fSum / (float)ui32FilterCount;
}

/*-----------------------------------------------------------*/
/* Timer callback — runs in timer daemon context
 * wake the sensor task */

static void vSampleTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    vTaskNotifyGiveFromISR(xSensorTaskHandle, NULL);
}

/*-----------------------------------------------------------*/
/* Sensor sampling task */

static void vSensorTask(void *pvParameters)
{
    uint16_t ui16RawData;
    float fRawLux;
    float fFilteredLux;
    SensorMsg_t xMsg;
    BaseType_t xSendResult;

    (void)pvParameters;

    /* Initialise OPT3001 */
    UARTprintf("Initialising OPT3001...\n");
    sensorOpt3001Init();

    while (!sensorOpt3001Test())
    {
        UARTprintf("OPT3001 test failed, retrying...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
        sensorOpt3001Init();
    }
    UARTprintf("OPT3001 ready.\n");

    for (;;)
    {
        /* Block here until timer fires */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Attempt one read */
        if (sensorOpt3001Read(&ui16RawData))
        {

            sensorOpt3001Convert(ui16RawData, &fRawLux);
            fFilteredLux = prvMovingAverage(fRawLux);

            /* Build message */
            xMsg.ui32Sequence  = ui32SequenceNum++;
            xMsg.ui32Timestamp = xTaskGetTickCount();
            xMsg.fRawLux       = fRawLux;
            xMsg.fFilteredLux  = fFilteredLux;
            xMsg.ui32Missed    = ui32MissedSamples;

           
            /* Set threshold event bits based on filtered value */
            if (fFilteredLux > LUX_HIGH_THRESHOLD)
            {
                xEventGroupSetBits(xSensorEventGroup, EVENT_HIGH_THRESHOLD);
            }
            else if (fFilteredLux < LUX_LOW_THRESHOLD)
            {
                xEventGroupSetBits(xSensorEventGroup, EVENT_LOW_THRESHOLD);
            }
            else
            {
                /* Clear threshold bits when back in normal range */
                xEventGroupClearBits(xSensorEventGroup,
                                    EVENT_HIGH_THRESHOLD | EVENT_LOW_THRESHOLD);
            }

            /* Send to queue */
            xSendResult = xQueueSend(xSensorQueue, &xMsg, 0);

            if (xSendResult != pdPASS)
            {
                xEventGroupSetBits(xSensorEventGroup, EVENT_QUEUE_FULL);
                UARTprintf("[SENSOR] Queue full! seq=%u dropped\n",
                        (unsigned int)xMsg.ui32Sequence);
            }
        }
        else
        {
            /* Conversion not ready */
            ui32MissedSamples++;
            xEventGroupSetBits(xSensorEventGroup, EVENT_SENSOR_MISSED);
            UARTprintf("[SENSOR] Missed sample #%u\n",
                       (unsigned int)ui32MissedSamples);
        }
    }
}

/*-----------------------------------------------------------*/
/* Display task */

static void vDisplayTask(void *pvParameters)
{
    SensorMsg_t xMsg;
    (void)pvParameters;

    for (;;)
    {
        /* Block until a message arrives */
        if (xQueueReceive(xSensorQueue, &xMsg, portMAX_DELAY) == pdPASS)
        {
            
            UARTprintf("%d.%02d,%d.%02d\n",
                (int)xMsg.fRawLux,
                (int)(xMsg.fRawLux * 100.0f) % 100,
                (int)xMsg.fFilteredLux,
                (int)(xMsg.fFilteredLux * 100.0f) % 100);


             vTaskDelay(pdMS_TO_TICKS(350));                          
        }
    }
}

/*-----------------------------------------------------------*/
/* Create everything */

void vCreateSensorTasks(void)
{
    TimerHandle_t xSampleTimer;
    
    i2cOptDriverInit();

    /* Create the queue */
    xSensorQueue = xQueueCreate(SENSOR_QUEUE_LENGTH, sizeof(SensorMsg_t));

    if (xSensorQueue == NULL)
    {
        UARTprintf("Sensor queue creation failed!\n");
        return;
    }

    /* Create event group */
    xSensorEventGroup = xEventGroupCreate();
    if (xSensorEventGroup == NULL)
    {
        UARTprintf("Event group creation failed!\n");
        return;
    }

    /* Create sensor task first so we have the handle for the timer */
    xTaskCreate(vSensorTask,
                "SensorTask",
                512,
                NULL,
                tskIDLE_PRIORITY + 2,
                &xSensorTaskHandle);   /* store handle for timer callback */

    /* Create display task at lower priority */
    xTaskCreate(vDisplayTask,
                "DisplayTask",
                512,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    /* Create 5Hz repeating software timer */
    xSampleTimer = xTimerCreate(
        "SampleTimer",
        pdMS_TO_TICKS(200),   /* 200ms = 5Hz */
        pdTRUE,               /* auto-reload */
        NULL,
        vSampleTimerCallback
    );

    if (xSampleTimer == NULL)
    {
        UARTprintf("Timer creation failed!\n");
        return;
    }

    xTimerStart(xSampleTimer, 0);
}