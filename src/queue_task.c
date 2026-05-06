/*
 * queue_task.c
 *
 * Clean FreeRTOS queue example for EGH456 Workshop 5.
 *
 * This example creates two queues:
 *   1. xQueue stores complete struct MsgObj messages.
 *   2. xPointerQueue stores pointers to struct MsgObj messages.
 *
 * Students should inspect the queue creation, send, and receive calls to
 * understand what each queue stores and how the tasks communicate.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "utils/uartstdio.h"

/*-----------------------------------------------------------*/
/* Task and queue settings for Task A. */

#define mainQUEUE_LENGTH                    ( 4U )
#define mainQUEUE_SEND_TICKS_TO_WAIT        ( ( TickType_t ) 0U )
#define mainQUEUE_RECEIVE_TICKS_TO_WAIT     ( ( TickType_t ) 10U )

#define mainRECEIVER_DELAY_MS               ( 0U )

#define mainQUEUE_SEND_TASK_PRIORITY        ( tskIDLE_PRIORITY + 1U )
#define mainQUEUE_RECEIVE_TASK_PRIORITY     ( tskIDLE_PRIORITY + 2U )

/*-----------------------------------------------------------*/
/* Message object used by the queue examples. */

struct MsgObj
{
    uint8_t id;
    uint8_t val;
    uint32_t seq;
    TickType_t tick;
};

struct MsgWriterConfig
{
    uint8_t id;
    uint8_t step;
    uint32_t delayMs;
};

/*-----------------------------------------------------------*/
/* Queue handles. */

static QueueHandle_t xQueue = NULL;
static QueueHandle_t xPointerQueue = NULL;

/* Four writer task configurations. */
static const struct MsgWriterConfig xWriterConfig[] =
{
    { 0xA0U, 1U, 20U },
    { 0xA1U, 2U, 30U },
    { 0xA2U, 3U, 50U },
    { 0xA3U, 4U, 80U }
};

/*-----------------------------------------------------------*/
/* Task prototypes. */

static void writerTask( void *pvParameters );
static void readerTask( void *pvParameters );
static void pointerReaderTask( void *pvParameters );

/*-----------------------------------------------------------*/

void vcreateQueueTasks( void )
{
    uint32_t i;

    xQueue = xQueueCreate( mainQUEUE_LENGTH,
                           sizeof( struct MsgObj ) );

    xPointerQueue = xQueueCreate( mainQUEUE_LENGTH,
                                  sizeof( struct MsgObj * ) );

    if( ( xQueue == NULL ) || ( xPointerQueue == NULL ) )
    {
        UARTprintf( "Queue creation failed\r\n" );
        return;
    }

    xTaskCreate( readerTask,
                 "readerTask",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 mainQUEUE_RECEIVE_TASK_PRIORITY,
                 NULL );

    xTaskCreate( pointerReaderTask,
                 "ptrReaderTask",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 mainQUEUE_RECEIVE_TASK_PRIORITY,
                 NULL );

    for( i = 0U; i < ( sizeof( xWriterConfig ) / sizeof( xWriterConfig[ 0 ] ) ); i++ )
    {
        xTaskCreate( writerTask,
                     "writerTask",
                     configMINIMAL_STACK_SIZE,
                     ( void * ) &( xWriterConfig[ i ] ),
                     mainQUEUE_SEND_TASK_PRIORITY,
                     NULL );
    }
}

/*-----------------------------------------------------------*/

static void writerTask( void *pvParameters )
{
    const struct MsgWriterConfig *pxConfig;
    struct MsgObj xMsgObj;
    struct MsgObj *ptrMsgObj;
    BaseType_t xSendStatus;

    pxConfig = ( const struct MsgWriterConfig * ) pvParameters;

    xMsgObj.id = pxConfig->id;
    xMsgObj.val = 0U;
    xMsgObj.seq = 0U;
    xMsgObj.tick = 0U;

    for( ;; )
    {
        xMsgObj.seq++;
        xMsgObj.val = ( uint8_t ) ( xMsgObj.val + pxConfig->step );
        xMsgObj.tick = xTaskGetTickCount();

        xSendStatus = xQueueSend( xQueue,
                                  ( void * ) &xMsgObj,
                                  mainQUEUE_SEND_TICKS_TO_WAIT );

        ( void ) xSendStatus;

        ptrMsgObj = &xMsgObj;

        xQueueSend( xPointerQueue,
                    ( void * ) &ptrMsgObj,
                    mainQUEUE_SEND_TICKS_TO_WAIT );

        vTaskDelay( pdMS_TO_TICKS( pxConfig->delayMs ) );
    }
}

/*-----------------------------------------------------------*/

static void readerTask( void *pvParameters )
{
    struct MsgObj xReadMsgObj;

    ( void ) pvParameters;

    for( ;; )
    {
        if( xQueueReceive( xQueue,
                           ( void * ) &xReadMsgObj,
                           mainQUEUE_RECEIVE_TICKS_TO_WAIT ) == pdPASS )
        {
            UARTprintf( "RX xQueue id=0x%02x val=%u seq=%u tick=%u\r\n",
                        ( unsigned int ) xReadMsgObj.id,
                        ( unsigned int ) xReadMsgObj.val,
                        ( unsigned int ) xReadMsgObj.seq,
                        ( unsigned int ) xReadMsgObj.tick );
        }

        if( mainRECEIVER_DELAY_MS > 0U )
        {
            vTaskDelay( pdMS_TO_TICKS( mainRECEIVER_DELAY_MS ) );
        }
    }
}

/*-----------------------------------------------------------*/

static void pointerReaderTask( void *pvParameters )
{
    struct MsgObj *ptrReadMsgObj;

    ( void ) pvParameters;

    for( ;; )
    {
        ptrReadMsgObj = NULL;

        if( xQueueReceive( xPointerQueue,
                           ( void * ) &ptrReadMsgObj,
                           mainQUEUE_RECEIVE_TICKS_TO_WAIT ) == pdPASS )
        {
            if( ptrReadMsgObj != NULL )
            {
                UARTprintf( "RX xPointerQueue id=0x%02x val=%u seq=%u tick=%u\r\n",
                            ( unsigned int ) ptrReadMsgObj->id,
                            ( unsigned int ) ptrReadMsgObj->val,
                            ( unsigned int ) ptrReadMsgObj->seq,
                            ( unsigned int ) ptrReadMsgObj->tick );
            }
        }
    }
}

/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
}
