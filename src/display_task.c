#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"


#include "event_bits.h"
#include "sensor_task.h"

#include "grlib/grlib.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

/*-----------------------------------------------------------*/
/* Display layout constants */

/* Graph area */
#define GRAPH_X_MIN     10
#define GRAPH_X_MAX     310
#define GRAPH_Y_MIN     30
#define GRAPH_Y_MAX     190
#define GRAPH_WIDTH     (GRAPH_X_MAX - GRAPH_X_MIN)
#define GRAPH_HEIGHT    (GRAPH_Y_MAX - GRAPH_Y_MIN)

/* Status bar at bottom */
#define STATUS_Y        200

/* Max lux for scaling — adjust to your environment */
#define LUX_DISPLAY_MAX  1000.0f

/* Number of x-pixels = number of data points we store */
#define GRAPH_POINTS    GRAPH_WIDTH

/*-----------------------------------------------------------*/
/* Plot mode */
typedef enum {
    PLOT_FILTERED_ONLY = 0,
    PLOT_RAW_AND_FILTERED
} PlotMode_t;

/*-----------------------------------------------------------*/
/* Globals */

extern uint32_t g_ui32SysClock;

/* Event group — defined here, extern in event_bits.h */
EventGroupHandle_t xSensorEventGroup = NULL;

static tContext sContext;

/* Circular buffers for graph data */
static float fRawBuf[GRAPH_POINTS]      = {0};
static float fFilteredBuf[GRAPH_POINTS] = {0};
static uint32_t ui32BufIndex = 0;
static uint32_t ui32BufCount = 0;

static PlotMode_t ePlotMode   = PLOT_FILTERED_ONLY;
static bool bDisplayHold      = false;

/* Button press flags — set by ISR, processed by task */
static volatile uint32_t ui32ButtonFlags = 0;
#define BTN_FLAG_TOGGLE_PLOT  (1UL << 0)
#define BTN_FLAG_DISPLAY_HOLD (1UL << 1)

/* Button debounce timer */
static TickType_t xLastButtonTime = 0;
#define BTN_DEBOUNCE_MS 200

/*-----------------------------------------------------------*/
/* Button ISR — Port J */

void GPIOJIntHandler(void)
{
    uint32_t ui32Status;
    TickType_t xCurrentTime = xTaskGetTickCountFromISR();

    ui32Status = GPIOIntStatus(GPIO_PORTJ_BASE, true);
    GPIOIntClear(GPIO_PORTJ_BASE, ui32Status);

    /* Simple debounce check */
    if ((xCurrentTime - xLastButtonTime) < pdMS_TO_TICKS(BTN_DEBOUNCE_MS))
    {
        return;
    }
    xLastButtonTime = xCurrentTime;

    /* Set volatile flags for task to process */
    if (ui32Status & USR_SW1)
    {
        ui32ButtonFlags |= BTN_FLAG_TOGGLE_PLOT;
    }

    if (ui32Status & USR_SW2)
    {
        ui32ButtonFlags |= BTN_FLAG_DISPLAY_HOLD;
    }
}

/*-----------------------------------------------------------*/
/* Configure button interrupts */

static void prvConfigureButtons(void)
{
    /* Port J already enabled by PinoutSet */
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, USR_SW1 | USR_SW2);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, USR_SW1 | USR_SW2,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOIntTypeSet(GPIO_PORTJ_BASE, USR_SW1 | USR_SW2, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTJ_BASE, USR_SW1 | USR_SW2);

    IntPrioritySet(INT_GPIOJ, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_GPIOJ);
}

/*-----------------------------------------------------------*/
/* Draw the graph border and labels */

static void prvDrawGraphBorder(void)
{
    tRectangle sRect;

    /* Black background */
    sRect.i16XMin = GRAPH_X_MIN;
    sRect.i16YMin = GRAPH_Y_MIN;
    sRect.i16XMax = GRAPH_X_MAX;
    sRect.i16YMax = GRAPH_Y_MAX;
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);

    /* White border */
    GrContextForegroundSet(&sContext, ClrWhite);
    GrRectDraw(&sContext, &sRect);

    /* Title */
    GrContextFontSet(&sContext, &g_sFontCm14);
    GrContextForegroundSet(&sContext, ClrWhite);
    GrStringDrawCentered(&sContext, "Lux vs Time", -1,
                         160, 12, true);
}

/*-----------------------------------------------------------*/
/* Convert a lux value to a Y pixel coordinate */

static int16_t prvLuxToY(float fLux)
{
    if (fLux < 0.0f) fLux = 0.0f;
    if (fLux > LUX_DISPLAY_MAX) fLux = LUX_DISPLAY_MAX;

    /* Map 0..LUX_DISPLAY_MAX to GRAPH_Y_MAX..GRAPH_Y_MIN (inverted) */
    return (int16_t)(GRAPH_Y_MAX -
            (int32_t)((fLux / LUX_DISPLAY_MAX) * GRAPH_HEIGHT));
}

/*-----------------------------------------------------------*/
/* Redraw the entire graph from the circular buffer */

static void prvRedrawGraph(void)
{
    uint32_t i;
    uint32_t ui32Start;
    int16_t i16X, i16Y, i16YPrev;
    int16_t i16YFilt, i16YFiltPrev;
    uint32_t ui32Count;
    tRectangle sRect;

    /* Clear graph area */
    sRect.i16XMin = GRAPH_X_MIN + 1;
    sRect.i16YMin = GRAPH_Y_MIN + 1;
    sRect.i16XMax = GRAPH_X_MAX - 1;
    sRect.i16YMax = GRAPH_Y_MAX - 1;
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);

    ui32Count = (ui32BufCount < GRAPH_POINTS) ? ui32BufCount : GRAPH_POINTS;
    if (ui32Count < 2) return;

    /* Find start index in circular buffer */
    ui32Start = (ui32BufIndex + GRAPH_POINTS - ui32Count) % GRAPH_POINTS;

    /* Draw filtered (green) */
    GrContextForegroundSet(&sContext, ClrGreen);
    i16YFiltPrev = prvLuxToY(fFilteredBuf[ui32Start]);

    for (i = 1; i < ui32Count; i++)
    {
        uint32_t idx = (ui32Start + i) % GRAPH_POINTS;
        i16X = (int16_t)(GRAPH_X_MIN + (i * GRAPH_WIDTH / ui32Count));
        i16YFilt = prvLuxToY(fFilteredBuf[idx]);

        GrLineDraw(&sContext,
                   i16X - (int16_t)(GRAPH_WIDTH / ui32Count),
                   i16YFiltPrev,
                   i16X,
                   i16YFilt);
        i16YFiltPrev = i16YFilt;
    }

    /* Draw raw (yellow) only in RAW_AND_FILTERED mode */
    if (ePlotMode == PLOT_RAW_AND_FILTERED)
    {
        GrContextForegroundSet(&sContext, ClrYellow);
        i16YPrev = prvLuxToY(fRawBuf[ui32Start]);

        for (i = 1; i < ui32Count; i++)
        {
            uint32_t idx = (ui32Start + i) % GRAPH_POINTS;
            i16X = (int16_t)(GRAPH_X_MIN + (i * GRAPH_WIDTH / ui32Count));
            i16Y = prvLuxToY(fRawBuf[idx]);

            GrLineDraw(&sContext,
                       i16X - (int16_t)(GRAPH_WIDTH / ui32Count),
                       i16YPrev,
                       i16X,
                       i16Y);
            i16YPrev = i16Y;
        }
    }
}

/*-----------------------------------------------------------*/
/* Update status bar with event info */

static void prvUpdateStatus(EventBits_t xBits)
{
    tRectangle sRect;

    /* Clear status area */
    sRect.i16XMin = 0;
    sRect.i16YMin = STATUS_Y;
    sRect.i16XMax = 319;
    sRect.i16YMax = 239;
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);

    GrContextFontSet(&sContext, &g_sFontCm12);

    /* Build status string */
    if (xBits & EVENT_HIGH_THRESHOLD)
    {
        GrContextForegroundSet(&sContext, ClrRed);
        GrStringDraw(&sContext, "! HIGH LUX", -1, 5, STATUS_Y, true);
    }
    else if (xBits & EVENT_LOW_THRESHOLD)
    {
        GrContextForegroundSet(&sContext, ClrBlue);
        GrStringDraw(&sContext, "! LOW LUX", -1, 5, STATUS_Y, true);
    }
    else
    {
        GrContextForegroundSet(&sContext, ClrGreen);
        GrStringDraw(&sContext, "OK", -1, 5, STATUS_Y, true);
    }

    if (xBits & EVENT_SENSOR_MISSED)
    {
        GrContextForegroundSet(&sContext, ClrOrange);
        GrStringDraw(&sContext, "MISSED", -1, 80, STATUS_Y, true);
    }

    if (xBits & EVENT_QUEUE_FULL)
    {
        GrContextForegroundSet(&sContext, ClrRed);
        GrStringDraw(&sContext, "Q-FULL", -1, 150, STATUS_Y, true);
    }

    /* Plot mode indicator */
    GrContextForegroundSet(&sContext, ClrWhite);
    if (ePlotMode == PLOT_RAW_AND_FILTERED)
        GrStringDraw(&sContext, "RAW+FILT", -1, 220, STATUS_Y, true);
    else
        GrStringDraw(&sContext, "FILT", -1, 260, STATUS_Y, true);

    /* Hold indicator */
    if (bDisplayHold)
    {
        GrContextForegroundSet(&sContext, ClrYellow);
        GrStringDraw(&sContext, "HOLD", -1, 220, STATUS_Y + 14, true);
    }
}

/*-----------------------------------------------------------*/
/* Display task */

static void vDisplayTask(void *pvParameters)
{
    SensorMsg_t xMsg;
    EventBits_t xBits;
    (void)pvParameters;

    /* Init display */
    Kentec320x240x16_SSD2119Init(g_ui32SysClock);
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

    /* Clear screen */
    tRectangle sRect = {0, 0, 319, 239};
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);

    prvDrawGraphBorder();

    for (;;)
    {
        /*--------------------------------------------------
         * 1. Drain the sensor queue
         *--------------------------------------------------*/
        while (xQueueReceive(xSensorQueue, &xMsg, 0) == pdPASS)
        {
            /* Always store data in buffer even when held */
            fRawBuf[ui32BufIndex]      = xMsg.fRawLux;
            fFilteredBuf[ui32BufIndex] = xMsg.fFilteredLux;
            ui32BufIndex = (ui32BufIndex + 1) % GRAPH_POINTS;
            if (ui32BufCount < GRAPH_POINTS) ui32BufCount++;
        }

        /*--------------------------------------------------
         * 2. Process button flags (set by ISR)
         *--------------------------------------------------*/
        if (ui32ButtonFlags & BTN_FLAG_TOGGLE_PLOT)
        {
            ePlotMode = (ePlotMode == PLOT_FILTERED_ONLY)
                        ? PLOT_RAW_AND_FILTERED
                        : PLOT_FILTERED_ONLY;
            ui32ButtonFlags &= ~BTN_FLAG_TOGGLE_PLOT;
            xEventGroupSetBits(xSensorEventGroup, EVENT_BTN_TOGGLE_PLOT);
        }

        if (ui32ButtonFlags & BTN_FLAG_DISPLAY_HOLD)
        {
            bDisplayHold = !bDisplayHold;
            ui32ButtonFlags &= ~BTN_FLAG_DISPLAY_HOLD;
            xEventGroupSetBits(xSensorEventGroup, EVENT_DISPLAY_HOLD);
        }

        /*--------------------------------------------------
         * 3. Update graph only if not held
         *--------------------------------------------------*/
        static bool bWasHeld = false;

        if (!bDisplayHold)
        {
            prvRedrawGraph();
            prvDrawGraphBorder();
            bWasHeld = false;
        }
        else if (!bWasHeld)
        {
            /* First frame of hold — draw once to freeze current state */
            prvRedrawGraph();
            prvDrawGraphBorder();
            bWasHeld = true;
        }

        /*--------------------------------------------------
         * 4. Check event group — non-blocking (0 timeout)
         *--------------------------------------------------*/
        xBits = xEventGroupWaitBits(
                    xSensorEventGroup,
                    EVENT_HIGH_THRESHOLD | EVENT_LOW_THRESHOLD |
                    EVENT_SENSOR_MISSED  | EVENT_QUEUE_FULL    |
                    EVENT_BTN_TOGGLE_PLOT| EVENT_DISPLAY_HOLD,
                    pdFALSE,      /* don't clear on exit */
                    pdFALSE,      /* wait for ANY bit */
                    0);           /* 0 timeout — non-blocking */

        /* Clear button bits after handling (they were set by task above) */
        xEventGroupClearBits(xSensorEventGroup,
                             EVENT_BTN_TOGGLE_PLOT | EVENT_DISPLAY_HOLD);

        /* Clear transient bits after handling */
        xEventGroupClearBits(xSensorEventGroup,
                             EVENT_SENSOR_MISSED | EVENT_QUEUE_FULL);

        /*--------------------------------------------------
         * 5. Update status bar
         *--------------------------------------------------*/
        prvUpdateStatus(xBits);

        /* Debug heartbeat */
        static uint32_t ui32Heartbeat = 0;
        if (++ui32Heartbeat % 20 == 0) {  // Every ~1 second
            UARTprintf("Display task running...\n");
        }

        /* 50ms gives ~20 display updates/sec — plenty for 5Hz sensor */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/*-----------------------------------------------------------*/

void vDisplayDemoTask(void)
{
    prvConfigureButtons();

    xTaskCreate(vDisplayTask,
                "DisplayTask",
                1024,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
}