/**************************************************************************************************
*  Filename:       i2cOptDriver.c
*  By:             Jesse Haviland
*  Created:        1 February 2019
*  Revised:        23 March 2019
*  Revision:       2.0
*
*  Description:    i2c Driver for use with opt3001.c and the TI OP3001 Optical Sensor
*************************************************************************************************/

// ----------------------- Includes -----------------------
#include <stdbool.h>
#include <stdint.h>

#include "i2cOptDriver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"


/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

extern uint32_t g_ui32SysClock;

/*
 * The binary semaphore 
 */
static SemaphoreHandle_t xI2CDoneSemaphore;
static SemaphoreHandle_t xI2CMutex;

typedef enum
{
    I2C_OP_NONE,
    I2C_OP_WRITE,
    I2C_OP_READ
} I2C_Operation_t;

typedef enum
{
    I2C_STATE_IDLE,

    /* Write states */
    I2C_STATE_WRITE_SEND_REG,
    I2C_STATE_WRITE_SEND_DATA0,
    I2C_STATE_WRITE_SEND_DATA1,

    /* Read states */
    I2C_STATE_READ_SEND_REG,
    I2C_STATE_READ_RESTART,
    I2C_STATE_READ_GET_BYTE0,
    I2C_STATE_READ_GET_BYTE1,

    I2C_STATE_DONE,
    I2C_STATE_ERROR
} I2C_State_t;

static volatile I2C_Operation_t g_i2cOp = I2C_OP_NONE;
static volatile I2C_State_t g_i2cState = I2C_STATE_IDLE;

static uint8_t g_txData[2];
static uint8_t g_rxData[2];
static uint8_t g_reg;
static uint8_t g_addr;
static bool g_success;

volatile uint32_t g_i2cIsrHit = 0;

void I2C0IntHandler(void);


void i2cOptDriverInit(void)
{
    //
    // The I2C0 peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Wait until both peripherals are fully clocked before touching their
    // registers. Skipping this causes intermittent failures on TM4C129x.
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

    //
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    // This step is not necessary if your part does not support pin muxing.
    //
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, g_ui32SysClock, false);
    

    xI2CDoneSemaphore = xSemaphoreCreateBinary();
    xI2CMutex = xSemaphoreCreateMutex();

    g_i2cOp = I2C_OP_NONE;
    g_i2cState = I2C_STATE_IDLE;
    g_success = false;

    //
    // Enable interrupts to the processor.
    //
    I2CMasterIntClear(I2C0_BASE);

    I2CIntRegister(I2C0_BASE, I2C0IntHandler);

    IntPrioritySet(INT_I2C0, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    I2CMasterIntEnable(I2C0_BASE);
    IntEnable(INT_I2C0);
}

/*
 * Sets slave address to ui8Addr
 * Puts ui8Reg followed by two data bytes in *data and transfers
 * over i2c
 */
bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Prevent starting if busy
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100)) != pdPASS)
        return false;

    if(g_i2cState != I2C_STATE_IDLE)
    {
        xSemaphoreGive(xI2CMutex);
        return false;
    }

    g_i2cOp = I2C_OP_WRITE;
    g_i2cState = I2C_STATE_WRITE_SEND_REG;
    g_addr = ui8Addr;
    g_reg = ui8Reg;
    g_txData[0] = data[0];
    g_txData[1] = data[1];
    g_success = false;

    g_i2cState = I2C_STATE_WRITE_SEND_REG;

    // Set slave address (write mode)
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);

    // Start transfer
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterIntClear(I2C0_BASE);
    I2CMasterIntEnable(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Semaphore will be given in I2C interrupt handler when transfer is complete
    if (xSemaphoreTake(xI2CDoneSemaphore, pdMS_TO_TICKS(50)) != pdPASS)
    {
        g_i2cState = I2C_STATE_IDLE;
        g_i2cOp = I2C_OP_NONE;
        g_success = false;
        xSemaphoreGive(xI2CMutex);
        return false;
    }

    xSemaphoreGive(xI2CMutex);
    return g_success;
}



/*
 * Sets slave address to ui8Addr
 * Writes ui8Reg over i2c to specify register being read from
 * Reads two bytes from i2c slave and stores them into *data
 */
bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100)) != pdPASS)
    {
        return false;
    }

    if (g_i2cState != I2C_STATE_IDLE)
    {
        xSemaphoreGive(xI2CMutex);
        return false;
    }

    g_i2cOp = I2C_OP_READ;
    g_i2cState = I2C_STATE_READ_SEND_REG;
    g_addr = ui8Addr;
    g_reg = ui8Reg;
    g_success = false;

    // Load device slave address and change I2C to write
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterIntEnable(I2C0_BASE);
    I2CMasterIntClear(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    if (xSemaphoreTake(xI2CDoneSemaphore, pdMS_TO_TICKS(50)) != pdPASS)
    {
        g_i2cState = I2C_STATE_IDLE;
        g_i2cOp = I2C_OP_NONE;
        g_success = false;
        xSemaphoreGive(xI2CMutex);
        return false;
    }

    if (g_success)
    {
        data[0] = g_rxData[0];
        data[1] = g_rxData[1];
    }

    xSemaphoreGive(xI2CMutex);
    return g_success;
}

void I2C0IntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t err;
    g_i2cIsrHit++;
    I2CMasterIntClear(I2C0_BASE);
    err = I2CMasterErr(I2C0_BASE);

    if (err != I2C_MASTER_ERR_NONE)
    {
        g_success = false;
        g_i2cState = I2C_STATE_ERROR;
    }
    else
    {
        switch (g_i2cState)
        {
            case I2C_STATE_WRITE_SEND_REG:
                I2CMasterDataPut(I2C0_BASE, g_txData[0]);
                g_i2cState = I2C_STATE_WRITE_SEND_DATA0;
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
                return;

            case I2C_STATE_WRITE_SEND_DATA0:
                I2CMasterDataPut(I2C0_BASE, g_txData[1]);
                g_i2cState = I2C_STATE_WRITE_SEND_DATA1;
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
                return;

            case I2C_STATE_WRITE_SEND_DATA1:
                g_success = true;
                g_i2cState = I2C_STATE_DONE;
                break;

            case I2C_STATE_READ_SEND_REG:
                I2CMasterSlaveAddrSet(I2C0_BASE, g_addr, true);
                g_i2cState = I2C_STATE_READ_RESTART;
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
                return;

            case I2C_STATE_READ_RESTART:
                g_rxData[0] = I2CMasterDataGet(I2C0_BASE);
                g_i2cState = I2C_STATE_READ_GET_BYTE0;
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                return;

            case I2C_STATE_READ_GET_BYTE0:
                g_rxData[1] = I2CMasterDataGet(I2C0_BASE);
                g_i2cState = I2C_STATE_READ_GET_BYTE1;
                g_success = true;
                g_i2cState = I2C_STATE_DONE;
                break;


            default:
                g_success = false;
                g_i2cState = I2C_STATE_ERROR;
                break;
        }
    }

    // Wake the blocked task
    xSemaphoreGiveFromISR(xI2CDoneSemaphore, &xHigherPriorityTaskWoken);

    g_i2cState = I2C_STATE_IDLE;
    g_i2cOp = I2C_OP_NONE;

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


