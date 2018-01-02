/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== i2ctmp007.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>

/* Example/Board Header files */
#include "Board.h"

#define TASKSTACKSIZE       640

#define TMP007_DIE_TEMP     0x0001  /* Die Temp Result Register */
#define TMP007_OBJ_TEMP     0x0003  /* Object Temp Result Register */

static Display_Handle display;
I2C_Handle      i2c;    // i2c handle for BME280

#include "bme280.h"
// BME280 HAL
void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
    usleep(period * 1000);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    I2C_Transaction i2cTransaction;
    uint8_t txBuffer = reg_addr;

    i2cTransaction.slaveAddress = Board_BME280_ADDR;
    i2cTransaction.writeBuf = &txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = reg_data;
    i2cTransaction.readCount = len;

    if(!I2C_transfer(i2c, &i2cTransaction)) {
        Display_printf(display, 0, 0, "I2C Bus fault during reading\n");
        rslt = BME280_E_COMM_FAIL;
    }
    return rslt;
}

#define BME280_TXBUF_SIZE  20

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[BME280_TXBUF_SIZE];
    uint8_t rxBuffer;

    if(len > (BME280_TXBUF_SIZE - 1)) {
        Display_printf(display, 0, 0, "Buffer to long for I2C, buffer size = %d\n", len);
        return BME280_E_INVALID_LEN;
    }

    // compose write command
    txBuffer[0] = reg_addr;
    memcpy(txBuffer+1, reg_data, len);

    i2cTransaction.slaveAddress = Board_BME280_ADDR;
    i2cTransaction.writeBuf = &txBuffer;
    i2cTransaction.writeCount = len+1;
    i2cTransaction.readBuf = &rxBuffer;
    i2cTransaction.readCount = 1;

    if(!I2C_transfer(i2c, &i2cTransaction)) {
        Display_printf(display, 0, 0, "I2C Bus fault during reading\n");
        rslt = BME280_E_COMM_FAIL;
    }
    return rslt;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    I2C_Params      i2cParams;

    /* Call driver init functions */
    Display_init();
    GPIO_init();
    I2C_init();

    /* Open the HOST display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        while (1);
    }

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
    Display_printf(display, 0, 0, "Starting the bme280 example\n");

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_BME280, &i2cParams);
    if (i2c == NULL) {
        Display_printf(display, 0, 0, "Error Initializing I2C\n");
        while (1);
    }
    else {
        Display_printf(display, 0, 0, "I2C Initialized!\n");
    }

    struct bme280_dev dev;

    int8_t rslt = BME280_OK;

    dev.dev_id = BME280_I2C_ADDR_PRIM;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    rslt = bme280_init(&dev);

    Display_printf(display, 0, 0, "BME280 Init result = %d\n", rslt);

#if 0
    /* Point to the T ambient register and read its 2 bytes */
    txBuffer[0] = TMP007_OBJ_TEMP;
    i2cTransaction.slaveAddress = Board_TMP_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    /* Take 20 samples and print them out onto the console */
    for (i = 0; i < 20; i++) {
        if (I2C_transfer(i2c, &i2cTransaction)) {
            /* Extract degrees C from the received data; see TMP102 datasheet */
            temperature = (rxBuffer[0] << 6) | (rxBuffer[1] >> 2);

            /*
             * If the MSB is set '1', then we have a 2's complement
             * negative value which needs to be sign extended
             */
            if (rxBuffer[0] & 0x80) {
                temperature |= 0xF000;
            }
           /*
            * For simplicity, divide the temperature value by 32 to get rid of
            * the decimal precision; see TI's TMP007 datasheet
            */
            temperature /= 32;

            Display_printf(display, 0, 0, "Sample %u: %d (C)\n", i, temperature);
        }
        else {
            Display_printf(display, 0, 0, "I2C Bus fault\n");
        }

        /* Sleep for 1 second */
        sleep(1);
    }
#endif

    /* Deinitialized I2C */
    I2C_close(i2c);
    Display_printf(display, 0, 0, "I2C closed!\n");

    return (NULL);
}
