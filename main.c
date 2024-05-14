/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "cpu-fifo.pio.h"

void init_cpu_fifo();

char fifo_head = 0;
char fifo_tail = 0;
char fifo[8];

bool fifo_empty()
{
    return fifo_head == fifo_tail;
}

bool fifo_full()
{
    return fifo_head == (fifo_tail + 1) % 8;
}

char fifo_peek()
{
    if (fifo_head == fifo_tail)
    {
        return 0;
    }
    return fifo[fifo_head];

}

char fifo_pop()
{
    if (fifo_head == fifo_tail)
    {
        return 0;
    }
    char data = fifo[fifo_head];
    fifo_head = (fifo_head + 1) % 8;
    return data;
}

bool fifo_push(int data)
{
    if (fifo_full())
    {
        return false;
    }
    fifo[fifo_tail] = data;
    fifo_tail = (fifo_tail + 1) % 8;
    return true;
}

uint8_t data[2] = {0x00, 0x0A};
int addr = 0x00;

int main()
{
    stdio_init_all();

    printf("Hello, world!\n");

    init_cpu_fifo();
}

void init_cpu_fifo()
{
    static PIO pio_cpufifo = pio0;
    //static PIO pio_writefifo = pio1;

    // Setup data pins and data out state machine
    static const uint base_data_pin = 2;
    static const uint cs_pin = 16;
    static const uint addr_pin = 17;
    static const uint rd_pin = 18;
    static const uint wr_pin = 19;

    uint sm_dataout = pio_claim_unused_sm(pio_cpufifo, true);
    uint sm_readdata = pio_claim_unused_sm(pio_cpufifo, true);

    uint sm_writedata = pio_claim_unused_sm(pio_cpufifo, true);

    // Setup DATA pins
    for (uint pin = 0; pin < 8; pin++)
    {
        pio_gpio_init(pio_cpufifo, base_data_pin + pin);
        gpio_set_input_enabled(base_data_pin + pin, false);
    }

    // CS + Addr + RD + WR pins
    for (uint pin = cs_pin; pin < rd_pin + 1; pin++)
    {
        pio_gpio_init(pio_cpufifo, pin);
        gpio_set_pulls(pin, true, false);
        gpio_set_input_enabled(pin, true);
    }

    // Setup data out state machine
    uint offset_dataout = pio_add_program(pio_cpufifo, &dataout_program);
    pio_sm_config c_dataout = dataout_program_get_default_config(offset_dataout);

    sm_config_set_out_pins(&c_dataout, base_data_pin, 8);
    sm_config_set_out_shift(&c_dataout, true, true, 8);

    pio_sm_init(pio_cpufifo, sm_dataout, offset_dataout, &c_dataout);
    pio_sm_set_enabled(pio_cpufifo, sm_dataout, true);

    // Setup read data state machine(read data from RP2040)
    uint offset_readdata = pio_add_program(pio_cpufifo, &readdata_program);
    pio_sm_config c_readdata = readdata_program_get_default_config(offset_readdata);

    //pio_sm_set_consecutive_pindirs(pio_cpufifo, sm_readdata, cs_pin, 1, false);
    pio_sm_set_consecutive_pindirs(pio_cpufifo, sm_writedata, base_data_pin, cs_pin - base_data_pin, false);
    sm_config_set_in_pins(&c_readdata, base_data_pin);
    sm_config_set_out_pins(&c_readdata, base_data_pin, 8);
    sm_config_set_jmp_pin(&c_readdata, rd_pin);
    sm_config_set_in_shift(&c_readdata, true, false, 32);

    pio_sm_init(pio_cpufifo, sm_readdata, offset_readdata, &c_readdata);
    pio_sm_set_enabled(pio_cpufifo, sm_readdata, true);


    // Setup WR state machine
    pio_gpio_init(pio_cpufifo, cs_pin);
    pio_gpio_init(pio_cpufifo, wr_pin);
    gpio_set_pulls(cs_pin, true, false);
    gpio_set_pulls(wr_pin, true, false);
    gpio_set_input_enabled(cs_pin, true);
    gpio_set_input_enabled(wr_pin, true);

    uint offset_writedata = pio_add_program(pio_cpufifo, &writedata_program);
    pio_sm_config c_writedata = writedata_program_get_default_config(offset_writedata);

    for (uint pin = base_data_pin; pin <= cs_pin; pin++)
    {
        pio_gpio_init(pio_cpufifo, pin);
        gpio_set_input_enabled(pin, true);
    }
    sm_config_set_in_pins(&c_writedata, base_data_pin);
    sm_config_set_jmp_pin(&c_writedata, wr_pin);
    sm_config_set_in_shift(&c_writedata, true, false, 32);

    pio_sm_init(pio_cpufifo, sm_writedata, offset_writedata, &c_writedata);
    pio_sm_set_enabled(pio_cpufifo, sm_writedata, true);


    char temp = 0;

    pio_cpufifo->txf[sm_dataout] = 0xAA;

    while (true)
    {
        // addr = gpio_get(addr_pin) > 0;

        if (!fifo_full() && uart_is_readable(uart0)) // RX
        {
            temp = uart_getc(uart0);
            fifo_push(temp);
            printf("UART Received: %c\n", temp);
        }

        if (!fifo_empty())
        {
            temp = fifo_pop();
            printf("Sending: %c\n", temp);
            pio_cpufifo->txf[sm_dataout] = temp;
        }

        // TX
        if (!pio_sm_is_rx_fifo_empty(pio_cpufifo, sm_writedata))
        {
            printf("Received: %x\n", pio_cpufifo->rxf[sm_writedata]);
        }
    }
}
