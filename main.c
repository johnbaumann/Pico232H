/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fifo.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "cpu-fifo.pio.h"

#define FT_STATUS_DATA_AVAILABLE 0x01  // RXF
#define FT_STATUS_SPACE_AVAILABLE 0x02 // TXE
#define FT_STATUS_SUSPEND 0x04         // SUSP
#define FT_STATUS_CONFIGURED 0x08      // CONFIG

#define IRQ_READDATA_SYS 0
#define IRQ_READSTATUS_SYS 1
#define IRQ_WRITEDATA_SYS 2

union datastatus
{
    uint16_t value;
    struct
    {
        uint8_t data;
        uint8_t status;
    };
};

union datastatus datastatus_reg;

static PIO pio_cpufifo = pio1;
static PIO pio_writefifo = pio1;

static uint sm_cpufifo;
static uint sm_readdata;
static uint sm_readstatus;

static uint sm_writedata;

static int8_t pio_irq_data;
static int8_t pio_irq_status;

void init_cpu_fifo();
void core1_main();

int main()
{
    stdio_init_all();

    printf("Hello, world!\n");

    // multicore_launch_core1(core1_main);
    init_cpu_fifo();
    // core1_main();
    while (1)
    {
        sleep_ms(1000);
    }
}

static inline void update_status_register()
{
    if (pio_sm_is_tx_fifo_empty(pio_cpufifo, sm_readdata))
    {
        datastatus_reg.status &= ~FT_STATUS_DATA_AVAILABLE;
    }
    else
    {
        datastatus_reg.status |= FT_STATUS_DATA_AVAILABLE;
    }
}

static void pio_data_irq_func(void)
{
    // Update the status register fifo
    pio_sm_drain_tx_fifo(pio_cpufifo, sm_readstatus);
    update_status_register();
    pio_cpufifo->txf[sm_readstatus] = datastatus_reg.status;

    // Update the data register fifo
    if (!fifo_empty())
    {
        datastatus_reg.data = fifo_pop();
        pio_cpufifo->txf[sm_readdata] = datastatus_reg.value;
    }
    pio_interrupt_clear(pio_cpufifo, IRQ_READDATA_SYS);
}

static void pio_status_irq_func(void)
{
    uint8_t irq_mask = pio_cpufifo->irq & 0xff;

    pio_sm_drain_tx_fifo(pio_cpufifo, sm_readstatus);
    update_status_register();
    pio_cpufifo->txf[sm_readstatus] = datastatus_reg.status;

    if (irq_mask & (1u << IRQ_READSTATUS_SYS))
    {
        pio_interrupt_clear(pio_cpufifo, IRQ_READSTATUS_SYS);
    }

    if (irq_mask & (1u << IRQ_READDATA_SYS))
    {
        pio_interrupt_clear(pio_cpufifo, IRQ_READDATA_SYS);
    }
}

void cpufifo_program_init(PIO pio, uint sm, uint offset)
{
    pio_sm_config c = cpufifo_program_get_default_config(offset);
    // Setup data pins and data out state machine
    static const uint base_data_pin = 2;
    static const uint cs_pin = 10;
    static const uint addr_pin = 11;
    static const uint rd_pin = 12;

    static const uint pin_count = rd_pin - base_data_pin;

    // Data pins
    for (uint pin = base_data_pin; pin < base_data_pin + 8; pin++)
    {
        pio_gpio_init(pio, pin);
        gpio_set_input_enabled(pin, false);
    }

    // CS + Addr + RD pins
    for (uint pin = cs_pin; pin <= rd_pin; pin++)
    {
        pio_gpio_init(pio, pin);
        gpio_set_pulls(pin, true, false);
        gpio_set_input_enabled(pin, true);
    }

    pio_sm_set_consecutive_pindirs(pio, sm, base_data_pin, pin_count, false);

    sm_config_set_in_pins(&c, base_data_pin);
    sm_config_set_out_pins(&c, base_data_pin, 8);
    sm_config_set_jmp_pin(&c, rd_pin);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_out_shift(&c, true, true, 32);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void readdata_program_init(PIO pio, uint sm, uint offset)
{
    static const uint base_data_pin = 2;

    pio_sm_config c = readdata_program_get_default_config(offset);

    // Data pins
    for (uint pin = base_data_pin; pin < base_data_pin + 8; pin++)
    {
        pio_gpio_init(pio, pin);
        gpio_set_input_enabled(pin, false);
    }

    pio_sm_set_consecutive_pindirs(pio, sm, base_data_pin, 8, false);
    sm_config_set_out_pins(&c, base_data_pin, 8);
    sm_config_set_out_shift(&c, true, false, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    pio_set_irqn_source_enabled(pio, pio_irq_status, pis_interrupt0, true);                                       // Set pio to tell us when irq0 is raised
    pio_interrupt_clear(pio, IRQ_READDATA_SYS);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void readstatus_program_init(PIO pio, uint sm, uint offset)
{
    static const uint base_data_pin = 2;

    pio_sm_config c = readstatus_program_get_default_config(offset);

    // Data pins
    for (uint pin = base_data_pin; pin < base_data_pin + 8; pin++)
    {
        pio_gpio_init(pio, pin);
        gpio_set_input_enabled(pin, false);
    }

    pio_sm_set_consecutive_pindirs(pio, sm, base_data_pin, 8, false);
    sm_config_set_out_pins(&c, base_data_pin, 8);
    sm_config_set_out_shift(&c, true, false, 8);

    pio_irq_status = (pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0;
    if (irq_get_exclusive_handler(pio_irq_status))
    {
        pio_irq_status++;
        if (irq_get_exclusive_handler(pio_irq_status))
        {
            panic("All IRQs are in use");
        }
    }

    // Enable interrupt
    irq_add_shared_handler(pio_irq_status, pio_status_irq_func, PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY); // Add a shared IRQ handler
    // irq_set_exclusive_handler(pio_irq_status, pio_status_irq_func);                    // Add an exclusive IRQ handler
    irq_set_enabled(pio_irq_status, true);                                             // Enable the IRQ
    const uint irq_index = pio_irq_status - ((pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0); // Get index of the IRQ
    pio_set_irqn_source_enabled(pio, irq_index, pis_interrupt1, true);                 // Set pio to tell us when irq1 is raised
    pio_interrupt_clear(pio, IRQ_READSTATUS_SYS);                                      // Clear any pending interrupts

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void init_cpu_fifo()
{
    static const uint base_data_pin = 2;
    static const uint cs_pin = 10;
    static const uint addr_pin = 11;
    static const uint wr_pin = 13;

    sm_cpufifo = pio_claim_unused_sm(pio_cpufifo, true);
    sm_readdata = pio_claim_unused_sm(pio_cpufifo, true);
    sm_readstatus = pio_claim_unused_sm(pio_cpufifo, true);
    // sm_writedata = pio_claim_unused_sm(pio_cpufifo, true);

    // Setup read data state machine(read data from RP2040)
    uint offset_cpufifo = pio_add_program(pio_cpufifo, &cpufifo_program);
    uint offset_readdata = pio_add_program(pio_cpufifo, &readdata_program);
    uint offset_readstatus = pio_add_program(pio_cpufifo, &readstatus_program);

    cpufifo_program_init(pio_cpufifo, sm_cpufifo, offset_cpufifo);
    readdata_program_init(pio_cpufifo, sm_readdata, offset_readdata);
    readstatus_program_init(pio_cpufifo, sm_readstatus, offset_readstatus);

    // Setup WR state machine
    /*pio_sm_set_consecutive_pindirs(pio_writefifo, sm_writedata, base_data_pin, cs_pin - base_data_pin, false);
    pio_gpio_init(pio_writefifo, cs_pin);
    pio_gpio_init(pio_writefifo, wr_pin);
    gpio_set_pulls(cs_pin, true, false);
    gpio_set_pulls(wr_pin, true, false);
    gpio_set_input_enabled(cs_pin, true);
    gpio_set_input_enabled(wr_pin, true);

    uint offset_writedata = pio_add_program(pio_writefifo, &writedata_program);
    pio_sm_config c_writedata = writedata_program_get_default_config(offset_writedata);

    for (uint pin = base_data_pin; pin <= wr_pin; pin++)
    {
        pio_gpio_init(pio_writefifo, pin);
        gpio_set_input_enabled(pin, true);
    }
    sm_config_set_in_pins(&c_writedata, base_data_pin);
    sm_config_set_jmp_pin(&c_writedata, wr_pin);
    sm_config_set_in_shift(&c_writedata, true, false, 32);

    pio_sm_init(pio_writefifo, sm_writedata, offset_writedata, &c_writedata);
    pio_sm_set_enabled(pio_writefifo, sm_writedata, true);*/

    char temp = 0;

    // Set initial data
    datastatus_reg.data = 0xff;
    datastatus_reg.status = FT_STATUS_CONFIGURED | FT_STATUS_SPACE_AVAILABLE;

    pio_cpufifo->txf[sm_readstatus] = datastatus_reg.status;

    while (true)
    {
        // addr = gpio_get(addr_pin) > 0;

        if (!pio_sm_is_tx_fifo_full(pio_cpufifo, sm_readdata) && uart_is_readable(uart0)) // RX
        {
            temp = uart_getc(uart0);
            pio_cpufifo->txf[sm_readdata] = temp;
            while(pio_sm_is_tx_fifo_empty(pio_cpufifo, sm_readdata))
                ;
            update_status_register();
            pio_sm_drain_tx_fifo(pio_cpufifo, sm_readstatus);
            pio_cpufifo->txf[sm_readstatus] = datastatus_reg.status;
            printf("%c", temp); // Echo character
        }

        /*if (!fifo_empty() && pio_sm_is_tx_fifo_full(pio_cpufifo, sm_readdata) == false) // TX
        {
            datastatus_reg.data = fifo_pop();
            printf("Sending: %x\n", datastatus_reg.value);
            pio_cpufifo->txf[sm_readdata] = datastatus_reg.value;
        }*/

        // TX
        /*if (!pio_sm_is_rx_fifo_empty(pio_writefifo, sm_writedata))
        {
            printf("Received: %x\n", pio_writefifo->rxf[sm_writedata]);
        }*/
    }
}
