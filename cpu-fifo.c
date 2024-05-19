// Core 0 - UART Transciever
// Core 1 - Parallel FIFO

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#define FT_STATUS_DATA_AVAILABLE 0x01  // RXF
#define FT_STATUS_SPACE_AVAILABLE 0x02 // TXE
#define FT_STATUS_SUSPEND 0x04         // SUSP
#define FT_STATUS_CONFIGURED 0x08      // CONFIG

static const uint base_data_pin = 2;
static const uint data_mask = (uint32_t)(0xff) << base_data_pin;
static const uint cs_pin = 10;
static const uint addr_pin = 11;
static const uint rd_pin = 12;
static const uint wr_pin = 13;
static const uint oe_pin = 15;

void init_pins()
{
    // Setup DATA pins
    for (uint pin = base_data_pin; pin < base_data_pin + 8; pin++)
    {
        gpio_init(pin);
        gpio_set_input_enabled(pin, true);
        gpio_set_pulls(pin, false, false);
    }

    // CS + Addr + RD + WR pins
    for (uint pin = cs_pin; pin <= wr_pin; pin++)
    {
        gpio_init(pin);
        gpio_set_input_enabled(pin, true);
        gpio_set_pulls(pin, false, false);
    }

    gpio_init(oe_pin);
    gpio_set_input_enabled(oe_pin, true);
    gpio_set_pulls(oe_pin, false, true);
}

void core1_main()
{
    stdio_init_all();
    init_pins();
    printf("Hello from core 1!\n");
    uint8_t datastatus_reg[2] = {0x55, FT_STATUS_DATA_AVAILABLE | FT_STATUS_SPACE_AVAILABLE | FT_STATUS_CONFIGURED};

    uint32_t pin_states = 0;

    while (true)
    {
        pin_states = gpio_get_all();
        // While CS low:
        while (!(pin_states & (1 << cs_pin)) && (pin_states & (1 << oe_pin)))
        {
            // -Wait for RD or WR low
            // -if WR low, read data from pins
            // -else if RD low, write data to pins, switch to output
            if (!(pin_states & (1 << wr_pin)))
            {
                pin_states = gpio_get_all();
                // Read data from pins
                uint addr = (pin_states & (1 << addr_pin)) > 0;
                uint8_t data = 0;
                for (uint pin = 0; pin < 8; pin++)
                {
                    data |= gpio_get(base_data_pin + pin) << pin;
                }
                while (!gpio_get(wr_pin) && !gpio_get(cs_pin))
                    ;
                printf("Read data: %d to addr: %d\n", data, addr);
            }
            else if (!(pin_states & (1 << rd_pin)))
            {
                // Write data to pins
                uint addr = (gpio_get(addr_pin) > 0);
                gpio_set_dir_masked(data_mask, data_mask);
                gpio_put_masked(data_mask, datastatus_reg[addr] << base_data_pin);

                while (!gpio_get(rd_pin) && !gpio_get(cs_pin) && gpio_get(oe_pin))
                    ;
                gpio_set_dir_masked(data_mask, ~data_mask);
                // printf("Data requested at addr: %d\n", 0);
            }
            pin_states = gpio_get_all();
        }
    }
}