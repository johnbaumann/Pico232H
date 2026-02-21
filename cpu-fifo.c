// To-do: Bug with status register, getting a random value on FIFO when first
// booting

#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/regs/pio.h>
#include <pico/multicore.h>
#include <pico/platform.h>
#include <pico/platform/common.h>
#include <stdbool.h>
#include <stdint.h>
#include <tusb.h>

#include "cpu-fifo.pio.h"

typedef struct {
    int readProgramLoOffset;
    int readProgramHiOffset;
    int writeProgramLoOffset;
    int writeProgramHiOffset;
    int statusProgramOffset;
} programOffsets;

typedef struct {
    int channelA;
    int channelB;
} statusChannels;

enum Pin {
    PIN_D0 = 0u,
    PIN_D1 = 1u,
    PIN_D2 = 2u,
    PIN_D3 = 3u,
    PIN_D4 = 4u,
    PIN_D5 = 5u,
    PIN_D6 = 6u,
    PIN_D7 = 7u,
    PIN_D8 = 8u,
    PIN_D9 = 9u,
    PIN_D10 = 10u,
    PIN_D11 = 11u,
    PIN_D12 = 12u,
    PIN_D13 = 13u,
    PIN_D14 = 14u,
    PIN_D15 = 15u,
    PIN_CS = 16u,
    PIN_RD = 17u,
    PIN_WR = 18u,
    PIN_A0 = 19u,
    // The pins below are not physical connections
    // These are used for internal purposes only
    STATUS_D0 = 20u,
    STATUS_D1 = 21u,
    STATUS_D2 = 22u,
    STATUS_D3 = 23u,
};

static volatile unsigned int s_statusRegister = 0x0a;

static const PIO s_pioInstanceLo = pio0;
static const unsigned int s_smReadLo = 0;
static const unsigned int s_smStatus = 1;
static const unsigned int s_smWriteLo = 2;

static const PIO s_pioInstanceHi = pio1;
static const unsigned int s_smReadHi = 0;
static const unsigned int s_smWriteHi = 1;

static void core1_entry(void);
static statusChannels initDMA(void);
static void initGPIO(void);
static programOffsets initPIO(void);
static void initProgramRead(const PIO pio, const unsigned int sm, const unsigned int offset,
                            const unsigned int dataPinBase);
static void initProgramStatus(const PIO pio, const unsigned int sm, const unsigned int offset);
static void initProgramWrite(const PIO pio, const unsigned int sm, const unsigned int offset,
                             const unsigned int dataPinBase);
static void statusIRQHandler(void);
static void updateStatusRegister(void);
static void usbRead(void);
static void usbWrite(void);

static void core1_entry() {
    while (true) {
        updateStatusRegister();
    }
}

void cpu_fifo(void) {
    initGPIO();

    const programOffsets offsets = initPIO();

    multicore_launch_core1(core1_entry);

    const statusChannels channels = initDMA();

    tusb_init();

    while (true) {
        tud_task();

        usbRead();
        usbWrite();
    }
}
static statusChannels initDMA(void) {
    statusChannels channels;

    channels.channelA = dma_claim_unused_channel(true);
    channels.channelB = dma_claim_unused_channel(true);

    dma_channel_config dmaConfig = {0};

    // Shared channel settings
    channel_config_set_read_increment(&dmaConfig, false);
    channel_config_set_write_increment(&dmaConfig, false);
    channel_config_set_dreq(&dmaConfig, pio_get_dreq(s_pioInstanceLo, s_smStatus, true));
    channel_config_set_transfer_data_size(&dmaConfig, DMA_SIZE_8);
    channel_config_set_ring(&dmaConfig, false, 0);
    channel_config_set_bswap(&dmaConfig, false);
    channel_config_set_irq_quiet(&dmaConfig, false);
    channel_config_set_enable(&dmaConfig, true);
    channel_config_set_sniff_enable(&dmaConfig, false);
    channel_config_set_high_priority(&dmaConfig, false);

    // Channel A
    channel_config_set_chain_to(&dmaConfig, channels.channelB);
    dma_channel_configure(channels.channelA, &dmaConfig, &s_pioInstanceLo->txf[s_smStatus], &s_statusRegister, 1,
                          false);

    // Channel B
    channel_config_set_chain_to(&dmaConfig, channels.channelA);
    dma_channel_configure(channels.channelB, &dmaConfig, &s_pioInstanceLo->txf[s_smStatus], &s_statusRegister, 1, true);

    return channels;
}

static void initGPIO(void) {
    static const unsigned int controlPins[] = {
        PIN_CS,
        PIN_RD,
        PIN_WR,
        PIN_A0,
    };

    // Internal status data pins
    for (unsigned int pin = STATUS_D0; pin <= STATUS_D3; pin++) {
        pio_gpio_init(s_pioInstanceLo, pin);
        gpio_set_pulls(pin, true, true);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
    }

    // Data pins - Lo
    for (unsigned int pin = PIN_D0; pin <= PIN_D7; pin++) {
        pio_gpio_init(s_pioInstanceLo, pin);
        gpio_set_pulls(pin, false, false);
        gpio_set_input_enabled(pin, true);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_4MA);
    }

    // Data pins - Hi
    for (unsigned int pin = PIN_D8; pin <= PIN_D15; pin++) {
        pio_gpio_init(s_pioInstanceHi, pin);
        gpio_set_pulls(pin, false, false);
        gpio_set_input_enabled(pin, true);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_4MA);
    }

    // Control pins
    for (unsigned int pin = 0; pin < (sizeof(controlPins) / sizeof(controlPins[0])); pin++) {
        gpio_init(controlPins[pin]);
        gpio_set_pulls(controlPins[pin], false, false);
        gpio_set_input_enabled(controlPins[pin], true);
        gpio_set_slew_rate(controlPins[pin], GPIO_SLEW_RATE_FAST);
    }
}

static programOffsets initPIO(void) {
    programOffsets offsets;

    offsets.statusProgramOffset = pio_add_program(s_pioInstanceLo, &statusreg_program);
    initProgramStatus(s_pioInstanceLo, s_smStatus, offsets.statusProgramOffset);

    offsets.readProgramLoOffset = pio_add_program(s_pioInstanceLo, &readdata_program);
    offsets.readProgramHiOffset = pio_add_program(s_pioInstanceHi, &readdata_program);
    initProgramRead(s_pioInstanceLo, s_smReadLo, offsets.readProgramLoOffset, PIN_D0);
    initProgramRead(s_pioInstanceHi, s_smReadHi, offsets.readProgramHiOffset, PIN_D8);

    offsets.writeProgramLoOffset = pio_add_program(s_pioInstanceLo, &writedata_program);
    offsets.writeProgramHiOffset = pio_add_program(s_pioInstanceHi, &writedata_program);
    initProgramWrite(s_pioInstanceLo, s_smWriteLo, offsets.writeProgramLoOffset, PIN_D0);
    initProgramWrite(s_pioInstanceHi, s_smWriteHi, offsets.writeProgramHiOffset, PIN_D8);

    return offsets;
}

static void initProgramRead(const PIO pio, const unsigned int sm, const unsigned int offset,
                            const unsigned int dataPinBase) {
    pio_sm_config c = readdata_program_get_default_config(offset);

    sm_config_set_out_pins(&c, dataPinBase, 8);
    sm_config_set_out_shift(&c, true, false, 8);

    sm_config_set_in_pins(&c, PIN_A0);
    sm_config_set_in_shift(&c, true, false, 0);
    sm_config_set_jmp_pin(&c, PIN_RD);

    sm_config_set_set_pins(&c, dataPinBase, 5);           // Set pin D0 to D5 for the set(pindirs) instruction
    sm_config_set_sideset(&c, 3 + 1, true, true);         // 3 bits sideset + 1 bit for SIDE_EN(optional sideset)
    sm_config_set_sideset_pin_base(&c, dataPinBase + 5);  // Set the base pin for the sideset to 5th bit
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static void initProgramStatus(const PIO pio, const unsigned int sm, const unsigned int offset) {
    pio_sm_config c = statusreg_program_get_default_config(offset);

    pio_sm_set_consecutive_pindirs(pio, sm, STATUS_D0, 8, true);
    sm_config_set_out_pins(&c, STATUS_D0, 8);
    sm_config_set_out_shift(&c, true, true, 8);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static void initProgramWrite(const PIO pio, const unsigned int sm, const unsigned int offset,
                             const unsigned int dataPinBase) {
    pio_sm_config c = writedata_program_get_default_config(offset);

    pio_sm_set_consecutive_pindirs(pio, sm, dataPinBase, 8, false);  // Set the pin direction to input
    sm_config_set_in_pins(&c, dataPinBase);
    sm_config_set_jmp_pin(&c, PIN_WR);
    sm_config_set_in_shift(&c, false, true, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline void updateStatusRegister(void) {
    // Status register format:
    // Bit 0: Data Available (RXF)
    // Bit 1: Space Available (TXE)
    // Bit 2: Suspend(Not implemented, always 0)
    // Bit 3: Configured

    static const bool deviceConfigured = true;
    const uint32_t notFstat = ~s_pioInstanceLo->fstat;
    const bool dataAvailable = (notFstat & (1u << (PIO_FSTAT_TXEMPTY_LSB + s_smReadLo)));
    const bool spaceAvailable = (notFstat & (1u << (PIO_FSTAT_RXFULL_LSB + s_smWriteLo)));

    s_statusRegister = ((deviceConfigured << 3u) | (spaceAvailable << 1u) | (dataAvailable << 0u));
}

static inline void usbRead(void) {
    static uint8_t buffer[16];
    // USB READ, USB RX -> PIO TX
    const unsigned int available = tud_cdc_n_available(0);
    if (available && (available % 2) == 0) {
        if (!pio_sm_is_tx_fifo_full(s_pioInstanceLo, s_smReadLo)) {
            const unsigned int len = 8 - pio_sm_get_tx_fifo_level(s_pioInstanceLo, s_smReadLo);
            const unsigned int count = tud_cdc_n_read(0, buffer, len*2);

            for (unsigned int i = 0; i < count; i+=2) {
                pio_sm_put(s_pioInstanceHi, s_smReadHi, buffer[i]);
                pio_sm_put(s_pioInstanceLo, s_smReadLo, buffer[i+1]);
            }
        }
    }
}

static inline void usbWrite(void) {
    static uint8_t buffer[16];
    // USB WRITE, PIO RX -> USB TX
    if (!pio_sm_is_rx_fifo_empty(s_pioInstanceLo, s_smWriteLo)) {
        unsigned int len = pio_sm_get_rx_fifo_level(s_pioInstanceLo, s_smWriteLo) * 2;
        len = MIN(len, tud_cdc_n_write_available(0));

        if (len) {
            for (unsigned int i = 0; i < len; i+=2) {
                buffer[i] = pio_sm_get(s_pioInstanceHi, s_smWriteHi);
                buffer[i+1] = pio_sm_get(s_pioInstanceLo, s_smWriteLo);
            }

            // Data gets discarded if the USB is not connected
            if (tud_cdc_n_connected(0)) {
                tud_cdc_n_write(0, buffer, len);
                tud_cdc_n_write_flush(0);
            }
        }
    }
}