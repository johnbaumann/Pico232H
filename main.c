#include <stdio.h>

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#ifdef PICO_STDIO_USB_CONNECTION_WITHOUT_DTR
#undef PICO_STDIO_USB_CONNECTION_WITHOUT_DTR
#endif

#define PICO_STDIO_USB_CONNECTION_WITHOUT_DTR 1

#ifdef PICO_DISABLE_SHARED_IRQ_HANDLERS
#undef PICO_DISABLE_SHARED_IRQ_HANDLERS
#endif
#define PICO_DISABLE_SHARED_IRQ_HANDLERS 1

void cpu_fifo();

void core1_entry() {
    cpu_fifo(); // No return
}

int main()
{
    //set_sys_clock_khz(250000, true);

    stdio_init_all();

    multicore_launch_core1(core1_entry);

    while(1) {
        sleep_ms(1000);
    }

    //cpu_fifo();
    /// No return
}
