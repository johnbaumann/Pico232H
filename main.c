#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

void cpu_fifo();

int main()
{
    stdio_init_all();

    cpu_fifo();
    // No return
}
