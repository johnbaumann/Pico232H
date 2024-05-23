#include <stdbool.h>
#include <stdint.h>

#pragma once

bool fifo_empty();
bool fifo_full();
char fifo_peek();
char fifo_pop();
bool fifo_push(int data);
