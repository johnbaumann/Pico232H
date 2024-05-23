#include "fifo.h"

#include <stdbool.h>
#include <stdint.h>

char fifo_head = 0;
char fifo_tail = 0;
char fifo[8];

inline bool fifo_empty()
{
    return fifo_head == fifo_tail;
}

inline bool fifo_full()
{
    return fifo_head == (fifo_tail + 1) % 8;
}

inline char fifo_peek()
{
    if (fifo_head == fifo_tail)
    {
        return 0;
    }
    return fifo[fifo_head];
}

inline char fifo_pop()
{
    if (fifo_head == fifo_tail)
    {
        return 0;
    }
    char data = fifo[fifo_head];
    fifo_head = (fifo_head + 1) % 8;
    return data;
}

inline bool fifo_push(int data)
{
    if (fifo_full())
    {
        return false;
    }
    fifo[fifo_tail] = data;
    fifo_tail = (fifo_tail + 1) % 8;
    return true;
}