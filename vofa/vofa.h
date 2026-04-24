#include "usart.h"
#include <string.h>
#include <stdarg.h>

#define MAX_CHANNELS 32

typedef struct 
{
    float data[MAX_CHANNELS];
    uint8_t tail[4];
}vofa_send_t;

void vofa_send_data(uint8_t num, ...);
