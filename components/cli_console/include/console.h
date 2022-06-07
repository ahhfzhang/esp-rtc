
#ifndef _CONSOLE_H
#define _CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_console.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
void console_init();

#ifdef __cplusplus
}
#endif

#endif