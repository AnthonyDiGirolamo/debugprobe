/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef CDC_UART_H
#define CDC_UART_H

#include <hardware/uart.h>

#include "FreeRTOS.h"
#include "probe_config.h"
#include "task.h"

typedef struct {
  uart_inst_t* instance;
  uint baudrate;
  uint8_t usb_interface;
  uint tx_pin;
  uint rx_pin;

  // negative values of the below pins indicate that they are disabled.
  int rx_led_pin;
  int tx_led_pin;
  int cts_pin;
  int rts_pin;
  int dtr_pin;
} cdc_uart_config_t;

typedef struct {
  const cdc_uart_config_t* config;
  TaskHandle_t uart_taskhandle;
  TickType_t last_wake, interval;  // move to init = 100;
  volatile TickType_t break_expiry;
  volatile bool timed_break;

  /* Max 1 FIFO worth of data */
  uint8_t tx_buf[32];
  uint8_t rx_buf[32];

  uint debounce_ticks;  // move to init = 5;

  volatile uint tx_led_debounce;
  uint rx_led_debounce;

  int was_connected;
  uint cdc_tx_oe;
} cdc_uart_t;

void cdc_uart_init(cdc_uart_t* cdc, const cdc_uart_config_t* config);
void cdc_thread(void* ptr);
bool cdc_task(cdc_uart_t* cdc);

#endif
