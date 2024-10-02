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
#include "cdc_uart.h"

#include <pico/stdlib.h>

#include "FreeRTOS.h"
// #include "probe_config.h"
#include "task.h"
#include "tusb.h"

// Actually s^-1 so 25ms
#define DEBOUNCE_MS 40

static cdc_uart_t* cdc_interfaces[2];

void cdc_uart_init(cdc_uart_t* cdc, const cdc_uart_config_t* config) {
  memset(cdc, 0x0, sizeof(*cdc));
  cdc->interval = 100;
  cdc->debounce_ticks = 5;
  cdc->config = config;
  cdc_interfaces[config->usb_interface] = cdc;

  gpio_set_function(config->tx_pin, GPIO_FUNC_UART);
  gpio_set_function(config->rx_pin, GPIO_FUNC_UART);
  gpio_set_pulls(config->tx_pin, 1, 0);
  gpio_set_pulls(config->rx_pin, 1, 0);
  uart_init(config->instance, config->baudrate);

  if (config->cts_pin >= 0) {
    /* HWFC implies that hardware flow control is implemented and the
     * UART operates in "full-duplex" mode (See USB CDC PSTN120 6.3.12).
     * Default to pulling in the active direction, so an unconnected CTS
     * behaves the same as if CTS were not enabled. */
    gpio_set_pulls(config->cts_pin, 0, 1);
    gpio_set_function(config->cts_pin, GPIO_FUNC_UART);
    gpio_set_function(config->cts_pin, GPIO_FUNC_UART);
    uart_set_hw_flow(config->instance, true, true);
  }
  if (config->rts_pin >= 0) {
    gpio_init(config->rts_pin);
    gpio_set_dir(config->rts_pin, GPIO_OUT);
    gpio_put(config->rts_pin, 1);
  }

  if (config->dtr_pin >= 0) {
    gpio_init(config->dtr_pin);
    gpio_set_dir(config->dtr_pin, GPIO_OUT);
    gpio_put(config->dtr_pin, 1);
  }
}

void cdc_activate_rx_led(cdc_uart_t* cdc) {
  if (cdc->config->rx_led_pin >= 0) {
    gpio_put(cdc->config->rx_led_pin, 1);
    cdc->rx_led_debounce = cdc->debounce_ticks;
  }
}

void cdc_debounce_rx_led(cdc_uart_t* cdc) {
  if (cdc->config->rx_led_pin >= 0) {
    if (cdc->rx_led_debounce)
      cdc->rx_led_debounce--;
    else
      gpio_put(cdc->config->rx_led_pin, 0);
  }
}

void cdc_activate_tx_led(cdc_uart_t* cdc) {
  if (cdc->config->tx_led_pin >= 0) {
    gpio_put(cdc->config->tx_led_pin, 1);
    cdc->tx_led_debounce = cdc->debounce_ticks;
  }
}

void cdc_debounce_tx_led(cdc_uart_t* cdc) {
  if (cdc->config->tx_led_pin >= 0) {
    if (cdc->tx_led_debounce)
      cdc->tx_led_debounce--;
    else
      gpio_put(cdc->config->tx_led_pin, 0);
  }
}

bool cdc_task(cdc_uart_t* cdc) {
  static int was_connected = 0;
  static uint cdc_tx_oe = 0;
  uint rx_len = 0;
  bool keep_alive = false;

  uint8_t itf = cdc->config->usb_interface;
  uart_inst_t* uart = cdc->config->instance;

  // Consume uart fifo regardless even if not connected
  while (uart_is_readable(uart) && (rx_len < sizeof(cdc->rx_buf))) {
    cdc->rx_buf[rx_len++] = uart_getc(uart);
  }

  if (tud_cdc_n_connected(itf)) {
    was_connected = 1;
    int written = 0;
    /* Implicit overflow if we don't write all the bytes to the host.
     * Also throw away bytes if we can't write... */
    if (rx_len) {
      cdc_activate_rx_led(cdc);
      written = MIN(tud_cdc_n_write_available(itf), rx_len);
      if (rx_len > written)
        cdc_tx_oe++;

      if (written > 0) {
        tud_cdc_n_write(itf, cdc->rx_buf, written);
        tud_cdc_n_write_flush(itf);
      }
    } else {
      cdc_debounce_rx_led(cdc);
    }

      /* Reading from a firehose and writing to a FIFO. */
    size_t watermark = MIN(tud_cdc_n_available(itf), sizeof(cdc->tx_buf));
    if (watermark > 0) {
      size_t tx_len;
      cdc_activate_tx_led(cdc);

      /* Batch up to half a FIFO of data - don't clog up on RX */
      watermark = MIN(watermark, 16);
      tx_len = tud_cdc_n_read(itf, cdc->tx_buf, watermark);
      uart_write_blocking(uart, cdc->tx_buf, tx_len);
    } else {
      cdc_debounce_tx_led(cdc);
    }
      /* Pending break handling */
    if (cdc->timed_break) {
      if (((int)cdc->break_expiry - (int)xTaskGetTickCount()) < 0) {
        cdc->timed_break = false;
        uart_set_break(uart, false);
        cdc->tx_led_debounce = 0;
      } else {
        keep_alive = true;
      }
    }
  } else if (was_connected) {
    tud_cdc_n_write_clear(itf);
    uart_set_break(uart, false);
    cdc->timed_break = false;
    was_connected = 0;
    cdc->tx_led_debounce = 0;
    cdc_tx_oe = 0;
  }
    return keep_alive;
}

void cdc_thread(void *ptr)
{
  cdc_uart_t* cdc = ptr;
  BaseType_t delayed;
  cdc->last_wake = xTaskGetTickCount();
  bool keep_alive;
  /* Threaded with a polling interval that scales according to linerate */
  while (1) {
    keep_alive = cdc_task(cdc);
    if (!keep_alive) {
      delayed = xTaskDelayUntil(&cdc->last_wake, cdc->interval);
      if (delayed == pdFALSE)
        cdc->last_wake = xTaskGetTickCount();
    }
  }
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* line_coding) {
  cdc_uart_t* cdc = cdc_interfaces[itf];
  if (cdc == NULL) {
    return;
  }
  uart_inst_t* uart = cdc->config->instance;

  uart_parity_t parity;
  uint data_bits, stop_bits;
  /* Set the tick thread interval to the amount of time it takes to
   * fill up half a FIFO. Millis is too coarse for integer divide.
   */
  uint32_t micros = (1000 * 1000 * 16 * 10) / MAX(line_coding->bit_rate, 1);
  /* Modifying state, so park the thread before changing it. */
  vTaskSuspend(cdc->uart_taskhandle);
  cdc->interval = MAX(1, micros / ((1000 * 1000) / configTICK_RATE_HZ));
  cdc->debounce_ticks =
      MAX(1, configTICK_RATE_HZ / (cdc->interval * DEBOUNCE_MS));
  probe_info("New baud rate %ld micros %ld interval %lu\n",
             line_coding->bit_rate,
             micros,
             cdc->interval);
  uart_deinit(uart);
  tud_cdc_n_write_clear(itf);
  tud_cdc_n_read_flush(itf);
  uart_init(uart, line_coding->bit_rate);

  switch (line_coding->parity) {
  case CDC_LINE_CODING_PARITY_ODD:
    parity = UART_PARITY_ODD;
    break;
  case CDC_LINE_CODING_PARITY_EVEN:
    parity = UART_PARITY_EVEN;
    break;
  default:
    probe_info("invalid parity setting %u\n", line_coding->parity);
    /* fallthrough */
  case CDC_LINE_CODING_PARITY_NONE:
    parity = UART_PARITY_NONE;
    break;
  }

  switch (line_coding->data_bits) {
  case 5:
  case 6:
  case 7:
  case 8:
    data_bits = line_coding->data_bits;
    break;
  default:
    probe_info("invalid data bits setting: %u\n", line_coding->data_bits);
    data_bits = 8;
    break;
  }

  /* The PL011 only supports 1 or 2 stop bits. 1.5 stop bits is translated to 2,
   * which is safer than the alternative. */
  switch (line_coding->stop_bits) {
  case CDC_LINE_CONDING_STOP_BITS_1_5:
  case CDC_LINE_CONDING_STOP_BITS_2:
    stop_bits = 2;
  break;
  default:
    probe_info("invalid stop bits setting: %u\n", line_coding->stop_bits);
    /* fallthrough */
  case CDC_LINE_CONDING_STOP_BITS_1:
    stop_bits = 1;
  break;
  }

  uart_set_format(uart, data_bits, stop_bits, parity);
  vTaskResume(cdc->uart_taskhandle);
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  cdc_uart_t* cdc = cdc_interfaces[itf];
  if (cdc == NULL) {
    return;
  }

  if (cdc->config->rts_pin >= 0) {
    gpio_put(cdc->config->rts_pin, !rts);
  }
  if (cdc->config->dtr_pin >= 0) {
    gpio_put(cdc->config->dtr_pin, !rts);
  }

  /* CDC drivers use linestate as a bodge to activate/deactivate the interface.
   * Resume our UART polling on activate, stop on deactivate */
  if (!dtr && !rts) {
    vTaskSuspend(cdc->uart_taskhandle);
    if (cdc->config->rx_led_pin >= 0) {
      gpio_put(cdc->config->rx_led_pin, 0);
      cdc->rx_led_debounce = 0;
    }
    if (cdc->config->tx_led_pin >= 0) {
      gpio_put(cdc->config->tx_led_pin, 0);
      cdc->tx_led_debounce = 0;
    }
  } else
    vTaskResume(cdc->uart_taskhandle);
}

void tud_cdc_send_break_cb(uint8_t itf, uint16_t wValue) {
  cdc_uart_t* cdc = cdc_interfaces[itf];
  if (cdc == NULL) {
    return;
  }
  uart_inst_t* uart = cdc->config->instance;

  switch(wValue) {
    case 0:
      uart_set_break(uart, false);
      cdc->timed_break = false;
      cdc->tx_led_debounce = 0;
      break;
    case 0xffff:
      uart_set_break(uart, true);
      cdc->timed_break = false;
      if (cdc->config->tx_led_pin >= 0) {
        gpio_put(cdc->config->tx_led_pin, 0);
        cdc->tx_led_debounce = 0;
      }
    break;
    default:
      uart_set_break(uart, true);
      cdc->timed_break = true;
      if (cdc->config->tx_led_pin >= 0) {
        gpio_put(cdc->config->tx_led_pin, 0);
        cdc->tx_led_debounce = 0;
      }
      cdc->break_expiry =
          xTaskGetTickCount() + (wValue * (configTICK_RATE_HZ / 1000));
      break;
  }
}
