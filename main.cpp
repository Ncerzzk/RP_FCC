/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
extern "C"
{
#include "minihdlc/minihdlc.h"
}

#define UART_ID uart0

#define UART_TX_PIN 0
#define UART_RX_PIN 1

void on_uart_rx()
{
    while (uart_is_readable(UART_ID))
    {
        uint8_t ch = uart_getc(UART_ID);
        // Can we send it back?
        minihdlc_char_receiver(ch);
    }
}

void send_char(uint8_t data)
{
    uart_putc(UART_ID, data);
}

void frame_received(const uint8_t *frame_buffer, uint16_t frame_length)
{
    printf("Received frame: ");
    printf("%s\n", frame_buffer);
}

int main()
{
    uart_init(UART_ID, 115200);
    stdio_usb_init();

    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);

    uart_set_fifo_enabled(UART_ID, false);

    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);

    uart_set_irq_enables(UART_ID, true, false); // only enable rx irq

    minihdlc_init(send_char, frame_received);

    printf("hello,world!\n");
    while (1)
    {
        printf("wake\n");
        sleep_ms(1000);
    }
    // tight_loop_contents();
}