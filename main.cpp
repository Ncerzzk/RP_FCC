/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include "PicoLed.hpp"
#include "PIO_DShot.h"

extern "C"
{
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "minihdlc/minihdlc.h"
#include "rp_fcc_protocol.h"
}

#define DSHOT_PIO pio0
#define PROGRAM_LED_PIO pio1

bi_decl(bi_program_feature_group(0x1111, 0, "UART Configuration"));
bi_decl(bi_ptr_int32(0x1111, 0, UART_TX_PIN, 0));
bi_decl(bi_ptr_int32(0x1111, 0, UART_RX_PIN, 1));
bi_decl(bi_ptr_int32(0x1111, 0, LED_PIN, 16));
bi_decl(bi_ptr_int32(0x1111, 0, LED_LENGTH, 4));
bi_decl(bi_ptr_int32(0x1111, 0, UART_ID, 0));
bi_decl(bi_ptr_int32(0x1111, 0, BAUDRATE, 1152000));

bi_decl(bi_program_feature_group(0x1111, 1, "DSHOT Configuration"));
bi_decl(bi_ptr_int32(0x1111, 1, DSHOT0_PIN, 25));
bi_decl(bi_ptr_int32(0x1111, 1, DSHOT1_PIN, 24));
bi_decl(bi_ptr_int32(0x1111, 1, DSHOT2_PIN, 23));
bi_decl(bi_ptr_int32(0x1111, 1, DSHOT3_PIN, 22));

uart_inst_t * UART_HW = uart0;
BidirDShotX1 *ESCs[4];

void on_uart_rx()
{
    while (uart_is_readable(UART_HW))
    {
        uint8_t ch = uart_getc(UART_HW);
        // Can we send it back?
        minihdlc_char_receiver(ch);
    }
}

void send_char(uint8_t data)
{
    uart_putc(UART_HW, data);
}

void frame_received(const uint8_t *frame_buffer, uint16_t frame_length)
{
    uint8_t frame_type = frame_buffer[0];
    if(frame_type == RP_FCC_DSHOT_CMD){
        rp_fcc_dshot_cmd_s *dshot_cmd = (rp_fcc_dshot_cmd_s *)frame_buffer;
        if(dshot_cmd->dshot_cmd >=48 ){
            printf("unknown dshot cmd:%d\n",dshot_cmd->dshot_cmd); 
            return;
        }

        for(int i = 0; i < DSHOT_CHANNEL_NUM; i++){
            if(dshot_cmd->channel_mask & (1 << i)){
                ESCs[i]->sendRaw11Bit(dshot_cmd->dshot_cmd);
            }
        }
    }else if(frame_type == RP_FCC_OUTPUT){
        rp_fcc_output_s *output = (rp_fcc_output_s *)frame_buffer;
        for(int i = 0; i < DSHOT_CHANNEL_NUM; i++){
            ESCs[i]->sendThrottle(output->outputs[i]);
        }
        // printf("rcv output:");
        // for(int i=0; i< 4 ;++i){
        //     printf("%d ", output->outputs[i]);
        // }
        // printf("\n");
    } else {
        printf("Unknown frame type: %d\n", frame_type);
    }
}

void led_init(){
    int sm = pio_claim_unused_sm(PROGRAM_LED_PIO, false);
    auto ledStrip = PicoLed::addLeds<PicoLed::WS2812B>(PROGRAM_LED_PIO, sm, LED_PIN, LED_LENGTH, PicoLed::FORMAT_GRB);
    ledStrip.setBrightness(100);
    ledStrip.fill({100,100,100,100});

    ledStrip.show(); 
}

int main()
{
    UART_HW = uart_get_instance(UART_ID);

    uart_init(UART_HW, BAUDRATE);

    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_HW, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_HW, UART_RX_PIN));

    uart_set_hw_flow(UART_HW, false, false);
    uart_set_format(UART_HW, 8, 1, UART_PARITY_NONE);

    uart_set_fifo_enabled(UART_HW, false);

    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);

    uart_set_irq_enables(UART_HW, true, false); // only enable rx irq

    ESCs[0] = new BidirDShotX1(DSHOT0_PIN, 600, DSHOT_PIO);
    ESCs[1] = new BidirDShotX1(DSHOT1_PIN, 600, DSHOT_PIO);
    ESCs[2] = new BidirDShotX1(DSHOT2_PIN, 600, DSHOT_PIO);
    ESCs[3] = new BidirDShotX1(DSHOT3_PIN, 600, DSHOT_PIO);

    stdio_usb_init();
    minihdlc_init(nullptr, frame_received);

    led_init();
    printf("hello,world!\n");
    while (1)
    {
        printf("wake\n");
        sleep_ms(1000);
    }
    // tight_loop_contents();
}