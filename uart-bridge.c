// SPDX-License-Identifier: MIT
/*
 * Copyright 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 */

#include <hardware/irq.h>
#include <hardware/structs/sio.h>
#include <hardware/uart.h>
#include <hardware/structs/pio.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <string.h>
#include <tusb.h>
#include <uart_rx.pio.h>
#include <uart_tx.pio.h>

#if !defined(MIN)
#define MIN(a, b) ((a > b) ? b : a)
#endif /* MIN */

#define LED_PIN 25

// might as well use our RAM
#define BUFFER_SIZE 5120

#define DEF_BIT_RATE 9600
#define DEF_STOP_BITS 1
#define DEF_PARITY 0
#define DEF_DATA_BITS 8

typedef struct {
	uart_inst_t *const inst;
	uint8_t tx_pin;
	uint8_t rx_pin;
    uint    sm;

} uart_id_t;

typedef struct {
	cdc_line_coding_t usb_lc;
	cdc_line_coding_t uart_lc;
	mutex_t lc_mtx;
	uint8_t uart_buffer[BUFFER_SIZE];
	uint32_t uart_pos;
	mutex_t uart_mtx;
	uint8_t usb_buffer[BUFFER_SIZE];
	uint32_t usb_pos;
	uint32_t usb_snd;
	mutex_t usb_mtx;
} uart_data_t;

uart_id_t UART_ID[CFG_TUD_CDC] = {
	{
		.inst = uart0,
		.tx_pin = 0,
		.rx_pin = 1,
	},{
		.inst = uart1,
		.tx_pin = 4,
		.rx_pin = 5,
	},{
		.inst = 0,
		.tx_pin = 8,
		.rx_pin = 9,
        .sm = 0,
	},{
		.inst = 0,
		.tx_pin = 12,
		.rx_pin = 13,
        .sm = 1,
	},{
		.inst = 0,
		.tx_pin = 16,
		.rx_pin = 17,
        .sm = 2,
	}
};

uart_data_t UART_DATA[CFG_TUD_CDC];


uint rx_offset=0;
uint rxp_offset=0;

uint tx_offset=0;
uint txp_offset=0;


static inline uint databits_usb2uart(uint8_t data_bits)
{
	switch (data_bits) {
		case 5:
			return 5;
		case 6:
			return 6;
		case 7:
			return 7;
		default:
			return 8;
	}
}

static inline uart_parity_t parity_usb2uart(uint8_t usb_parity)
{
	switch (usb_parity) {
		case 1:
			return UART_PARITY_ODD;
		case 2:
			return UART_PARITY_EVEN;
		default:
			return UART_PARITY_NONE;
	}
}

static inline uint stopbits_usb2uart(uint8_t stop_bits)
{
	switch (stop_bits) {
		case 2:
			return 2;
		default:
			return 1;
	}
}

void update_uart_cfg(uint8_t itf)
{
	uart_id_t *ui = &UART_ID[itf];
	uart_data_t *ud = &UART_DATA[itf];

	mutex_enter_blocking(&ud->lc_mtx);

    if (ui->inst != 0) { //regular uart
	    if (ud->usb_lc.bit_rate != ud->uart_lc.bit_rate) {
			uart_set_baudrate(ui->inst, ud->usb_lc.bit_rate);
			ud->uart_lc.bit_rate = ud->usb_lc.bit_rate;
	    }

	    if ((ud->usb_lc.stop_bits != ud->uart_lc.stop_bits) ||
	        (ud->usb_lc.parity != ud->uart_lc.parity) ||
	        (ud->usb_lc.data_bits != ud->uart_lc.data_bits)) {
		    uart_set_format(ui->inst,
				databits_usb2uart(ud->usb_lc.data_bits),
				stopbits_usb2uart(ud->usb_lc.stop_bits),
				parity_usb2uart(ud->usb_lc.parity));
		    ud->uart_lc.data_bits = ud->usb_lc.data_bits;
		    ud->uart_lc.parity = ud->usb_lc.parity;
			ud->uart_lc.stop_bits = ud->usb_lc.stop_bits;
	    }
    } else {
	    if (ud->usb_lc.bit_rate != ud->uart_lc.bit_rate) {
            uart_baud(pio0,ui->sm,ud->usb_lc.bit_rate);
            uart_baud(pio1,ui->sm,ud->usb_lc.bit_rate);
			ud->uart_lc.bit_rate = ud->usb_lc.bit_rate;
	    }
		if (ud->usb_lc.parity != ud->uart_lc.parity) {
			ud->uart_lc.parity = ud->usb_lc.parity;
			if (ud->usb_lc.parity == UART_PARITY_NONE) {
				uart_rx_program_init(pio0, ui->sm, rx_offset, ui->rx_pin, ud->uart_lc.bit_rate);
				uart_tx_program_init(pio1, ui->sm, tx_offset, ui->tx_pin, ud->uart_lc.bit_rate);
			} else {
				uart_rx_program_init(pio0, ui->sm, rxp_offset, ui->rx_pin, ud->uart_lc.bit_rate);
        		uart_tx_program_init(pio1, ui->sm, txp_offset, ui->tx_pin, ud->uart_lc.bit_rate);
			}
		}
    }

	mutex_exit(&ud->lc_mtx);
}

void usb_read_bytes(uint8_t itf) {
	uint32_t len = tud_cdc_n_available(itf);

	if (len) {
		uart_data_t *ud = &UART_DATA[itf];

		mutex_enter_blocking(&ud->usb_mtx);

		len = MIN(len, BUFFER_SIZE - ud->usb_pos);
		if (len) {
			uint32_t count;

			count = tud_cdc_n_read(itf, &ud->usb_buffer[ud->usb_pos], len);
			ud->usb_pos += count;
		}

		mutex_exit(&ud->usb_mtx);
	}
}

void usb_write_bytes(uint8_t itf) {
	uart_data_t *ud = &UART_DATA[itf];

	if (ud->uart_pos) {
		uint32_t count;

		mutex_enter_blocking(&ud->uart_mtx);

		count = tud_cdc_n_write(itf, ud->uart_buffer, ud->uart_pos);
		if (count < ud->uart_pos)
			memcpy(ud->uart_buffer, &ud->uart_buffer[count],
			       ud->uart_pos - count);
		ud->uart_pos -= count;

		mutex_exit(&ud->uart_mtx);

		if (count)
			tud_cdc_n_write_flush(itf);
	}
}

void usb_cdc_process(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];

	mutex_enter_blocking(&ud->lc_mtx);
	tud_cdc_n_get_line_coding(itf, &ud->usb_lc);
	mutex_exit(&ud->lc_mtx);

	usb_read_bytes(itf);
	usb_write_bytes(itf);
}

void core1_entry(void)
{
	tusb_init();

	while (1) {
		int itf;
		int con = 0;

		tud_task();
	
		for (itf = 0; itf < CFG_TUD_CDC; itf++) {
			if (tud_cdc_n_connected(itf)) {
				con = 1;
				usb_cdc_process(itf);
			}
		}

		gpio_put(LED_PIN, con);
	}
}

void uart_read_bytes(uint8_t itf) 
{
	const uart_id_t *ui = &UART_ID[itf];

    if (ui->inst != 0) {
	    if (uart_is_readable(ui->inst)) {
			uart_data_t *ud = &UART_DATA[itf];

			mutex_enter_blocking(&ud->uart_mtx);

			while (uart_is_readable(ui->inst) &&
					ud->uart_pos < BUFFER_SIZE) {
				ud->uart_buffer[ud->uart_pos] = uart_getc(ui->inst);
				ud->uart_pos++;
			}

			mutex_exit(&ud->uart_mtx);
	    }
    } else {
        if (!pio_sm_is_rx_fifo_empty(pio0, ui->sm)) {
            uart_data_t *ud = &UART_DATA[itf];
            mutex_enter_blocking(&ud->uart_mtx);
            while (!pio_sm_is_rx_fifo_empty(pio0, ui->sm) &&
                    ud->uart_pos < BUFFER_SIZE) {
                ud->uart_buffer[ud->uart_pos] =  uart_rx_program_getc(pio0, ui->sm);
                ud->uart_pos++;
            }

            mutex_exit(&ud->uart_mtx);
        }
    }      
}

void uart_write_bytes(uint8_t itf) {
	uart_data_t *ud = &UART_DATA[itf];

	if ((ud->usb_pos) && (ud->usb_snd < ud->usb_pos)) {
	    const uart_id_t *ui = &UART_ID[itf];

		mutex_enter_blocking(&ud->usb_mtx);
        if (ui->inst != 0){	

            while (uart_is_writable(ui->inst)&&(ud->usb_snd < ud->usb_pos)) {
                uart_putc(ui->inst, ud->usb_buffer[ud->usb_snd++]);
            }			
        } else {

            size_t bufspace=7-pio_sm_get_tx_fifo_level(pio1,ui->sm);	
            size_t tosend=ud->usb_pos-ud->usb_snd;	
            tosend = MIN(tosend,bufspace);

            for (size_t i = 0; i<tosend; ++i) {
        		uart_tx_program_putc(pio1, ui->sm, ud->usb_buffer[ud->usb_snd+i],ud->usb_lc.parity);
            }
            ud->usb_snd+=tosend;
        }
		// only reset buffers if we've sent everything
        if (ud->usb_snd == ud->usb_pos) {
            ud->usb_pos = 0;
            ud->usb_snd = 0;
        }
		mutex_exit(&ud->usb_mtx);
	}
}

void init_uart_data(uint8_t itf) {
	uart_id_t *ui = &UART_ID[itf];
	uart_data_t *ud = &UART_DATA[itf];

    if (ui->inst != 0) {
		/* Pinmux */
		gpio_set_function(ui->tx_pin, GPIO_FUNC_UART);
		gpio_set_function(ui->rx_pin, GPIO_FUNC_UART);
    }

	/* USB CDC LC */
	ud->usb_lc.bit_rate = DEF_BIT_RATE;
	ud->usb_lc.data_bits = DEF_DATA_BITS;
	ud->usb_lc.parity = DEF_PARITY;
	ud->usb_lc.stop_bits = DEF_STOP_BITS;

	/* UART LC */
	ud->uart_lc.bit_rate = DEF_BIT_RATE;
	ud->uart_lc.data_bits = DEF_DATA_BITS;
	ud->uart_lc.parity = DEF_PARITY;
	ud->uart_lc.stop_bits = DEF_STOP_BITS;

	/* Buffer */
	ud->uart_pos = 0;
	ud->usb_pos = 0;
	ud->usb_snd = 0;

	/* Mutex */
	mutex_init(&ud->lc_mtx);
	mutex_init(&ud->uart_mtx);
	mutex_init(&ud->usb_mtx);

    if (ui->inst != 0){
		/* UART start */
		uart_init(ui->inst, ud->usb_lc.bit_rate);
		uart_set_hw_flow(ui->inst, false, false);
		uart_set_format(ui->inst, databits_usb2uart(ud->usb_lc.data_bits),
		stopbits_usb2uart(ud->usb_lc.stop_bits),
		parity_usb2uart(ud->usb_lc.parity));
    } else {
        // Set up the state machine we're going to use to for rx/tx       
        uart_rx_program_init(pio0, ui->sm, rx_offset, ui->rx_pin, ud->uart_lc.bit_rate);
        uart_tx_program_init(pio1, ui->sm, tx_offset, ui->tx_pin, ud->uart_lc.bit_rate);
    }
}

int main(void)
{
	int itf;

	// store our PIO programs in tbe instruction registers
	// we'll use pio0 for RX and pio1 for tx so only one copy of each is needed
	// however we'll use a different program to send/receive with parity
	rx_offset = pio_add_program(pio0, &uart_rx_program);
	tx_offset = pio_add_program(pio1, &uart_tx_program);
	rxp_offset = pio_add_program(pio0, &uart_rxp_program);
	txp_offset = pio_add_program(pio1, &uart_txp_program);



	for (itf = 0; itf < CFG_TUD_CDC; itf++)
		init_uart_data(itf);

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	multicore_launch_core1(core1_entry);

	while (1) {
		for (itf = 0; itf < CFG_TUD_CDC; itf++) {
			update_uart_cfg(itf);
			uart_read_bytes(itf);
			uart_write_bytes(itf);
		}
	}

	return 0;
}
