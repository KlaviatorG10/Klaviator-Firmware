/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

/* Change this if you have a different UART device node */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)

#define MSG_SIZE 64

/* Queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* Receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (c != '\n' && c != '\r') {
			rx_buf[rx_buf_pos++] = c;
			if (rx_buf_pos >= (MSG_SIZE - 1)) {
				/* Drop data if buffer is full */
				rx_buf_pos = 0;
			}
		}
	}
}

void parse_message(char *msg)
{
	char state[10] = {0};
	int note = 0;
	int vel = 0;

	/* Format expected: STATE:NOTE:VELOCITY */
	char *token = strtok(msg, ":");
	if (token == NULL) return;
	strncpy(state, token, sizeof(state) - 1);

	token = strtok(NULL, ":");
	if (token == NULL) return;
	note = atoi(token);

	token = strtok(NULL, ":");
	if (token == NULL) return;
	vel = atoi(token);

	printk("Mottatt - State: %s, Note: %d, Vel: %d\n", state, note, vel);
}

int main(void)
{
	char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(uart_dev);

	printk("KLAVIATOR UART Nervesystem Klar.\n");
	printk("Venter på data: STATE:NOTE:VELOCITY\\n\n");

	while (1) {
		/* get a message from the queue */
		if (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
			parse_message(tx_buf);
		}
	}

	return 0;
}
