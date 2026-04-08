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
#include <zephyr/drivers/gpio.h>

/* Change this if you have a different UART device node */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)

/* Solenoid definition from Device Tree alias */
#define SOLENOID_NODE DT_ALIAS(solenoid)
static const struct gpio_dt_spec solenoid = GPIO_DT_SPEC_GET(SOLENOID_NODE, gpios);

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
			
			rx_buf[rx_buf_pos] = '\0';

			
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			
			rx_buf_pos = 0;
		} else if (c != '\n' && c != '\r') {
			rx_buf[rx_buf_pos++] = c;
			if (rx_buf_pos >= (MSG_SIZE - 1)) {
				
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

	/* Test the solenoid: turn ON if velocity > 0, OFF if velocity == 0 */
	if (vel > 0) {
		gpio_pin_set_dt(&solenoid, 1);
		printk("Solenoid ON (LED0 turned ON)\n");
	} else {
		gpio_pin_set_dt(&solenoid, 0);
		printk("Solenoid OFF (LED0 turned OFF)\n");
	}
}

int main(void)
{
	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!\n");
		/* Don't return, let's keep going to test the solenoid/LED */
	}

	/* Initialize the solenoid GPIO */
	if (!gpio_is_ready_dt(&solenoid)) {
		printk("Error: Solenoid device not ready\n");
		return 0;
	}
	gpio_pin_configure_dt(&solenoid, GPIO_OUTPUT_INACTIVE);

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
		/* Continue anyway so we can test the LED/Solenoid loop! */
	} else {
		uart_irq_rx_enable(uart_dev);
	}

	printk("KLAVIATOR Hardware Test Mode.\n");
	printk("Tester Solenoid på Pin P0.13 automatisk...\n");

	while (1) {
		/* Turn Solenoid ON */
		gpio_pin_set_dt(&solenoid, 1);
		printk("Solenoid ON (P0.13 HIGH)\n");
		k_msleep(1000); /* Wait 1 second */

		/* Turn Solenoid OFF */
		gpio_pin_set_dt(&solenoid, 0);
		printk("Solenoid OFF (P0.13 LOW)\n");
		k_msleep(1000); /* Wait 1 second */
	}

	return 0;
}
