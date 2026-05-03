/*
 * Simple LED blink test for nRF54L15 DK
 * Tests if the board is running ANY code at all
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

/* LED0 is typically on P1.13 for nRF54L15 DK */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	int ret;

	/* Check if LED device is ready */
	if (!gpio_is_ready_dt(&led)) {
		return -1;
	}

	/* Configure LED pin as output */
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}

	/* Blink forever */
	while (1) {
		gpio_pin_toggle_dt(&led);
		k_msleep(500);
	}

	return 0;
}
