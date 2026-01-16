#include "led.h"

/* Use the logging module registered in main.c */
LOG_MODULE_DECLARE(esb_prx);

int leds_init(void)
{
	if (!device_is_ready(leds[0].port)) {
		LOG_ERR("LEDs port not ready");
		return -ENODEV;
	}

	for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
		int err = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT);

		if (err) {
			LOG_ERR("Unable to configure LED%u, err %d.", i, err);
			return err;
		}
	}

	return 0;
}

void leds_update(uint8_t value)
{
	bool led0_status = !(value % 8 > 0 && value % 8 <= 4);
	bool led1_status = !(value % 8 > 1 && value % 8 <= 5);
	bool led2_status = !(value % 8 > 2 && value % 8 <= 6);
	bool led3_status = !(value % 8 > 3);

	gpio_port_pins_t mask = BIT(leds[0].pin) | BIT(leds[1].pin) |
				BIT(leds[2].pin) | BIT(leds[3].pin);

	gpio_port_value_t val = led0_status << leds[0].pin |
				led1_status << leds[1].pin |
				led2_status << leds[2].pin |
				led3_status << leds[3].pin;

	gpio_port_set_masked_raw(leds[0].port, mask, val);
}