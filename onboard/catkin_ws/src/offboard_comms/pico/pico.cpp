#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    const uint led_pin = PICO_DEFAULT_LED_PIN;

    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    while (1) {
        gpio_put(led_pin, 1);
        sleep_ms(500);
        gpio_put(led_pin, 0);
        sleep_ms(500);
    }

    return 0;
}
