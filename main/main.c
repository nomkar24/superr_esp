#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"

static const char *TAG = "example";

// GPIO assignment
// Note: ESP32-S3 DevKitC-1 often has an RGB LED on GPIO 48.
// We are using GPIO 10 for an external strip as per plan.
#define LED_STRIP_GPIO_PIN 10
// Numbers of the LED in the strip
#define LED_STRIP_LED_NUMBERS 24
// 10MHz resolution, 1 tick = 0.1us - compatible with RMT backend
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

void app_main(void)
{
    // LED strip general initialization
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN,
        .max_leds = LED_STRIP_LED_NUMBERS,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // WS2812B is usually GRB
        .led_model = LED_MODEL_WS2812, // This supports WS2812B
        .flags.invert_out = false, // standard WS2812
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        #if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
            .rmt_channel = 0,
        #else
            .clk_src = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
            .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
            .flags.with_dma = false, // DMA is available on ESP32-S3, but not explicitly needed for short strips
        #endif
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "LED Strip initialized on GPIO %d", LED_STRIP_GPIO_PIN);

    ESP_LOGI(TAG, "Start cycling colors");
    while (1) {
        // Red
        for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 50, 0, 0)); // R, G, B (50 is brightness)
        }
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Green
        for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 50, 0));
        }
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Blue
        for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 50));
        }
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Rainbow effect
        for (int j = 0; j < 255; j += 5) {
            for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                 // Simple hue calculation attempt (just shifting colors)
                 // For a proper rainbow we'd need HSV to RGB, but let's keep it simple
                 uint32_t red = (i * 10 + j) % 255;
                 uint32_t green = (i * 10 + j + 85) % 255;
                 uint32_t blue = (i * 10 + j + 170) % 255;
                 // Dimming it down for safety
                 led_strip_set_pixel(led_strip, i, red/5, green/5, blue/5);
            }
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        
        // Clear
        led_strip_clear(led_strip);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
