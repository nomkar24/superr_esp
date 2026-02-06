#include "ble_midi.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h" // Required for microsecond timing
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include <stdio.h>

static const char *TAG = "velocity_key";

// --- LED Strip Config ---
#define LED_STRIP_GPIO_PIN 10
#define LED_STRIP_LED_NUMBERS 24
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

// --- Velocity Key Config ---
// Column Drives (Output)
// Row 1: First Switch (Top) -> Starts Timer
// Row A: Second Switch (Bottom) -> Triggers Note with Velocity

#define COL_GPIO 4
// SWAPPED based on logs indicating S2 hit before S1
#define ROW_1_GPIO 2 // First contact (Matrix 1) - Was GPIO 1
#define ROW_A_GPIO 1 // Second contact (Matrix 2) - Was GPIO 2

// Velocity Calculation Config
#define MIN_TIME_US 2000   // 2ms (Fastest hit -> Max Velocity 127)
#define MAX_TIME_US 200000 // 200ms (Slowest hit -> Min Velocity 1)

void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize BLE MIDI
  ble_midi_init();

  // LED strip initialization
  led_strip_config_t strip_config = {
      .strip_gpio_num = LED_STRIP_GPIO_PIN,
      .max_leds = LED_STRIP_LED_NUMBERS,
      .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
      .led_model = LED_MODEL_WS2812,
      .flags.invert_out = false,
  };
  led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
      .rmt_channel = 0,
#else
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = LED_STRIP_RMT_RES_HZ,
      .flags.with_dma = false,
#endif
  };
  led_strip_handle_t led_strip;
  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  ESP_LOGI(TAG, "LED Strip initialized on GPIO %d", LED_STRIP_GPIO_PIN);

  // --- GPIO Configuration ---

  // 1. Column (Output)
  gpio_config_t col_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << COL_GPIO),
      .pull_down_en = 0,
      .pull_up_en = 0,
  };
  gpio_config(&col_conf);
  gpio_set_level(COL_GPIO, 1); // Start Inactive (High)

  // 2. Rows (Inputs with Pull-up)
  gpio_config_t row_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = (1ULL << ROW_1_GPIO) | (1ULL << ROW_A_GPIO),
      .pull_down_en = 0,
      .pull_up_en = 1,
  };
  gpio_config(&row_conf);

  ESP_LOGI(TAG, "Velocity Key Configured (SWAPPED): Col %d, Row1 %d, RowA %d",
           COL_GPIO, ROW_1_GPIO, ROW_A_GPIO);

  // State Variables
  bool key_pressed = false;
  int64_t start_time = 0;
  bool timer_running = false;

  while (1) {
    // 1. Activate Column (Drive LOW)
    gpio_set_level(COL_GPIO, 0);
    // esp_rom_delay_us(5); // Settle

    // 2. Read Sensors (Active Low)
    bool s1_active = (gpio_get_level(ROW_1_GPIO) == 0);
    bool sa_active = (gpio_get_level(ROW_A_GPIO) == 0);

    // --- Logic ---

    // Check for S1 Activation (Start Timer)
    if (s1_active && !timer_running && !key_pressed) {
      start_time = esp_timer_get_time();
      timer_running = true;
      // ESP_LOGI(TAG, "S1 Contact (Timer Start)");
    }

    // Check for SA Activation (Trigger Note)
    if (sa_active && timer_running && !key_pressed) {
      int64_t end_time = esp_timer_get_time();
      int64_t delta_us = end_time - start_time;

      timer_running = false;
      key_pressed = true;

      // Velocity Mapping
      int velocity = 1;
      if (delta_us <= MIN_TIME_US) {
        velocity = 127;
      } else if (delta_us >= MAX_TIME_US) {
        velocity = 1;
      } else {
        velocity = 127 - ((delta_us - MIN_TIME_US) * 126 /
                          (MAX_TIME_US - MIN_TIME_US));
      }

      ESP_LOGI(TAG, "Note ON! Delta: %lld us -> Velocity: %d", delta_us,
               velocity);

      // LED Feedback
      led_strip_set_pixel(led_strip, 0, velocity * 2, 0, 0);
      led_strip_refresh(led_strip);

      uint8_t note_on[] = {0x80, 0x80, 0x90, 60, (uint8_t)velocity};
      ble_midi_send_packet(note_on, sizeof(note_on));
    }

    // Fallback: If SA hits but we missed S1 (Timer not running)
    // This might happen on very fast presses or bounces.
    if (sa_active && !timer_running && !key_pressed) {
      key_pressed = true;
      ESP_LOGW(TAG, "SA hit without S1! Forcing Note On");

      // If we missed S1 timestamps, we assume Max Velocity (Fastest time)
      uint8_t note_on[] = {0x80, 0x80, 0x90, 60, 127};
      ble_midi_send_packet(note_on, sizeof(note_on));

      led_strip_set_pixel(led_strip, 0, 200, 0, 0);
      led_strip_refresh(led_strip);
    }

    // Check for Release (Wait for BOTH to open to prevent bounce-spam)
    // Was: if (!s1_active && key_pressed)
    if (!s1_active && !sa_active && key_pressed) {
      key_pressed = false;
      ESP_LOGI(TAG, "Note UP");

      led_strip_clear(led_strip);

      uint8_t note_off[] = {0x80, 0x80, 0x80, 60, 0};
      ble_midi_send_packet(note_off, sizeof(note_off));
    }

    // Reset timer if S1 released before SA hit (Aborted press)
    if (!s1_active && timer_running) {
      timer_running = false;
    }

    // 3. Deactivate Column (Drive HIGH)
    gpio_set_level(COL_GPIO, 1);

    // Delay (1ms is maintained by SDKConfig fix)
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
