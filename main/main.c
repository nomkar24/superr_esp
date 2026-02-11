/*
 * DUAL-MATRIX VELOCITY-SENSITIVE MIDI KEYBOARD
 *
 * Hardware:
 * - 24 keys (6 rows x 4 cols)
 * - 2 matrices: S1 (First Contact), SA (Second Contact)
 * - Shared columns, separate rows for each matrix.
 *
 * Logic:
 * - Velocity derived from time difference between S1 and SA contacts.
 */

#include "ble_midi.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include <stdio.h>

static const char *TAG = "velocity_matrix";

// --- LED Strip Config ---
#define LED_STRIP_GPIO_PIN 10
#define LED_STRIP_LED_NUMBERS 24
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

// --- Matrix Configuration ---
#define NUM_MATRICES 2
#define COLS_PER_MATRIX 4
#define ROWS_PER_MATRIX 6
#define NUM_KEYS 24

// Shared Columns (INPUT with PULL-UP)
#define COL1_GPIO 4
#define COL2_GPIO 5
#define COL3_GPIO 6
#define COL4_GPIO 7

// Matrix 1 Rows (S1 - First Contact) (OUTPUT)
#define M1_ROW1_GPIO 16
#define M1_ROW2_GPIO 17
#define M1_ROW3_GPIO 18
#define M1_ROW4_GPIO 8
#define M1_ROW5_GPIO 3
#define M1_ROW6_GPIO 46

// Matrix 2 Rows (SA - Second Contact) (OUTPUT)
#define M2_ROW1_GPIO 35
#define M2_ROW2_GPIO 36
#define M2_ROW3_GPIO 37
#define M2_ROW4_GPIO 38
#define M2_ROW5_GPIO 39
#define M2_ROW6_GPIO 41

// Velocity Calculation Config
#define MIN_TIME_US 2000   // 2ms (Max Velocity 127)
#define MAX_TIME_US 200000 // 200ms (Min Velocity 1)

typedef struct {
  int64_t start_time;
  bool timer_running;
  bool key_pressed;
  uint8_t velocity;
  uint8_t midi_note;
  uint8_t debounce_count; // Debounce counter
} KeyState;

typedef struct {
  gpio_num_t col_gpios[COLS_PER_MATRIX];
  gpio_num_t row_gpios[ROWS_PER_MATRIX];
} MatrixConfig;

static KeyState keys[NUM_KEYS];
static led_strip_handle_t led_strip;

static const gpio_num_t shared_cols[COLS_PER_MATRIX] = {COL1_GPIO, COL2_GPIO,
                                                        COL3_GPIO, COL4_GPIO};

static const MatrixConfig matrices[NUM_MATRICES] = {
    {
        // Matrix 1 (S1)
        .col_gpios = {COL1_GPIO, COL2_GPIO, COL3_GPIO, COL4_GPIO},
        .row_gpios = {M1_ROW1_GPIO, M1_ROW2_GPIO, M1_ROW3_GPIO, M1_ROW4_GPIO,
                      M1_ROW5_GPIO, M1_ROW6_GPIO},
    },
    {
        // Matrix 2 (SA)
        .col_gpios = {COL1_GPIO, COL2_GPIO, COL3_GPIO, COL4_GPIO},
        .row_gpios = {M2_ROW1_GPIO, M2_ROW2_GPIO, M2_ROW3_GPIO, M2_ROW4_GPIO,
                      M2_ROW5_GPIO, M2_ROW6_GPIO},
    },
};

static int calculate_velocity(int64_t delta_us) {
  if (delta_us <= MIN_TIME_US) {
    return 127;
  } else if (delta_us >= MAX_TIME_US) {
    return 1;
  } else {
    return 127 - ((delta_us - MIN_TIME_US) * 126 / (MAX_TIME_US - MIN_TIME_US));
  }
}

static void process_key_logic(int key_index, bool s1_active, bool sa_active) {
  KeyState *key = &keys[key_index];

  // 1. S1 Activation (Start Timer)
  if (s1_active && !key->timer_running && !key->key_pressed) {
    key->start_time = esp_timer_get_time();
    key->timer_running = true;
    ESP_LOGI(TAG, "Key %d: S1 DETECTED - Timer Started", key_index);
  }

  // 2. SA Activation (Trigger Note)
  if (sa_active && key->timer_running && !key->key_pressed) {
    int64_t end_time = esp_timer_get_time();
    int64_t delta_us = end_time - key->start_time;

    key->timer_running = false;
    key->key_pressed = true;
    key->velocity = calculate_velocity(delta_us);

    ESP_LOGI(TAG, "Key %d: SA DETECTED - Velocity %d (Delta: %lld us)",
             key_index, key->velocity, delta_us);

    if (key_index < LED_STRIP_LED_NUMBERS) {
      led_strip_set_pixel(led_strip, key_index, key->velocity * 2, 0, 0);
      led_strip_refresh(led_strip);
    }

    // BLE MIDI packet format (per specification):
    // [Header] [Timestamp Low] [Timestamp High] [MIDI Status] [MIDI Data...]
    uint16_t timestamp =
        (esp_timer_get_time() / 1000) & 0x1FFF; // 13-bit timestamp
    uint8_t note_on[] = {
        0x80,                               // Header byte (timestamp valid)
        (uint8_t)(timestamp & 0x7F),        // Timestamp low 7 bits
        (uint8_t)((timestamp >> 7) & 0x3F), // Timestamp high 6 bits
        0x90,                               // Note On, Channel 1
        key->midi_note,                     // Note number
        key->velocity                       // Velocity
    };
    ble_midi_send_packet(note_on, sizeof(note_on));
  }

  // 3. Fallback - SA without S1
  if (sa_active && !key->timer_running && !key->key_pressed) {
    key->key_pressed = true;
    key->velocity = 127;
    ESP_LOGW(TAG, "Key %d: SA WITHOUT S1 (Fallback - Max Velocity)", key_index);

    uint16_t timestamp = (esp_timer_get_time() / 1000) & 0x1FFF;
    uint8_t note_on[] = {0x80,
                         (uint8_t)(timestamp & 0x7F),
                         (uint8_t)((timestamp >> 7) & 0x3F),
                         0x90,
                         key->midi_note,
                         127};
    ble_midi_send_packet(note_on, sizeof(note_on));

    if (key_index < LED_STRIP_LED_NUMBERS) {
      led_strip_set_pixel(led_strip, key_index, 200, 0, 0);
      led_strip_refresh(led_strip);
    }
  }

  // 4. Release (Note OFF)
  if (!s1_active && !sa_active && key->key_pressed) {
    key->key_pressed = false;
    ESP_LOGI(TAG, "Key %d: RELEASED", key_index);

    if (key_index < LED_STRIP_LED_NUMBERS) {
      led_strip_set_pixel(led_strip, key_index, 0, 0, 0);
      led_strip_refresh(led_strip);
    }

    uint16_t timestamp = (esp_timer_get_time() / 1000) & 0x1FFF;
    uint8_t note_off[] = {0x80,
                          (uint8_t)(timestamp & 0x7F),
                          (uint8_t)((timestamp >> 7) & 0x3F),
                          0x80, // Note Off, Channel 1
                          key->midi_note,
                          0};
    ble_midi_send_packet(note_off, sizeof(note_off));
  }

  // 5. Aborted Press
  if (!s1_active && key->timer_running) {
    key->timer_running = false;
    ESP_LOGW(TAG, "Key %d: S1 RELEASED before SA (Aborted)", key_index);
  }
}

void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ble_midi_init();

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
  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  ESP_LOGI(TAG, "LED Strip initialized on GPIO %d", LED_STRIP_GPIO_PIN);

  // Initialize key states with standard piano layout (C4 to B5)
  // Keys are arranged in 6 rows × 4 columns, mapped left-to-right,
  // bottom-to-top
  const uint8_t piano_notes[NUM_KEYS] = {
      60, 61, 62, 63, // Row 0: C4, C#4, D4, D#4
      64, 65, 66, 67, // Row 1: E4, F4, F#4, G4
      68, 69, 70, 71, // Row 2: G#4, A4, A#4, B4
      72, 73, 74, 75, // Row 3: C5, C#5, D5, D#5
      76, 77, 78, 79, // Row 4: E5, F5, F#5, G5
      80, 81, 82, 83  // Row 5: G#5, A5, A#5, B5
  };

  for (int i = 0; i < NUM_KEYS; i++) {
    keys[i].start_time = 0;
    keys[i].timer_running = false;
    keys[i].key_pressed = false;
    keys[i].velocity = 0;
    keys[i].midi_note = piano_notes[i];
    keys[i].debounce_count = 0;
  }

  uint64_t col_pin_mask = 0;
  uint64_t row_pin_mask = 0;

  for (int c = 0; c < COLS_PER_MATRIX; c++) {
    col_pin_mask |= (1ULL << shared_cols[c]);
  }

  for (int m = 0; m < NUM_MATRICES; m++) {
    for (int r = 0; r < ROWS_PER_MATRIX; r++) {
      row_pin_mask |= (1ULL << matrices[m].row_gpios[r]);
    }
  }

  gpio_config_t col_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = col_pin_mask,
      .pull_down_en = 0,
      .pull_up_en = 0,
  };
  gpio_config(&col_conf);

  // Set all columns LOW (inactive for Active-High)
  for (int c = 0; c < COLS_PER_MATRIX; c++) {
    gpio_set_level(shared_cols[c], 0);
  }

  // Rows with PULL-DOWN for Active-High scanning
  gpio_config_t row_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = row_pin_mask,
      .pull_down_en = 1,
      .pull_up_en = 0,
  };
  gpio_config(&row_conf);

  ESP_LOGI(TAG, "24-Key Velocity Matrix Initialized");

  while (1) {
    for (int c = 0; c < COLS_PER_MATRIX; c++) {
      // Active-High: Drive column HIGH to scan
      gpio_set_level(shared_cols[c], 1);
      esp_rom_delay_us(10);

      for (int m = 0; m < NUM_MATRICES; m++) {
        const MatrixConfig *matrix = &matrices[m];

        for (int r = 0; r < ROWS_PER_MATRIX; r++) {
          int key_index = (r * COLS_PER_MATRIX) + c;
          int level = gpio_get_level(matrix->row_gpios[r]);

          // Active-High: Row reads HIGH when pressed
          bool contact_active = (level == 1);
          bool s1_active = (m == 0) ? contact_active : false;
          bool sa_active = (m == 1) ? contact_active : false;

          process_key_logic(key_index, s1_active, sa_active);
        }
      }
      // Deactivate column (drive LOW)
      gpio_set_level(shared_cols[c], 0);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
