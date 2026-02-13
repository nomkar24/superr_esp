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

#include "ble_config_service.h"
#include "ble_midi_service.h"
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
#define COL4_GPIO 15

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
#define M2_ROW6_GPIO 45

// Velocity Calculation Config
#define MIN_TIME_US 2000           // 2ms (Max Velocity 127)
#define MAX_TIME_US 200000         // 200ms (Min Velocity 1)
#define VELOCITY_TIMEOUT_US 500000 // 500ms timeout for stuck timers

// Debounce Configuration
// Debounce Configuration
#define DEBOUNCE_TIME_US 5000 // 5ms lockout
#define SCAN_INTERVAL_US 50   // Target 20kHz scan rate

// LED Configuration
#define LED_VELOCITY_SCALE 2
#define LED_MAX_BRIGHTNESS 200
#define LED_OFF 0

typedef struct {
  int64_t start_time;
  bool timer_running;
  bool key_pressed;
  uint8_t velocity;
  uint8_t midi_note;
  // Debounce State
  bool s1_stable;
  bool sa_stable;
  int64_t s1_last_change;
  int64_t sa_last_change;
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

/**
 * @brief Send a MIDI note message over BLE using helper functions
 *
 * @param note MIDI note number (0-127)
 * @param velocity Note velocity (0-127, 0 = note off)
 * @param note_on true for Note On, false for Note Off
 * @return int 0 on success, negative on error
 */
static int send_midi_note(uint8_t note, uint8_t velocity, bool note_on) {
  uint8_t midi_packet[5];
  int len;

  if (note_on) {
    len = ble_midi_note_on(note, velocity, 0, midi_packet, sizeof(midi_packet));
  } else {
    len =
        ble_midi_note_off(note, velocity, 0, midi_packet, sizeof(midi_packet));
  }

  if (len < 0) {
    return len; // Error from helper function
  }

  return ble_midi_send(midi_packet, len);
}

static const char *get_note_name(uint8_t note) {
  static const char *note_names[] = {"C",  "C#", "D",  "D#", "E",  "F",
                                     "F#", "G",  "G#", "A",  "A#", "B"};
  return note_names[note % 12];
}

static void process_key_logic(int key_index, bool s1_raw, bool sa_raw) {
  KeyState *key = &keys[key_index];
  int64_t now = esp_timer_get_time();

  // --- S1 Debounce (Eager) ---
  if (s1_raw != key->s1_stable) {
    if ((now - key->s1_last_change) > DEBOUNCE_TIME_US) {
      key->s1_stable = s1_raw;
      key->s1_last_change = now;

      // S1 Rising Edge
      if (key->s1_stable && !key->timer_running && !key->key_pressed) {
        key->start_time = now;
        key->timer_running = true;
        // Log S1 Contact
        ESP_LOGI(TAG, "Key %d (%s%d): S1 (First Contact) detected", key_index,
                 get_note_name(key->midi_note), (key->midi_note / 12) - 1);
      }
      // S1 Falling Edge (early release)
      else if (!key->s1_stable && key->timer_running) {
        key->timer_running = false;
        ESP_LOGW(TAG, "Key %d (%s%d): S1 Released before SA (Aborted)",
                 key_index, get_note_name(key->midi_note),
                 (key->midi_note / 12) - 1);
      }
    }
  }

  // --- SA Debounce (Eager) ---
  if (sa_raw != key->sa_stable) {
    if ((now - key->sa_last_change) > DEBOUNCE_TIME_US) {
      key->sa_stable = sa_raw;
      key->sa_last_change = now;

      // SA Rising Edge
      if (key->sa_stable) {
        // 1. Normal Strike (S1 -> SA)
        if (key->timer_running && !key->key_pressed) {
          int64_t delta_us = now - key->start_time;
          key->timer_running = false;
          key->key_pressed = true;
          key->velocity = calculate_velocity(delta_us);

          // Log SA Contact and Velocity
          ESP_LOGI(TAG,
                   "Key %d (%s%d): SA (Second Contact) detected | Delta: %lld "
                   "us | Vel: %d",
                   key_index, get_note_name(key->midi_note),
                   (key->midi_note / 12) - 1, delta_us, key->velocity);

          if (key_index < LED_STRIP_LED_NUMBERS) {
            led_strip_set_pixel(led_strip, key_index,
                                key->velocity * LED_VELOCITY_SCALE, 0, 0);
            led_strip_refresh(led_strip);
          }
          send_midi_note(key->midi_note, key->velocity, true);
        }
        // 2. Fallback (SA only)
        else if (!key->key_pressed) {
          key->key_pressed = true;
          key->velocity = 127;

          ESP_LOGW(TAG, "Key %d (%s%d): SA WITHOUT S1 (Fallback)", key_index,
                   get_note_name(key->midi_note), (key->midi_note / 12) - 1);

          send_midi_note(key->midi_note, 127, true);
          if (key_index < LED_STRIP_LED_NUMBERS) {
            led_strip_set_pixel(led_strip, key_index, LED_MAX_BRIGHTNESS, 0, 0);
            led_strip_refresh(led_strip);
          }
        }
      }
    }
  }

  // Release Condition (Both Released)
  if (!key->s1_stable && !key->sa_stable && key->key_pressed) {
    key->key_pressed = false;
    ESP_LOGI(TAG, "Key %d (%s%d): Released", key_index,
             get_note_name(key->midi_note), (key->midi_note / 12) - 1);

    if (key_index < LED_STRIP_LED_NUMBERS) {
      led_strip_set_pixel(led_strip, key_index, LED_OFF, LED_OFF, LED_OFF);
      led_strip_refresh(led_strip);
    }
    send_midi_note(key->midi_note, 0, false);
  }

  // Timeout Check
  if (key->timer_running && (now - key->start_time > VELOCITY_TIMEOUT_US)) {
    key->timer_running = false;
    ESP_LOGW(TAG, "Key %d (%s%d): Velocity Timeout", key_index,
             get_note_name(key->midi_note), (key->midi_note / 12) - 1);
  }
}

// ... existing code ...

static void scan_task(void *arg) {
  ESP_LOGI(TAG, "Starting High-Speed Scan Task on Core 1");

  while (1) {
    int64_t loop_start = esp_timer_get_time();

    for (int c = 0; c < COLS_PER_MATRIX; c++) {
      // Drive Column HIGH
      gpio_set_level(shared_cols[c], 1);
      esp_rom_delay_us(10); // Settling time

      for (int r = 0; r < ROWS_PER_MATRIX; r++) {
        int key_index = (r * COLS_PER_MATRIX) + c;

        // Read Rows
        bool s1_active = gpio_get_level(matrices[0].row_gpios[r]);
        bool sa_active = gpio_get_level(matrices[1].row_gpios[r]);

        process_key_logic(key_index, s1_active, sa_active);
      }

      // Drive Column LOW
      gpio_set_level(shared_cols[c], 0);
    }

    // Precise Delay for Loop Frequency
    int64_t took = esp_timer_get_time() - loop_start;
    if (took < SCAN_INTERVAL_US) {
      esp_rom_delay_us(SCAN_INTERVAL_US - took);
    }
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

  // Initialize BLE MIDI
  ret = ble_midi_init();
  if (ret != 0) {
    ESP_LOGE(TAG, "BLE MIDI initialization failed: %d", ret);
    // Continue anyway - keyboard will work without BLE
  }

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
    keys[i].s1_stable = false;
    keys[i].sa_stable = false;
    keys[i].s1_last_change = 0;
    keys[i].sa_last_change = 0;
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

  // Create HIGH Priority Task on Core 1
  xTaskCreatePinnedToCore(scan_task, "scan_task", 4096, NULL,
                          configMAX_PRIORITIES - 1, NULL, 1);

  // Idle in main task or delete
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
