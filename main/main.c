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

// BLE config service removed - simplified to MIDI only
#include "blemidi.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "velocity_matrix";

// --- LED Strip Config ---
#define LED_STRIP_GPIO_PIN 10
#define LED_STRIP_LED_NUMBERS 24
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

// --- LED Animation System ---
#define LED_FRAME_RATE_MS 8 // ~120 FPS for faster response (was 16ms/60fps)
#define LERP_FACTOR 0.12f   // Balanced fade speed

// LED Event Types
typedef enum { LED_EVENT_NOTE_ON, LED_EVENT_NOTE_OFF } led_event_type_t;

typedef struct {
  led_event_type_t type;
  uint8_t key_index;
  uint8_t velocity;
} led_event_t;

// RGB Color Structure
typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} rgb_color_t;

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
#define MIN_TIME_US 2000            // 2ms (Max Velocity 127)
#define MAX_TIME_US 200000          // 200ms (Min Velocity 1)
#define VELOCITY_TIMEOUT_US 2000000 // 2s timeout for slow presses

// Debounce Configuration
#define DEBOUNCE_TIME_US 5000 // 5ms - faster response, still stable
#define SCAN_INTERVAL_US 50   // Target 20kHz scan rate

// LED Configuration
#define LED_VELOCITY_SCALE 2
#define LED_MAX_BRIGHTNESS 100 // Maximum brightness
#define LED_MIN_BRIGHTNESS                                                     \
  15 // Minimum brightness (15%) for soft notes visibility
#define LED_OFF 0
#define LED_SATURATION 0.85f // 85% saturation for vibrant but not harsh colors

#define VELOCITY_CURVE 1.8 // Optimized for better mid-range dynamics (was 2.0)

// LED Attack Effect (makes note-on more punchy)
#define LED_ATTACK_BOOST 1.5f      // 50% brighter during attack
#define LED_ATTACK_DURATION_MS 100 // Attack lasts 100ms

// LED Animation System Globals
static QueueHandle_t led_msgq;
static rgb_color_t pixels[LED_STRIP_LED_NUMBERS];        // Current displayed
static rgb_color_t target_pixels[LED_STRIP_LED_NUMBERS]; // Target colors
static SemaphoreHandle_t
    led_mutex; // Protect target_pixels from race conditions

typedef struct {
  gpio_num_t col_gpios[COLS_PER_MATRIX];
  gpio_num_t row_gpios[ROWS_PER_MATRIX];
} MatrixConfig;

typedef struct {
  int midi_note;
  int64_t start_time;
  int64_t s1_last_change;
  int64_t sa_last_change;
  int64_t sa_release_time;
  bool s1_stable;
  bool sa_stable;
  bool timer_running;
  bool key_pressed;
  int velocity;
  int release_velocity;
  uint8_t last_velocity;     // Track last velocity for LED updates
  int64_t attack_start_time; // For LED attack effect
} KeyState;

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

static const char *note_names[] = {"C",  "C#", "D",  "D#", "E",  "F",
                                   "F#", "G",  "G#", "A",  "A#", "B"};

static const char *get_note_name(int note) { return note_names[note % 12]; }

// BLE MIDI receive callback (optional - for handling incoming MIDI)
void callback_midi_message_received(uint8_t blemidi_port, uint16_t timestamp,
                                    uint8_t midi_status,
                                    uint8_t *remaining_message, size_t len) {
  // Handle incoming MIDI if needed (e.g., for configuration)
  ESP_LOGI(TAG, "MIDI RX: port=%d, status=0x%02x, len=%zu", blemidi_port,
           midi_status, len);
}

// BLE MIDI tick timer callback - called every 1ms to flush output buffer
static void blemidi_tick_timer_callback(void *arg) { blemidi_tick(); }

// --- LED Animation Helper Functions ---

// Linear interpolation for smooth color transitions
static uint8_t lerp_u8(uint8_t a, uint8_t b, float t) {
  return (uint8_t)(a + t * (b - a));
}

static void lerp_color(rgb_color_t *current, const rgb_color_t *target) {
  current->r = lerp_u8(current->r, target->r, LERP_FACTOR);
  current->g = lerp_u8(current->g, target->g, LERP_FACTOR);
  current->b = lerp_u8(current->b, target->b, LERP_FACTOR);
}

// Velocity-based rainbow color gradient with saturation control
// Soft notes (low velocity) = Blue/Cyan
// Medium notes = Green/Yellow
// Hard notes (high velocity) = Orange/Red
static rgb_color_t get_led_color(uint8_t velocity) {
  rgb_color_t color = {0, 0, 0};
  float intensity = (velocity / 127.0f);

  // Apply minimum brightness for visibility
  uint8_t brightness =
      (uint8_t)(intensity * (LED_MAX_BRIGHTNESS - LED_MIN_BRIGHTNESS) +
                LED_MIN_BRIGHTNESS);

  // Map velocity to hue (0-127 → 240° to 0° on color wheel)
  // 240° = Blue, 120° = Green, 60° = Yellow, 0° = Red
  float hue = 240.0f * (1.0f - intensity); // Reverse: soft=blue, hard=red

  // Convert HSV to RGB with saturation control
  // First calculate fully saturated color
  rgb_color_t saturated = {0, 0, 0};

  if (hue >= 240) {
    // Blue (240°-180°)
    float factor = (hue - 180.0f) / 60.0f;
    saturated.b = brightness;
    saturated.g = (uint8_t)((1.0f - factor) * brightness);
  } else if (hue >= 180) {
    // Cyan (180°-120°)
    float factor = (hue - 120.0f) / 60.0f;
    saturated.g = brightness;
    saturated.b = (uint8_t)(factor * brightness);
  } else if (hue >= 120) {
    // Green (120°-60°)
    float factor = (hue - 60.0f) / 60.0f;
    saturated.g = brightness;
    saturated.r = (uint8_t)((1.0f - factor) * brightness);
  } else if (hue >= 60) {
    // Yellow (60°-0°)
    float factor = hue / 60.0f;
    saturated.r = brightness;
    saturated.g = (uint8_t)(factor * brightness);
  } else {
    // Red (0°)
    saturated.r = brightness;
  }

  // Apply saturation (blend with white/gray)
  // saturation = 1.0 means full color, 0.0 means grayscale
  uint8_t gray = brightness;
  color.r =
      (uint8_t)(saturated.r * LED_SATURATION + gray * (1.0f - LED_SATURATION));
  color.g =
      (uint8_t)(saturated.g * LED_SATURATION + gray * (1.0f - LED_SATURATION));
  color.b =
      (uint8_t)(saturated.b * LED_SATURATION + gray * (1.0f - LED_SATURATION));

  return color;
}

// Get LED color with attack effect (brighter during initial attack)
static rgb_color_t get_led_color_with_attack(uint8_t velocity,
                                             int64_t attack_start_time) {
  rgb_color_t color = get_led_color(velocity);

  // Apply attack boost if within attack duration
  if (attack_start_time > 0) {
    int64_t now = esp_timer_get_time();
    int64_t attack_elapsed_ms = (now - attack_start_time) / 1000;

    if (attack_elapsed_ms < LED_ATTACK_DURATION_MS) {
      // Boost brightness during attack phase
      color.r = (uint8_t)(color.r * LED_ATTACK_BOOST);
      color.g = (uint8_t)(color.g * LED_ATTACK_BOOST);
      color.b = (uint8_t)(color.b * LED_ATTACK_BOOST);

      // Clamp to max brightness
      if (color.r > 255)
        color.r = 255;
      if (color.g > 255)
        color.g = 255;
      if (color.b > 255)
        color.b = 255;
    }
  }

  return color;
}

// Function to calculate velocity from time delta (us)
static uint8_t calculate_velocity(int64_t delta_us) {
  if (delta_us < MIN_TIME_US)
    return 127;
  if (delta_us > MAX_TIME_US)
    return 1;

  float normalized = 1.0f - ((float)(delta_us - MIN_TIME_US) /
                             (float)(MAX_TIME_US - MIN_TIME_US));
  float curved = powf(normalized, VELOCITY_CURVE);
  int velocity = (int)(curved * 126.0f) + 1;

  // No sensitivity scaling - removed for simplicity

  if (velocity > 127)
    velocity = 127;
  if (velocity < 1)
    velocity = 1;

  return (uint8_t)velocity;
}

// BLE MIDI tick timer
static esp_timer_handle_t blemidi_tick_timer = NULL;

static int send_midi_note(uint8_t note, uint8_t velocity, bool note_on) {
  // Create MIDI message (3 bytes: status, note, velocity)
  uint8_t message[3];
  message[0] = note_on ? 0x90 : 0x80; // Note On/Off on channel 0
  message[1] = note & 0x7F;
  message[2] = velocity & 0x7F;

  // Send via blemidi library (returns < 0 if not connected)
  int result = blemidi_send_message(0, message, sizeof(message));
  if (result < 0) {
    // Silently ignore - not connected yet or connection lost
    return 0;
  }
  return result;
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
        ESP_LOGI(TAG, "Key %d (%s%d): S1 (First Contact) detected", key_index,
                 get_note_name(key->midi_note), (key->midi_note / 12) - 1);
      }
      // S1 Falling Edge (Release Complete)
      else if (!key->s1_stable) {
        if (key->key_pressed) {
          int64_t release_delta = now - key->sa_release_time;
          int release_vel = calculate_velocity(release_delta);

          ESP_LOGI(TAG, "Key %d (%s%d): Released | Delta: %lld us | RelVel: %d",
                   key_index, get_note_name(key->midi_note),
                   (key->midi_note / 12) - 1, (long long)release_delta,
                   release_vel);

          // Direct LED update (instant, no queue delay) - THREAD SAFE
          if (key_index < LED_STRIP_LED_NUMBERS) {
            if (xSemaphoreTake(led_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
              target_pixels[key_index] =
                  (rgb_color_t){0, 0, 0}; // Fade to black
              xSemaphoreGive(led_mutex);
            }
          }

          // Reset attack effect
          key->attack_start_time = 0;

          send_midi_note(key->midi_note, release_vel, false);

          key->key_pressed = false;
        } else if (key->timer_running) {
          key->timer_running = false;
          ESP_LOGW(TAG, "Key %d (%s%d): S1 Released before SA (Aborted)",
                   key_index, get_note_name(key->midi_note),
                   (key->midi_note / 12) - 1);
        }
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

          ESP_LOGI(TAG,
                   "Key %d (%s%d): SA (Second Contact) detected | Delta: %lld "
                   "us | Vel: %d",
                   key_index, get_note_name(key->midi_note),
                   (key->midi_note / 12) - 1, delta_us, key->velocity);

          // Direct LED update (instant, no queue delay) - THREAD SAFE
          if (key_index < LED_STRIP_LED_NUMBERS) {
            if (xSemaphoreTake(led_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
              key->attack_start_time =
                  esp_timer_get_time(); // Start attack effect
              target_pixels[key_index] = get_led_color_with_attack(
                  key->velocity, key->attack_start_time);
              xSemaphoreGive(led_mutex);
            }
          }
          key->last_velocity = key->velocity; // Track for LED

          send_midi_note(key->midi_note, key->velocity, true);
        }
        // 2. Fallback (SA only)
        else if (!key->key_pressed) {
          key->key_pressed = true;
          key->velocity = 127;

          ESP_LOGW(TAG, "Key %d (%s%d): SA WITHOUT S1 (Fallback)", key_index,
                   get_note_name(key->midi_note), (key->midi_note / 12) - 1);

          // Don't send LED event for fallback - prevents bounce-triggered stuck
          // LEDs led_event_t led_event = {.type = LED_EVENT_NOTE_ON,
          //                          .key_index = key_index,
          //                          .velocity = 127};
          // xQueueSend(led_msgq, &led_event, 0);

          send_midi_note(key->midi_note, 127, true);
        }
      }
      // SA Falling Edge (Start of Release)
      else if (!key->sa_stable) {
        key->sa_release_time = now;
      }
    }
  }

  // Timeout Check
  if (key->timer_running && (now - key->start_time > VELOCITY_TIMEOUT_US)) {
    key->timer_running = false;
    ESP_LOGW(TAG, "Key %d (%s%d): Velocity Timeout", key_index,
             get_note_name(key->midi_note), (key->midi_note / 12) - 1);
  }
}

// LED Thread - Runs at 60 FPS on Core 0
static void led_thread_entry(void *arg) {
  ESP_LOGI(TAG, "LED Animation Thread started @ 60 FPS on Core 0");

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t frame_delay = pdMS_TO_TICKS(LED_FRAME_RATE_MS);

  while (1) {
    // Phase 1: Process incoming events from queue
    led_event_t event;
    while (xQueueReceive(led_msgq, &event, 0) == pdTRUE) {
      if (event.key_index >= LED_STRIP_LED_NUMBERS)
        continue;

      if (event.type == LED_EVENT_NOTE_ON) {
        // Calculate target color - simple white gradient based on velocity
        target_pixels[event.key_index] = get_led_color(event.velocity);
      } else {
        // Note OFF - fade to black
        target_pixels[event.key_index] = (rgb_color_t){0, 0, 0};
      }
    }

    // Phase 2: Animation - Lerp current toward target (THREAD SAFE)
    // Also update attack effect for active notes (optimized)
    if (xSemaphoreTake(led_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      int64_t now = esp_timer_get_time();

      // Update attack effect ONLY for keys that are pressed AND within attack
      // duration
      for (int i = 0; i < NUM_KEYS && i < LED_STRIP_LED_NUMBERS; i++) {
        if (keys[i].key_pressed && keys[i].attack_start_time > 0) {
          int64_t attack_elapsed_ms = (now - keys[i].attack_start_time) / 1000;

          // Only update if still within attack duration
          if (attack_elapsed_ms < LED_ATTACK_DURATION_MS) {
            target_pixels[i] = get_led_color_with_attack(
                keys[i].velocity, keys[i].attack_start_time);
          }
        }
      }

      // Lerp animation
      for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
        lerp_color(&pixels[i], &target_pixels[i]);
      }
      xSemaphoreGive(led_mutex);
    }

    // Phase 3: Render to hardware
    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
      led_strip_set_pixel(led_strip, i, pixels[i].r, pixels[i].g, pixels[i].b);
    }
    led_strip_refresh(led_strip);

    // Phase 4: Maintain 60 FPS timing
    vTaskDelayUntil(&last_wake_time, frame_delay);
  }
}

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

  // Initialize BLE MIDI using library
  ESP_LOGI(TAG, "Initializing BLE MIDI library...");
  int status = blemidi_init(callback_midi_message_received);
  ESP_LOGI(TAG, "blemidi_init() returned: %d", status);

  if (status < 0) {
    ESP_LOGE(TAG, "BLE MIDI initialization failed: %d", status);
    ESP_LOGE(TAG, "CRITICAL: BLE MIDI will not work!");
    // Continue anyway - keyboard will work without BLE
  } else {
    ESP_LOGI(TAG, "BLE MIDI initialized successfully");
    ESP_LOGI(TAG, "Device should be advertising as: Superr_MIDI");

    // Create timer to call blemidi_tick() every 1ms
    esp_timer_create_args_t blemidi_tick_args = {
        .callback = &blemidi_tick_timer_callback,
        .arg = NULL,
        .name = "blemidi_tick"};
    ret = esp_timer_create(&blemidi_tick_args, &blemidi_tick_timer);
    if (ret == ESP_OK) {
      ret = esp_timer_start_periodic(
          blemidi_tick_timer, 5000); // 5ms - optimized for lower CPU usage
      if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BLE MIDI tick timer started (5ms interval)");
      } else {
        ESP_LOGE(TAG, "Failed to start BLE MIDI tick timer: %d", ret);
      }
    } else {
      ESP_LOGE(TAG, "Failed to create BLE MIDI tick timer: %d", ret);
    }
  }

  // BLE Config Service removed - simplified to MIDI only

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

  // Create LED message queue and mutex for thread safety
  led_msgq = xQueueCreate(128, sizeof(led_event_t));
  led_mutex = xSemaphoreCreateMutex();
  if (led_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create LED mutex!");
  }
  if (led_msgq == NULL) {
    ESP_LOGE(TAG, "FATAL: Failed to create LED message queue");
    abort(); // Cannot continue without LED queue
  }
  ESP_LOGI(TAG, "LED message queue created (128 events)");
  // Initialize pixel buffers to black
  memset(pixels, 0, sizeof(pixels));
  memset(target_pixels, 0, sizeof(target_pixels));

  // Create LED animation thread on Core 0 with priority 4
  // Priority: Scan (5) > LED (4) > Idle (0)
  xTaskCreatePinnedToCore(led_thread_entry, "led_thread", 4096, NULL, 4, NULL,
                          0);
  ESP_LOGI(TAG, "LED animation thread created on Core 0");

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
    keys[i].sa_release_time = 0;
    keys[i].attack_start_time = 0; // Initialize attack timer
  }

  // Reset all pins to clear any default JTAG/Boot functions
  for (int c = 0; c < COLS_PER_MATRIX; c++) {
    gpio_reset_pin(shared_cols[c]);
  }
  for (int m = 0; m < NUM_MATRICES; m++) {
    for (int r = 0; r < ROWS_PER_MATRIX; r++) {
      gpio_reset_pin(matrices[m].row_gpios[r]);
    }
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
  xTaskCreatePinnedToCore(scan_task, "scan_task", 4096, NULL, 5, NULL, 1);

  // Idle in main task or delete
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
