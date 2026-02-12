#pragma once

#include <stdint.h>

// Global Configuration Variables
extern uint8_t g_sensitivity; // 0-100
extern uint8_t g_led_theme;   // 0-2
extern int8_t g_transpose;    // -12 to +12

/**
 * @brief Initialize the BLE Configuration Service
 *
 * @return int 0 on success
 */
int ble_config_init(void);
