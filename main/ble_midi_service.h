#ifndef BLE_MIDI_H
#define BLE_MIDI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Initialize BLE MIDI service
 *
 * Sets up Bluetooth stack, GATT service, and starts advertising
 * as a BLE MIDI device.
 *
 * @return 0 on success, negative error code on failure
 */
int ble_midi_init(void);

/**
 * @brief Send raw BLE MIDI packet
 *
 * @param data Pointer to BLE MIDI packet data
 * @param len Length of packet (max 20 bytes)
 * @return 0 on success, negative error code on failure
 */
int ble_midi_send(const uint8_t *data, uint8_t len);

/**
 * @brief Check if BLE MIDI is connected and ready
 *
 * @return true if connected and notifications enabled
 * @return false otherwise
 */
bool ble_midi_is_connected(void);

/**
 * @brief Create BLE MIDI packet for Note On
 *
 * @param note MIDI note number (0-127, 60=Middle C)
 * @param velocity Note velocity (1-127, 0=Note Off)
 * @param channel MIDI channel (0-15)
 * @param buf Output buffer (minimum 5 bytes)
 * @param buf_len Buffer size
 * @return Length of packet, or negative on error
 */
int ble_midi_note_on(uint8_t note, uint8_t velocity, uint8_t channel,
                     uint8_t *buf, size_t buf_len);

/**
 * @brief Create BLE MIDI packet for Note Off
 *
 * @param note MIDI note number (0-127)
 * @param velocity Release velocity (0-127)
 * @param channel MIDI channel (0-15)
 * @param buf Output buffer (minimum 5 bytes)
 * @param buf_len Buffer size
 * @return Length of packet, or negative on error
 */
int ble_midi_note_off(uint8_t note, uint8_t velocity, uint8_t channel,
                      uint8_t *buf, size_t buf_len);

/**
 * @brief Create BLE MIDI packet for Control Change
 *
 * @param cc_num Control Change number (0-127)
 * @param value CC value (0-127)
 * @param channel MIDI channel (0-15)
 * @param buf Output buffer (minimum 5 bytes)
 * @param buf_len Buffer size
 * @return Length of packet, or negative on error
 */
int ble_midi_control_change(uint8_t cc_num, uint8_t value, uint8_t channel,
                            uint8_t *buf, size_t buf_len);

#endif // BLE_MIDI_H
