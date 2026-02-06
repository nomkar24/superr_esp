#ifndef BLE_MIDI_H
#define BLE_MIDI_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Initialize the BLE MIDI service and start advertising
 */
void ble_midi_init(void);

/**
 * @brief Send a MIDI packet over BLE
 *
 * @param midi_packet Pointer to the MIDI packet data
 * @param len Length of the packet
 * @return int 0 on success, -1 on failure
 */
int ble_midi_send_packet(uint8_t *midi_packet, size_t len);

#endif // BLE_MIDI_H
