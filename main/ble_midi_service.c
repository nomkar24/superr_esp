#include "ble_midi_service.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "BLE_MIDI"

// =============================================================================
// BLE MIDI Service & Characteristic UUIDs
// =============================================================================
// BLE MIDI Service UUID: 03B80E5A-EDE8-4B33-A751-6CE34EC4C700
static uint8_t midi_service_uuid[16] = {0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C,
                                        0x51, 0xA7, 0x33, 0x4B, 0xE8, 0xED,
                                        0x5A, 0x0E, 0xB8, 0x03};

// MIDI I/O Characteristic UUID: 7772E5DB-3868-4112-A1A9-F2669D106BF3
static uint8_t midi_char_uuid[16] = {0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2,
                                     0xA9, 0xA1, 0x12, 0x41, 0x68, 0x38,
                                     0xDB, 0xE5, 0x72, 0x77};

// =============================================================================
// Connection Tracking & State Management
// =============================================================================
static int conn_id = -1;
static uint16_t midi_interface = 0;
static bool is_connected = false;
static bool notifications_enabled = false;
static uint16_t mtu_size = 23; // Default MTU size

static uint8_t midi_data_buf[20] = {0};
static uint8_t midi_data_len = 0;

// Timer for delayed connection parameter update
static esp_timer_handle_t conn_params_timer = NULL;
static esp_bd_addr_t remote_bda_static = {0};

static void conn_params_timer_callback(void *arg) {
  if (is_connected && conn_id != -1) {
    esp_ble_conn_update_params_t conn_params = {0};
    memcpy(conn_params.bda, (esp_bd_addr_t *)arg, sizeof(esp_bd_addr_t));
    conn_params.min_int = 0x0C; // 15ms (min interval = value * 1.25ms)
    conn_params.max_int = 0x18; // 30ms (max interval = value * 1.25ms)
    conn_params.latency = 0;    // No slave latency
    conn_params.timeout = 400;  // 4000ms supervision timeout

    esp_ble_gap_update_conn_params(&conn_params);
    ESP_LOGI(TAG, "Requesting optimized connection params (delayed 3s)");
  }
}

// =============================================================================
// GATT Profile Configuration
// =============================================================================
#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

static esp_gatt_char_prop_t midi_property = 0;

// Forward declarations
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event,
                                          esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst {
  esp_gatts_cb_t gatts_cb;
  uint16_t gatts_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_handle;
  esp_gatt_srvc_id_t service_id;
  uint16_t char_handle;
  esp_bt_uuid_t char_uuid;
  esp_gatt_perm_t perm;
  esp_gatt_char_prop_t property;
  uint16_t descr_handle;
  esp_bt_uuid_t descr_uuid;
} gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] =
        {
            .gatts_cb = gatts_profile_a_event_handler,
            .gatts_if = ESP_GATT_IF_NONE,
        },
};

// =============================================================================
// BLE Advertising Configuration
// =============================================================================
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = false,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = midi_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// =============================================================================
// GAP Event Handler
// =============================================================================
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
  switch (event) {
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    esp_ble_gap_config_adv_data(&scan_rsp_data);
    break;
  case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
    esp_ble_gap_start_advertising(&adv_params);
    break;
  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(TAG, "Advertising start failed");
    } else {
      ESP_LOGI(TAG, "BLE MIDI advertising as 'SUPERR_MIDI'");
    }
    break;
  case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
    ESP_LOGI(
        TAG,
        "Connection params updated: status=%d, min_int=%d, max_int=%d, "
        "latency=%d, timeout=%d",
        param->update_conn_params.status, param->update_conn_params.min_int,
        param->update_conn_params.max_int, param->update_conn_params.latency,
        param->update_conn_params.timeout);
    break;
  case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
    if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(TAG, "Set local privacy failed, error code=%x",
               param->local_privacy_cmpl.status);
    }
    break;
  default:
    break;
  }
}

// =============================================================================
// GATT Service Event Handler
// =============================================================================
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event,
                                          esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t *param) {
  switch (event) {
  case ESP_GATTS_REG_EVT:
    gl_profile_tab[PROFILE_APP_ID].service_id.is_primary = true;
    gl_profile_tab[PROFILE_APP_ID].service_id.id.inst_id = 0x00;
    gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_128;
    memcpy(gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.uuid.uuid128,
           midi_service_uuid, ESP_UUID_LEN_128);

    esp_ble_gap_set_device_name("SUPERR_MIDI");
    esp_ble_gap_config_adv_data(&adv_data);

    esp_ble_gatts_create_service(gatts_if,
                                 &gl_profile_tab[PROFILE_APP_ID].service_id, 4);
    break;

  case ESP_GATTS_CREATE_EVT:
    gl_profile_tab[PROFILE_APP_ID].service_handle =
        param->create.service_handle;
    gl_profile_tab[PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_128;
    memcpy(gl_profile_tab[PROFILE_APP_ID].char_uuid.uuid.uuid128,
           midi_char_uuid, ESP_UUID_LEN_128);

    esp_ble_gatts_start_service(gl_profile_tab[PROFILE_APP_ID].service_handle);

    // MIDI I/O Characteristic: READ | WRITE_WITHOUT_RESP | NOTIFY
    midi_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE |
                    ESP_GATT_CHAR_PROP_BIT_NOTIFY |
                    ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

    esp_ble_gatts_add_char(gl_profile_tab[PROFILE_APP_ID].service_handle,
                           &gl_profile_tab[PROFILE_APP_ID].char_uuid,
                           ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                           midi_property, NULL, NULL);
    break;

  case ESP_GATTS_ADD_CHAR_EVT:
    gl_profile_tab[PROFILE_APP_ID].char_handle = param->add_char.attr_handle;
    midi_interface = gatts_if;

    // Add Client Characteristic Configuration Descriptor (CCC)
    gl_profile_tab[PROFILE_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
    gl_profile_tab[PROFILE_APP_ID].descr_uuid.uuid.uuid16 =
        ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
    esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_APP_ID].service_handle,
                                 &gl_profile_tab[PROFILE_APP_ID].descr_uuid,
                                 ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL,
                                 NULL);
    break;

  case ESP_GATTS_CONNECT_EVT:
    conn_id = param->connect.conn_id;
    is_connected = true;
    ESP_LOGI(TAG, "BLE MIDI Connected (ID: %d) - waiting for stability...",
             conn_id);

    // Copy address to a static buffer to ensure its validity for the timer
    // callback
    memcpy(remote_bda_static, param->connect.remote_bda, sizeof(esp_bd_addr_t));

    // Create timer if it doesn't exist, or delete and re-create to update arg
    if (conn_params_timer) {
      esp_timer_delete(conn_params_timer);
      conn_params_timer = NULL; // Reset handle after deletion
    }

    esp_timer_create_args_t timer_args = {.callback =
                                              &conn_params_timer_callback,
                                          .arg = &remote_bda_static,
                                          .name = "conn_params_timer"};
    esp_timer_create(&timer_args, &conn_params_timer);
    esp_timer_start_once(conn_params_timer, 3000000); // 3 seconds
    break;

  case ESP_GATTS_DISCONNECT_EVT:
    is_connected = false;
    notifications_enabled = false;

    // Stop timer if pending
    if (conn_params_timer) {
      esp_timer_stop(conn_params_timer);
    }

    ESP_LOGI(TAG, "BLE MIDI Disconnected (reason: 0x%02x)",
             param->disconnect.reason);

    // Restart advertising
    esp_ble_gap_start_advertising(&adv_params);
    break;

  case ESP_GATTS_WRITE_EVT:
    if (!param->write.is_prep) {
      // CCC (Client Characteristic Configuration) changed
      if (param->write.len == 2 && param->write.value[0] == 0x01 &&
          param->write.value[1] == 0x00) {
        notifications_enabled = true;
        ESP_LOGI(TAG, "MIDI notifications enabled");
      } else if (param->write.len == 2 && param->write.value[0] == 0x00 &&
                 param->write.value[1] == 0x00) {
        notifications_enabled = false;
        ESP_LOGW(TAG, "MIDI notifications disabled");
      } else {
        // Handle incoming MIDI data (optional - for bidirectional MIDI)
        ESP_LOGI(TAG, "Received MIDI data: %d bytes", param->write.len);
      }
    }
    break;

  case ESP_GATTS_READ_EVT: {
    // Return last MIDI data sent (optional)
    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.handle = param->read.handle;
    rsp.attr_value.len = midi_data_len;
    memcpy(rsp.attr_value.value, midi_data_buf, midi_data_len);
    esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                param->read.trans_id, ESP_GATT_OK, &rsp);
    break;
  }

  case ESP_GATTS_MTU_EVT:
    mtu_size = param->mtu.mtu;
    ESP_LOGI(TAG, "MTU exchange complete: %d bytes", mtu_size);
    break;

  default:
    break;
  }
}

// =============================================================================
// Main GATT Event Handler
// =============================================================================
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
  // Handle registration event
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
    } else {
      ESP_LOGE(TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id,
               param->reg.status);
      return;
    }
  }

  // Dispatch to profile handlers
  for (int idx = 0; idx < PROFILE_NUM; idx++) {
    if (gatts_if == ESP_GATT_IF_NONE ||
        gatts_if == gl_profile_tab[idx].gatts_if) {
      if (gl_profile_tab[idx].gatts_cb) {
        gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
      }
    }
  }
}

// =============================================================================
// Public API Functions
// =============================================================================

/**
 * @brief Initialize BLE MIDI service
 *
 * @return int 0 on success, negative on error
 */
int ble_midi_init(void) {
  esp_err_t ret;

  ESP_LOGI(TAG, "Initializing BLE MIDI...");

  // Release Classic BT memory
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  // Initialize BT controller
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(TAG, "Bluetooth controller init failed (err %d)", ret);
    return ret;
  }

  // Enable BLE mode
  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(TAG, "Bluetooth controller enable failed (err %d)", ret);
    return ret;
  }

  // Initialize Bluedroid stack with default config
  esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
  ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
  if (ret) {
    ESP_LOGE(TAG, "Bluedroid init failed (err %d)", ret);
    return ret;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(TAG, "Bluedroid enable failed (err %d)", ret);
    return ret;
  }

  // Register GATT callbacks
  ret = esp_ble_gatts_register_callback(gatts_event_handler);
  if (ret) {
    ESP_LOGE(TAG, "GATTS register callback failed (err 0x%x)", ret);
    return ret;
  }

  // Register GAP callbacks
  ret = esp_ble_gap_register_callback(gap_event_handler);
  if (ret) {
    ESP_LOGE(TAG, "GAP register callback failed (err 0x%x)", ret);
    return ret;
  }

  // Register GATT application
  ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
  if (ret) {
    ESP_LOGE(TAG, "GATTS app register failed (err 0x%x)", ret);
    return ret;
  }

  ESP_LOGI(TAG, "BLE MIDI initialized successfully");
  return 0;
}

/**
 * @brief Send MIDI data over BLE
 *
 * @param data Pointer to MIDI data buffer
 * @param len Length of data to send
 * @return int 0 on success, negative on error
 */
int ble_midi_send(const uint8_t *data, uint8_t len) {
  if (!data || len == 0 || len > sizeof(midi_data_buf)) {
    return -1; // Invalid parameters
  }

  // Store for read operations
  memcpy(midi_data_buf, data, len);
  midi_data_len = len;

  // Check connection status
  if (!is_connected) {
    ESP_LOGW(TAG, "MIDI NOT SENT - No BLE connection!");
    return -2;
  }

  if (!notifications_enabled) {
    ESP_LOGW(TAG, "MIDI NOT SENT - Notifications not enabled!");
    return -3;
  }

  // Send notification
  int err = esp_ble_gatts_send_indicate(
      midi_interface, conn_id, gl_profile_tab[PROFILE_APP_ID].char_handle, len,
      (uint8_t *)data, false);
  if (err) {
    ESP_LOGE(TAG, "MIDI notify failed (err %d)", err);
    return err;
  }

  // Debug log for MIDI packets
  if (len == 6) {
    ESP_LOGI(TAG, "MIDI Sent: [%02X %02X %02X %02X %02X %02X]", data[0],
             data[1], data[2], data[3], data[4], data[5]);
  }

  return 0;
}

/**
 * @brief Check if BLE MIDI is connected and ready
 *
 * @return true if connected and notifications enabled
 * @return false otherwise
 */
bool ble_midi_is_connected(void) {
  return (is_connected && notifications_enabled);
}

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
                     uint8_t *buf, size_t buf_len) {
  if (!buf || buf_len < 5) {
    return -1;
  }

  if (note > 127 || velocity > 127 || channel > 15) {
    return -2;
  }

  // Get 13-bit timestamp (milliseconds)
  uint16_t timestamp = (esp_timer_get_time() / 1000) & 0x1FFF;

  // BLE MIDI packet format
  buf[0] = 0x80 | ((timestamp >> 7) & 0x3F); // Header with timestamp high
  buf[1] = 0x80 | (timestamp & 0x7F);        // Timestamp low
  buf[2] = 0x90 | (channel & 0x0F);          // Note On + channel
  buf[3] = note & 0x7F;                      // Note number
  buf[4] = velocity & 0x7F;                  // Velocity

  return 5;
}

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
                      uint8_t *buf, size_t buf_len) {
  if (!buf || buf_len < 5) {
    return -1;
  }

  if (note > 127 || velocity > 127 || channel > 15) {
    return -2;
  }

  // Get 13-bit timestamp (milliseconds)
  uint16_t timestamp = (esp_timer_get_time() / 1000) & 0x1FFF;

  // BLE MIDI packet format
  buf[0] = 0x80 | ((timestamp >> 7) & 0x3F); // Header with timestamp high
  buf[1] = 0x80 | (timestamp & 0x7F);        // Timestamp low
  buf[2] = 0x80 | (channel & 0x0F);          // Note Off + channel
  buf[3] = note & 0x7F;                      // Note number
  buf[4] = velocity & 0x7F;                  // Velocity

  return 5;
}

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
                            uint8_t *buf, size_t buf_len) {
  if (!buf || buf_len < 5) {
    return -1;
  }

  if (cc_num > 127 || value > 127 || channel > 15) {
    return -2;
  }

  // Get 13-bit timestamp (milliseconds)
  uint16_t timestamp = (esp_timer_get_time() / 1000) & 0x1FFF;

  // BLE MIDI packet format
  buf[0] = 0x80 | ((timestamp >> 7) & 0x3F); // Header with timestamp high
  buf[1] = 0x80 | (timestamp & 0x7F);        // Timestamp low
  buf[2] = 0xB0 | (channel & 0x0F);          // Control Change + channel
  buf[3] = cc_num & 0x7F;                    // CC number
  buf[4] = value & 0x7F;                     // CC value

  return 5;
}
