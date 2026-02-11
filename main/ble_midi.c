#include "ble_midi.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
// #include "esp_system.h" // Unused
// #include "freertos/FreeRTOS.h" // Unused
#include "freertos/task.h"
// #include "nvs_flash.h" // Unused
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TAG "BLE_MIDI"

#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

/* MIDI Service & Characteristic UUIDs */
static uint8_t midi_service_uuid[16] = {0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C,
                                        0x51, 0xA7, 0x33, 0x4B, 0xE8, 0xED,
                                        0x5A, 0x0E, 0xB8, 0x03};

static uint8_t midi_char_uuid[16] = {0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2,
                                     0xA9, 0xA1, 0x12, 0x41, 0x68, 0x38,
                                     0xDB, 0xE5, 0x72, 0x77};

static esp_gatt_char_prop_t midi_property = 0;

static int conn_id = -1;
static uint16_t midi_interface = 0;
static bool is_connected = false;
static bool notifications_enabled = false;

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
    }
    break;
  default:
    break;
  }
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event,
                                          esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t *param) {
  switch (event) {
  case ESP_GATTS_REG_EVT:
    // ESP_LOGI(TAG, "GATTS Register Event: status %d, app_id %d",
    //          param->reg.status, param->reg.app_id);
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
    // ESP_LOGI(TAG, "Create Service: status %d, handle %d",
    // param->create.status,
    //          param->create.service_handle);
    gl_profile_tab[PROFILE_APP_ID].service_handle =
        param->create.service_handle;
    gl_profile_tab[PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_128;
    memcpy(gl_profile_tab[PROFILE_APP_ID].char_uuid.uuid.uuid128,
           midi_char_uuid, ESP_UUID_LEN_128);

    esp_ble_gatts_start_service(gl_profile_tab[PROFILE_APP_ID].service_handle);
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
    ESP_LOGI(TAG, "BLE Connected (ID: %d)", conn_id);
    break;
  case ESP_GATTS_DISCONNECT_EVT:
    is_connected = false;
    ESP_LOGI(TAG, "BLE Disconnected");
    esp_ble_gap_start_advertising(&adv_params);
    break;
  case ESP_GATTS_WRITE_EVT:
    // ESP_LOGI(TAG, "BLE Write Event received");
    if (!param->write.is_prep) {
      // Check if client is enabling/disabling notifications
      if (param->write.len == 2 && param->write.value[0] == 0x01 &&
          param->write.value[1] == 0x00) {
        notifications_enabled = true;
        ESP_LOGI(TAG, "Client ENABLED notifications - MIDI ready!");
      } else if (param->write.len == 2 && param->write.value[0] == 0x00 &&
                 param->write.value[1] == 0x00) {
        notifications_enabled = false;
        ESP_LOGW(TAG, "Client DISABLED notifications");
      }
      // ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);
    }
    break;
  default:
    break;
  }
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
    } else {
      ESP_LOGI(TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id,
               param->reg.status);
      return;
    }
  }

  do {
    int idx;
    for (idx = 0; idx < PROFILE_NUM; idx++) {
      if (gatts_if == ESP_GATT_IF_NONE ||
          gatts_if == gl_profile_tab[idx].gatts_if) {
        if (gl_profile_tab[idx].gatts_cb) {
          gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
        }
      }
    }
  } while (0);
}

void ble_midi_init(void) {
  esp_err_t ret;

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(TAG, "%s initialize controller failed", __func__);
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(TAG, "%s enable controller failed", __func__);
    return;
  }
  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(TAG, "%s init bluetooth failed", __func__);
    return;
  }
  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(TAG, "%s enable bluetooth failed", __func__);
    return;
  }

  ret = esp_ble_gatts_register_callback(gatts_event_handler);
  if (ret) {
    ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
    return;
  }
  ret = esp_ble_gap_register_callback(gap_event_handler);
  if (ret) {
    ESP_LOGE(TAG, "gap register error, error code = %x", ret);
    return;
  }
  ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
  if (ret) {
    ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
    return;
  }
  ESP_LOGI(TAG, "BLE MIDI Initialized");
}

int ble_midi_send_packet(uint8_t *midi_packet, size_t len) {
  if (!is_connected) {
    ESP_LOGW(TAG, "MIDI NOT SENT - No BLE connection!");
    return -1;
  }

  if (!notifications_enabled) {
    ESP_LOGW(TAG, "MIDI NOT SENT - Notifications not enabled!");
    return -1;
  }

  esp_ble_gatts_send_indicate(midi_interface, conn_id,
                              gl_profile_tab[PROFILE_APP_ID].char_handle, len,
                              midi_packet, false);
  if (len == 6) {
    ESP_LOGI(TAG, "MIDI Sent: [%02X %02X %02X %02X %02X %02X]", midi_packet[0],
             midi_packet[1], midi_packet[2], midi_packet[3], midi_packet[4],
             midi_packet[5]);
  }
  return 0;
}
