#include "ble_config_service.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include <string.h>

#define TAG "BLE_CONFIG"

// Default Values
uint8_t g_sensitivity = 50;
uint8_t g_led_theme = 0;
int8_t g_transpose = 0;

// UUIDs
#define CONFIG_SERVICE_UUID 0x00FF // Custom Short UUID for demo
#define CHAR_SENSITIVITY_UUID 0xFF01
#define CHAR_LED_THEME_UUID 0xFF02
#define CHAR_TRANSPOSE_UUID 0xFF03

#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

static uint16_t service_handle;

// Forward declarations
static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
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
} gl_profile = {
    .gatts_cb = gatts_profile_event_handler,
    .gatts_if = ESP_GATT_IF_NONE,
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param) {
  switch (event) {
  case ESP_GATTS_REG_EVT:
    ESP_LOGI(TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status,
             param->reg.app_id);
    gl_profile.service_id.is_primary = true;
    gl_profile.service_id.id.inst_id = 0x00;
    gl_profile.service_id.id.uuid.len = ESP_UUID_LEN_16;
    gl_profile.service_id.id.uuid.uuid.uuid16 = CONFIG_SERVICE_UUID;

    esp_ble_gatts_create_service(gatts_if, &gl_profile.service_id, 10);
    break;

  case ESP_GATTS_CREATE_EVT:
    ESP_LOGI(TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d",
             param->create.status, param->create.service_handle);
    service_handle = param->create.service_handle;

    esp_ble_gatts_start_service(service_handle);

    // Add Sensitivity Characteristic
    esp_bt_uuid_t uuid = {.len = ESP_UUID_LEN_16,
                          .uuid = {.uuid16 = CHAR_SENSITIVITY_UUID}};
    esp_ble_gatts_add_char(
        service_handle, &uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);

    // Add LED Theme Characteristic
    uuid.uuid.uuid16 = CHAR_LED_THEME_UUID;
    esp_ble_gatts_add_char(
        service_handle, &uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);

    // Add Transpose Characteristic
    uuid.uuid.uuid16 = CHAR_TRANSPOSE_UUID;
    esp_ble_gatts_add_char(
        service_handle, &uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
    break;

  case ESP_GATTS_ADD_CHAR_EVT:
    ESP_LOGI(TAG, "ADD_CHAR_EVT, status %d, attr_handle %d, uuid 0x%x",
             param->add_char.status, param->add_char.attr_handle,
             param->add_char.char_uuid.uuid.uuid16);
    break;

  case ESP_GATTS_READ_EVT: {
    ESP_LOGI(TAG, "ESP_GATTS_READ_EVT");
    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.handle = param->read.handle;
    rsp.attr_value.len = 1;

    // Simple logic for demo: return 0. (Real logic would map handles)
    rsp.attr_value.value[0] = 0;

    esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                param->read.trans_id, ESP_GATT_OK, &rsp);
    break;
  }

  case ESP_GATTS_WRITE_EVT: {
    if (!param->write.is_prep) {
      ESP_LOGI(TAG, "GATT_WRITE_EVT, handle %d, value len %d, value[0] %d",
               param->write.handle, param->write.len, param->write.value[0]);

      if (param->write.need_rsp) {
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                    param->write.trans_id, ESP_GATT_OK, NULL);
      }
    }
    break;
  }

  case ESP_GATTS_CONNECT_EVT:
    gl_profile.conn_id = param->connect.conn_id;
    break;

  case ESP_GATTS_DISCONNECT_EVT:
    break;

  default:
    break;
  }
}

int ble_config_init(void) {
  esp_ble_gatts_app_register(1); // App ID 1 (0 is MIDI)
  return 0;
}
