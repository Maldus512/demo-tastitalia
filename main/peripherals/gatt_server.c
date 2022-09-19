/**
 * @file gatt_server.c
 * @author Maldus512 ()
 * @brief BLE GATT server module
 * @version 0.1
 * @date 2022-09-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "gatt_server.h"
#include "esp_gatt_common_api.h"
#include "serializer/serializer.h"

#include "leds.h"


#define PROFILE_NUM        1
#define PROFILE_APP_IDX    0
#define ESP_APP_ID         0x55
#define SAMPLE_DEVICE_NAME "ESP_GATTS_DEMO"
#define SVC_INST_ID        0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
 *  the data length must be less than GATTS_CHAR_VAL_LEN_MAX.
 */
#define GATTS_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE   1024
#define CHAR_DECLARATION_SIZE  (sizeof(uint8_t))

#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static const char *TAG = "Gatt server";

static uint8_t adv_config_done = 0;

static uint16_t gatt_handle_table[HRS_IDX_NB];

typedef struct {
    uint8_t *prepare_buf;
    int      prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,     // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010,     // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 0,        // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL,     // test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0,        // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL,     //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min       = 200,
    .adv_int_max       = 200,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t       gatts_cb;
    uint16_t             gatts_if;
    uint16_t             app_id;
    uint16_t             conn_id;
    uint16_t             service_handle;
    esp_gatt_srvc_id_t   service_id;
    uint16_t             char_handle;
    esp_bt_uuid_t        char_uuid;
    esp_gatt_perm_t      perm;
    esp_gatt_char_prop_t property;
    uint16_t             descr_handle;
    esp_bt_uuid_t        descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT
 */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] =
        {
            .gatts_cb = gatts_profile_event_handler,
            .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        },
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID_TEST = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_BUTTONS = 0x0001;
static const uint16_t GATTS_CHAR_UUID_LEDS    = 0x0002;
static const uint16_t GATTS_CHAR_UUID_BATTERY = 0x0003;

static const uint16_t    primary_service_uuid       = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t    character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint8_t     char_prop_read             = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t     char_prop_read_write       = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static SemaphoreHandle_t sem                        = NULL;
static uint8_t           buttons_value[4]           = {0};
static uint8_t           leds_value[4]              = {0};
static uint8_t           battery_adc_value[4]       = {0};


/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] = {
    // Service Declaration
    [IDX_SVC] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t),
                  sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

    /* Characteristic Declaration */
    [IDX_CHAR_BUTTONS] = {{ESP_GATT_AUTO_RSP},
                          {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                           CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
    /* Characteristic Value */
    [IDX_CHAR_VAL_BUTTONS] = {{ESP_GATT_RSP_BY_APP},
                              {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_BUTTONS,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(buttons_value),
                               (uint8_t *)buttons_value}},

    /* Characteristic Declaration */
    [IDX_CHAR_LEDS] = {{ESP_GATT_AUTO_RSP},
                       {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
    /* Characteristic Value */
    [IDX_CHAR_VAL_LEDS] = {{ESP_GATT_AUTO_RSP},
                           {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_LEDS, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                            GATTS_CHAR_VAL_LEN_MAX, sizeof(leds_value), (uint8_t *)leds_value}},

    /* Characteristic Declaration */
    [IDX_CHAR_BATTERY] = {{ESP_GATT_AUTO_RSP},
                          {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                           CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
    /* Characteristic Value */
    [IDX_CHAR_VAL_BATTERY] = {{ESP_GATT_RSP_BY_APP},
                              {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_BATTERY, ESP_GATT_PERM_READ,
                               GATTS_CHAR_VAL_LEN_MAX, sizeof(battery_adc_value), (uint8_t *)battery_adc_value}},
};


/**
 * @brief GAP event handler
 *
 * @param event
 * @param param
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "advertising start failed");
            } else {
                ESP_LOGI(TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising stop failed");
            } else {
                ESP_LOGI(TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG,
                     "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, "
                     "timeout = %d",
                     param->update_conn_params.status, param->update_conn_params.min_int,
                     param->update_conn_params.max_int, param->update_conn_params.conn_int,
                     param->update_conn_params.latency, param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}


/**
 * @brief GATTS profile event handler
 *
 * @param event
 * @param gatts_if
 * @param param
 */
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret) {
                ESP_LOGE(TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
            // config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret) {
                ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            // config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret) {
                ESP_LOGE(TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
            if (create_attr_ret) {
                ESP_LOGE(TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        } break;
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d", param->read.conn_id,
                     param->read.trans_id, param->read.handle);
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

            if (gatt_handle_table[IDX_CHAR_VAL_BUTTONS] == param->read.handle) {
                rsp.attr_value.handle = param->read.handle;
                rsp.attr_value.len    = 4;
                xSemaphoreTake(sem, portMAX_DELAY);
                rsp.attr_value.value[0] = buttons_value[0];
                rsp.attr_value.value[1] = buttons_value[1];
                rsp.attr_value.value[2] = buttons_value[2];
                rsp.attr_value.value[3] = buttons_value[3];
                xSemaphoreGive(sem);
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);

            } else if (gatt_handle_table[IDX_CHAR_VAL_BATTERY] == param->read.handle) {
                rsp.attr_value.handle = param->read.handle;
                rsp.attr_value.len    = 4;
                xSemaphoreTake(sem, portMAX_DELAY);
                rsp.attr_value.value[0] = battery_adc_value[0];
                rsp.attr_value.value[1] = battery_adc_value[1];
                rsp.attr_value.value[2] = battery_adc_value[2];
                rsp.attr_value.value[3] = battery_adc_value[3];
                xSemaphoreGive(sem);
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            }
            break;
        }
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep) {
                // the data length of gattc write  must be less than GATTS_CHAR_VAL_LEN_MAX.
                ESP_LOGI(TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle,
                         param->write.len);
                esp_log_buffer_hex(TAG, param->write.value, param->write.len);

                if (gatt_handle_table[IDX_CHAR_VAL_LEDS] == param->write.handle && param->write.len == 4) {
                    leds_set(LEDS_LED_LEFT_SYMBOL, param->write.value[0]);
                    leds_set(LEDS_LED_LEFT_ARROW, param->write.value[1]);
                    leds_set(LEDS_LED_RIGHT_ARROW, param->write.value[2]);
                    leds_set(LEDS_LED_RIGHT_SYMBOL, param->write.value[3]);
                }

                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK,
                                                NULL);
                }
            } else {
                /* handle prepare write */
            }
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_CHAR_VAL_LEN_MAX.
            ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status,
                     param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters
             * restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;     // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;     // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;      // timeout = 400*10ms = 4000ms
            // start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            } else if (param->add_attr_tab.num_handle != HRS_IDX_NB) {
                ESP_LOGE(TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, HRS_IDX_NB);
            } else {
                ESP_LOGI(TAG, "create attribute table successfully, the number handle = %d\n",
                         param->add_attr_tab.num_handle);
                memcpy(gatt_handle_table, param->add_attr_tab.handles, sizeof(gatt_handle_table));
                esp_ble_gatts_start_service(gatt_handle_table[IDX_SVC]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}


/**
 * @brief GATTS event handler
 *
 * @param event
 * @param gatts_if
 * @param param
 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(TAG, "reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


void gatt_server_init(void) {
    ESP_LOGI(TAG, "Gatt server inizialization");
    sem = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
}


void gatt_server_start(void) {
    esp_err_t ret;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret                               = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    } else {
        ESP_LOGI(TAG, "BT controller initialized");
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    } else {
        ESP_LOGI(TAG, "BT controller enabled");
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    } else {
        ESP_LOGI(TAG, "Bluedroid initialized");
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    } else {
        ESP_LOGI(TAG, "Bluedroid enabled");
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

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return;
    } else {
        ESP_LOGI(TAG, "Gatt server registered");
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret) {
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    ESP_LOGI(TAG, "Gatt server started");
}


void gatt_server_stop(void) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_bluedroid_disable());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_bluedroid_deinit());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_bt_controller_disable());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_bt_controller_deinit());

    ESP_LOGI(TAG, "Gatt server stopped");
}


void gatt_server_set_buttons(uint8_t buttons[4]) {
    xSemaphoreTake(sem, portMAX_DELAY);
    memcpy(buttons_value, buttons, 4);
    xSemaphoreGive(sem);
}


void gatt_server_set_battery(uint32_t battery_adc_level) {
    xSemaphoreTake(sem, portMAX_DELAY);
    serialize_uint32_be(battery_adc_value, battery_adc_level);
    xSemaphoreGive(sem);
}