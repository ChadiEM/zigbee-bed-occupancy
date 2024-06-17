#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_zb_bed.h"
#include "string.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

typedef struct channel_definition {
    adc_channel_t channel;
    endpoint_t endpoint;
    uint16_t threshold;
} channel_definition;

// Voltage threshold for triggering an occupied alert (in mV)
static const uint16_t THRESHOLD = 175;

// Define channels and endpoints here
channel_definition channel_definitions[] = {{.channel = ADC_CHANNEL_2, .endpoint = ENDPOINT_BED_SIDE1, .threshold = THRESHOLD},
                                                  {.channel = ADC_CHANNEL_3, .endpoint = ENDPOINT_BED_SIDE2, .threshold = THRESHOLD}};

static const uint8_t channel_definitions_size = sizeof(channel_definitions) / sizeof(channel_definition);

static const char *TAG = "ESP_ZB_BED_OCCUPANCY";

static bool adc_calibration_init(adc_channel_t channel, adc_cali_handle_t *out_handle);

void reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length)
{
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = endpoint,
            .src_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    };
    esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(endpoint, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID);
    memcpy(value_r->data_p, value, value_length);
    esp_zb_zcl_report_attr_cmd_req(&cmd);
}

void bed_occupancy_task(void *pvParameters)
{
   //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = PRESSURE_SENSOR_ATTEN,
    };

    for (int i = 0; i < channel_definitions_size; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel_definitions[i].channel, &config));
    }

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_handle[channel_definitions_size];
    bool do_calibration[channel_definitions_size];

    for (int i = 0; i < channel_definitions_size; i++) {
        do_calibration[i] = adc_calibration_init(channel_definitions[i].channel, &adc1_cali_handle[i]);
    }

    uint8_t gpio_state[channel_definitions_size];
    for (int i = 0; i < channel_definitions_size; i++) {
        gpio_state[i] = 2;
    }

    int adc_raw[10];
    int voltage[10];

    while (1) {
        for (int i = 0; i < channel_definitions_size; i++) {

            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel_definitions[i].channel, &adc_raw[0]));
            // ESP_LOGI(TAG, "Channel[%d] Raw Data: %d", channel_definitions[i].channel, adc_raw[0]);
            if (do_calibration[i]) {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle[i], adc_raw[0], &voltage[0]));
                // ESP_LOGI(TAG, "Channel[%d] Cali Voltage: %d mV", channel_definitions[i].channel, voltage[0]);

                uint8_t data = voltage[0] > channel_definitions[i].threshold ? 1 : 0;

                if (data != gpio_state[i]) {
                    gpio_state[i] = data;
                    reportAttribute(channel_definitions[i].endpoint, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                    ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &data, sizeof(data));
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

/********************* Define functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Start network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            xTaskCreate(bed_occupancy_task, "bed_occupancy_task", 4096, NULL, 5, NULL);
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

// ------------------------------ Cluster BASIC ------------------------------
    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01,
    };
    uint32_t ApplicationVersion = 0x0001;
    uint32_t StackVersion = 0x0002;
    uint32_t HWVersion = 0x0002;
    uint8_t ManufacturerName[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};
    uint8_t ModelIdentifier[] = {13, 'B', 'e', 'd', ' ', 'O', 'c', 'c', 'u', 'p', 'a', 'n', 'c', 'y'};
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &ApplicationVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ManufacturerName);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ModelIdentifier);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, date_code);

    // ------------------------------ Cluster IDENTIFY ------------------------------
    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cluster_cfg);

    esp_zb_attribute_list_t *esp_zb_binary_input_clusters[channel_definitions_size];
    for (int i = 0; i < channel_definitions_size; i++) {
        esp_zb_binary_input_cluster_cfg_t binary_input_cfg = {
                .out_of_service = 0,
                .status_flags = 0,
        };
        uint8_t present_value = 0;
        esp_zb_attribute_list_t *esp_zb_binary_input_cluster = esp_zb_binary_input_cluster_create(&binary_input_cfg);
        esp_zb_binary_input_cluster_add_attr(esp_zb_binary_input_cluster, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &present_value);

        esp_zb_binary_input_clusters[i] = esp_zb_binary_input_cluster;
    }

    // ------------------------------ Create endpoint list ------------------------------
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    // ------------------------------ Create cluster list ------------------------------
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_endpoint_config_t endpoint_config = {.endpoint = ENDPOINT_INIT, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);

    for (int i = 0; i < channel_definitions_size; i++) {
        esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
        esp_zb_cluster_list_add_binary_input_cluster(esp_zb_cluster_list, esp_zb_binary_input_clusters[i], ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t endpoint_config = {.endpoint = channel_definitions[i].endpoint, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID};
        esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);
    }

    /* Register device */
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_channel_t channel, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = channel,
        .atten = PRESSURE_SENSOR_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
        calibrated = true;
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}