#include "esp_zigbee_core.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false    /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_2048MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */
#define PRESSURE_SENSOR_ATTEN           ADC_ATTEN_DB_6
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Zigbee primary channel mask use in the example */
#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                        \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,      \
    }

typedef enum {
    ENDPOINT_INIT = 9,
    ENDPOINT_BED_SIDE1 = 10,
    ENDPOINT_BED_SIDE2 = 11
} endpoint_t;

uint8_t date_code[] = {
        8,
        // YYYY year
        __DATE__[7], __DATE__[8],__DATE__[9], __DATE__[10],

        // First month letter, Oct Nov Dec = '1' otherwise '0'
        (__DATE__[0] == 'O' || __DATE__[0] == 'N' || __DATE__[0] == 'D') ? '1' : '0',

        // Second month letter
        (__DATE__[0] == 'J') ? ((__DATE__[1] == 'a') ? '1' :       // Jan, Jun or Jul
                                ((__DATE__[2] == 'n') ? '6' : '7')) :
        (__DATE__[0] == 'F') ? '2' :                                // Feb
        (__DATE__[0] == 'M') ? (__DATE__[2] == 'r') ? '3' : '5' :   // Mar or May
        (__DATE__[0] == 'A') ? (__DATE__[1] == 'p') ? '4' : '8' :   // Apr or Aug
        (__DATE__[0] == 'S') ? '9' :                                // Sep
        (__DATE__[0] == 'O') ? '0' :                                // Oct
        (__DATE__[0] == 'N') ? '1' :                                // Nov
        (__DATE__[0] == 'D') ? '2' :                                // Dec
        0,

        // First day letter, replace space with digit
        __DATE__[4] == ' ' ? '0' : __DATE__[4],

        // Second day letter
        __DATE__[5],
};