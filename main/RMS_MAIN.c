/* This file is the main entry point. This implements:
   - Board initialization (ESP32 specific)
   - Calls corresponding module init routines
   - Handle modem connection /  disconnection
   - Handle MQTT connection / disconnection and sending / receiving messages
   Note: All custom messages are handled in specific modules. For example,
   data logger messages are handled in data_logger.c. Also custom settings in Flash are
   managed in data_logger.c.
*/
#include "esp_timer.h"
#include "driver/uart.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "esp_netif_defaults.h"
#include "mqtt_client.h"
// #include "esp_modem_api.h"
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_mac.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "esp_flash.h"
#include "esp_sntp.h"
#include "esp_app_desc.h"
#include "esp_wifi.h"
#include "Defines.h"
#include "esp_smartconfig.h"
#include "esp_crt_bundle.h"
#include "esp_tls.h"
#include "esp_http_client.h"


#include "string.h"
#include "esp_log.h"
// #include "modbus_params.h"  // for modbus parameters structures
#include "mbcontroller.h"



#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048

#define ESP_INTR_FLAG_DEFAULT 0

#define LED_INDICATION_TIMER_COUNT 300000 
#define LED_TEST_TIMER_COUNT  1000000 * 15

static QueueHandle_t gpio_evt_queue = NULL;
esp_timer_handle_t green_led_action_timer;

const char *boot_reason_str[] = {
    "UNKNOWN",
    "POWERON",
    "EXTERNAL",
    "SOFTWARE",
    "PANIC",
    "INT_WDT",
    "TASK_WDT",
    "WDT",
    "DEEPSLEEP",
    "BROWNOUT",
    "SDIO"
};
esp_reset_reason_t boot_reason;

date_time_t datetime;

#if defined(CONFIG_EXAMPLE_FLOW_CONTROL_NONE)
#define EXAMPLE_FLOW_CONTROL ESP_MODEM_FLOW_CONTROL_NONE
#elif defined(CONFIG_EXAMPLE_FLOW_CONTROL_SW)
#define EXAMPLE_FLOW_CONTROL ESP_MODEM_FLOW_CONTROL_SW
#elif defined(CONFIG_EXAMPLE_FLOW_CONTROL_HW)
#define EXAMPLE_FLOW_CONTROL ESP_MODEM_FLOW_CONTROL_HW
#endif

static const char *TAG = "RMS_MAIN";
static const char *TAG1 = "wifi station";
static const char *TAG2 = "smartconfig_example";
static const char *TAG3 = "sntp";
// static const char *TAG4 = "non smart config";
static const char *TAG5 = "HTTP_CLIENT";
static const char *TAG6 = "MQTT";

static EventGroupHandle_t event_group = NULL;
static const int MQTT_CONNECT_BIT = BIT1;

bool boardReady = false;
bool sntpInitDone = false;

char inverter_type[20];
uint8_t numOfGoodweInverter,numOfDeltaInverter,numOfSungrowInverter,numOfSolisInverter,numOfSingegInverter,numOfGrowattInverter,numOfschniderinverter,numOfSmaInverter,numOfHuaweiInverter,numOfEnergyMeter;
uint8_t numofstring; 
uint8_t WIFI_CONNECT_STATUS = FALSE;
uint8_t sntp_init_status =FALSE;
int32_t rssi = 0;
// inv_flash_data_t inv_data;
uint8_t sync_done = FALSE;


extern void init_datalogger();
extern void get_sntp_conf(sntp_conf_t *p);
extern void get_ppp_conf(ppp_conf_t *p);
extern void get_mqtt_conf(mqtt_conf_t *p);
extern void get_dev_data(dev_data_t *p);
extern void flash_read(char *buff ,flash_sector_t sector);
extern void flash_write(flash_sector_t sector, char *str);

extern esp_err_t send_system_data(char *msg);
extern esp_err_t send_config_status(char *msg);
uint8_t count;
extern void init_ota(void);
esp_err_t _http_event_handler(esp_http_client_event_t *evt);

// MQTT details

#define MQTT_PREFIX         "RMS"
#define MQTT_SUB_TOPIC      "CONFIG"
#define MQTT_PUB_DATA       "POST"
// #define MQTT_PUB_DATA       "SHS"
#define MQTT_PUB_STATUS     "STATUS"
#define MQTT_CONFIG_STATUS     "CSTATUS"
#define MQTT_QOS            1
#define MQTT_MSG_RETAIN     0       // This is useful if broker needs to keep a copy of the last received message
#define MQTT_MSG_STORE      true    // This is ignored in case of QOS > 0

#define MQTT_DISABLE_AUTO_RECON false

char mqttTopic[30];
char mqttSubTopic[30];
char dev_mac[30];
char sm_ssid[33] = { 0 };
char sm_password[65] = { 0 };
char macid[20];
esp_mqtt_client_handle_t mqtt_client;
bool wifi_monitor_bit=false;

int disconnect_count=0;

uint8_t mqtt_status = WAITING;
uint8_t MQ_status =DISCONNECTED;

// #define EXAMPLE_ESP_WIFI_SSID      "Orb_Guest"
// #define EXAMPLE_ESP_WIFI_PASS      "a1@b2@c3$d4$e5"

#define EXAMPLE_ESP_WIFI_SSID      "myssid"
#define EXAMPLE_ESP_WIFI_PASS      "mypassword"

#define EXAMPLE_ESP_MAXIMUM_RETRY  5
extern unsigned int gframe, mframe,year, month, date, hour, minute,second; //Frame varaible for giving command
extern char serialno[50], mload, mresult, SIM_ID[30],IMEI[22], date_time[30], version[5],mresult1;
extern unsigned int systemcount, supply_voltage, inv_switch, maxsyscount; //
extern double bvol,fault_code,acpower, pv1vol, pv2vol, pvdcw, acvolt1, acvolt2, acvolt3, acvoltage, accurrent, watthours, wattage, accur1, accur2, accur3, pv1cur, pv2cur, frequency, invtemp, ambtemp, maxtemp, runtime;
extern char software_version[10];
extern unsigned int  adcresult, inverror, invwar, invfalt, fanfalt, strnfalt,status,id;
extern long double life_energy, today_energy;	
extern char sigstrnt[20];

int led_count = 0;

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t smart_wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;
uint8_t  smartconfig_mon_bit = WAIT;
// extern void init_uart();
extern esp_err_t modbus_master_init(void);
extern void set_led_indication(led_indicator_t);
extern esp_err_t send_ACK(char *msg);
void  init_sntp_service();
void get_device_mac_id(macid);


static void smartconfig_example_task(void * parm);
static void smart_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;
void https_with_url(char *msg);

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        printf("Connected to Wi-Fi.11\n");
        wifi_ap_record_t ap_info;
        while (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        // Get RSSI
        rssi = ap_info.rssi;
        ESP_LOGI(TAG, "RSSI: %d", rssi);
        wifi_monitor_bit=true;
        WIFI_CONNECT_STATUS = TRUE; 
        get_device_mac_id(macid);
        sprintf(dev_mac,"%s",macid);
        gpio_set_level(LED12,LOW);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        printf("Disconnected from hardcoded Wi-Fi.\n");
        // You can add error handling or retry logic here.
        wifi_monitor_bit=false;
        smartconfig_mon_bit = FALSE;
        disconnect_count++;
        WIFI_CONNECT_STATUS = FALSE; 
        gpio_set_level(LED12,HIGH);
        if (disconnect_count == 20){
            disconnect_count=0;
            printf("restarting because of wifi disconnection\n");
            esp_restart();
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG1, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        wifi_monitor_bit=true;
    }
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG5, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG5, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG5, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG5, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG5, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                if (evt->user_data) {
                    memcpy(evt->user_data + output_len, evt->data, evt->data_len);
                } else {
                    if (output_buffer == NULL) {
                        output_buffer = (char *) malloc(esp_http_client_get_content_length(evt->client));
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE(TAG5, "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    memcpy(output_buffer + output_len, evt->data, evt->data_len);
                }
                output_len += evt->data_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG5, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG5, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI(TAG5, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG5, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG5, "HTTP_EVENT_REDIRECT");
            esp_http_client_set_header(evt->client, "From", "user@example.com");
            esp_http_client_set_header(evt->client, "Accept", "text/html");
            esp_http_client_set_redirection(evt->client);
            break;
    }
    return ESP_OK;
}

void wifi_init_sta(void)
{
    printf("SmartConfig failed. Connecting to hardcoded Wi-Fi...\n");
    // Configure and connect to the hardcoded Wi-Fi network
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        }
    };
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());
    // Register a WiFi event handler to monitor connection status
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
}


network_msg_handler_t network_msg_handler = NULL;

void register_network_msg_handler (network_msg_handler_t hanlder) {
    network_msg_handler = hanlder;
}

void send_boot_complete() {
    char msg[200];
    time_t now;
    dev_data_t dev_data;

    get_dev_data(&dev_data);
    time(&now);
    now = now - esp_timer_get_time() / 1000000u; // Adjust the start time by subtracting time since boot
    sprintf(msg, "{\"ts\":%llu, \"model\": %s, \"serial\": %s, \"firmware\": %s, \"boot reason\": %s}",
        now, dev_data.model, dev_data.serial, dev_data.firmware, boot_reason_str[boot_reason]);
    
    send_system_data(msg);
    printf("\nBoot message: %s", msg);
}



void get_device_mac_id(char *id){
    uint8_t tmp[10];
	ESP_ERROR_CHECK(esp_read_mac(tmp, ESP_MAC_WIFI_STA));
	sprintf(id, MACSTR, MAC2STR(tmp));
    id=strupr(id);
}

void sntp_sync_callback(struct timeval *tv){
    ESP_LOGI(TAG3, "SNTP sync done: %llu", tv->tv_sec);
    sync_done = TRUE;
    if (boardReady && !sntpInitDone) {
        printf("board ready\n");
        // send_boot_complete(); // first time sntp sync done; send boot message
        sntpInitDone = true;
    }
}

// Setup SNTP service so that the time is updated at default interval (1 hour).
// Once initialized, this service runs in the background and there is no need to (re)initialize sntp.
// This service can be replaced with esp_netif_sntp when the ESP-IDF version is changed to 5.1+.
void init_sntp_service() {
    sntp_conf_t sntp_conf;
    get_sntp_conf(&sntp_conf);
    esp_sntp_setservername(0, sntp_conf.url);
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_set_sync_interval(sntp_conf.interval);
    sntp_set_time_sync_notification_cb(sntp_sync_callback);
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
    esp_sntp_init();
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG6, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG6, "MQTT_EVENT_CONNECTED");
        mqtt_status = CONNECTED;
        MQ_status = CONNECTED;
        msg_id = esp_mqtt_client_subscribe(client, mqttSubTopic, 0);
        ESP_LOGI(TAG6, "Sent subscribe successful, msg_id=%d  mqttSubTopic = %s\n", msg_id,mqttSubTopic);
        xEventGroupSetBits(event_group, MQTT_CONNECT_BIT);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG6, "MQTT_EVENT_DISCONNECTED");
        mqtt_status = DISCONNECTED;
        // esp_mqtt_client_reconnect(mqtt_client);
        count++;
        if(count==20)
        {
            count = 0;
            printf("\nrestartingggggggggggg\n");
            esp_restart();
        }
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG6, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    // case MQTT_EVENT_UNSUBSCRIBED:
    //     ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    //     break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG6, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        mqtt_status = PUBLISHED;
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG6, "MQTT_EVENT_DATA");
        (*(network_msg_handler))(event->data, event->data_len, event->topic, event->topic_len);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG6, "MQTT_EVENT_ERROR");
        break;
    case MQTT_EVENT_DELETED:
        ESP_LOGI(TAG6, "MQTT_EVENT_DELETED, msg_id=%d", event->msg_id);
        break;
    default:
        ESP_LOGI(TAG6, "MQTT other event id: %d", event->event_id);
        break;
    }
}

void init_mqtt () {
    printf("-----------------mqtt init------------------------\n");
    char macid[20];

    mqtt_conf_t mqtt_conf;

    get_mqtt_conf(&mqtt_conf);

	get_device_mac_id(macid);
    sprintf(dev_mac,"%s",macid);
    sprintf(mqttTopic, "%s/%s", MQTT_PREFIX, macid);
    sprintf(mqttSubTopic, "%s/%s", mqttTopic, MQTT_SUB_TOPIC);
    
    /* Config MQTT */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    // esp_mqtt_client_config_t mqtt_config = {
    //     .broker.address.uri = "mqtt://mqtt.thingsboard.cloud:1883",
    //     .credentials.client_id="orbenergy",
    //     .credentials.username = "orb123",
    //     .credentials.authentication.password = "1234567890",
    //     .network.reconnect_timeout_ms = 1000,
    //     .network.disable_auto_reconnect = MQTT_DISABLE_AUTO_RECON,
    //     .session.keepalive = 1
    // };
        esp_mqtt_client_config_t mqtt_config = {
        .broker.address.uri = "mqtt://13.60.190.6:1883",
        // .credentials.client_id="",
        .credentials.username = "adminuser",
        .credentials.authentication.password = "password@123",
        .network.reconnect_timeout_ms = 1000,
        .network.disable_auto_reconnect = MQTT_DISABLE_AUTO_RECON,
        .session.keepalive = 15
    };


#else
    esp_mqtt_client_config_t mqtt_config = {
        .uri = CONFIG_EXAMPLE_MQTT_BROKER_URI,
    };
#endif

    xEventGroupClearBits(event_group, MQTT_CONNECT_BIT);
    mqtt_client = esp_mqtt_client_init(&mqtt_config);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    /* Wait for MQTT connection */
    ESP_LOGI(TAG6, "Waiting for MQTT connection");
    xEventGroupWaitBits(event_group, MQTT_CONNECT_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}


static void smart_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(smart_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(smart_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG2, "Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG2, "Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG2, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        char ssid[33] = { 0 };
        char password[65] = { 0 };
        uint8_t rvd_data[33] = { 0 };
        
        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        memcpy(sm_ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(sm_password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG2, "SSID:%s", sm_ssid);
        ESP_LOGI(TAG2, "PASSWORD:%s", sm_password);
        // flash_write(WIFI_SSID_ADDR,ssid);
        // flash_write(WIFI_PASSWORD_ADDR,password);
        if (evt->type == SC_TYPE_ESPTOUCH_V2) {
            ESP_ERROR_CHECK( esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)) );
            ESP_LOGI(TAG2, "RVD_DATA:");
            for (int i=0; i<33; i++) {
                printf("%02x ", rvd_data[i]);
            }
            printf("\n");
        }
        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
        esp_wifi_connect();
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        flash_write(WIFI_SSID_ADDR,sm_ssid);
        flash_write(WIFI_PASSWORD_ADDR,sm_password);
        xEventGroupSetBits(smart_wifi_event_group, ESPTOUCH_DONE_BIT);
        printf("---------------------done bit\n");
        smartconfig_mon_bit = TRUE;
    }
}

static void initialise_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    smart_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &smart_event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &smart_event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &smart_event_handler, NULL) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void smartconfig_example_task(void * parm)
{
    EventBits_t uxBits;
    static TickType_t smartconfig_start_time;
    smartconfig_start_time = xTaskGetTickCount();
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
    while (1) {
        uxBits = xEventGroupWaitBits(smart_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, pdMS_TO_TICKS(1000));
        // Check if the CONNECTED_BIT is set
        if (uxBits & CONNECTED_BIT) {
            ESP_LOGI(TAG2, "WiFi Connected to ap");
        }
        // Check if the ESPTOUCH_DONE_BIT is set
        if (uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG2, "smartconfig over");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
        // Check if 1 minute has elapsed since SmartConfig started
        if (xTaskGetTickCount() - smartconfig_start_time >= pdMS_TO_TICKS(60000)) {
            ESP_LOGI(TAG2, "1 minute timeout reached, stopping SmartConfig");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

void factory_flash_config(){
    char flash_buff[20]={"\0"};
    flash_read(flash_buff,FIRST_BOOT_ADDR);
    if(strcmp(flash_buff,"2ndflash")!=0)
    {
		flash_write(FACTORY_PERVIOUS_FIRMWARE_VERSION_ADDR,"v0" );
		flash_write(FACTORY_CURRENT_FIRMWARE_VERSION_ADDR,"0");
		flash_write( FACTORY_FACTORY_FIRMWARE_VERSION_ADDR,"v1");
        flash_write(INVERTER_TYPE_ADDR,"GOODWE");
        flash_write(NUMBER_OF_INVERTER_ADDR,"5");
        flash_write(NUMBER_OF_STRING_ADDR,"8");
        flash_write( FIRST_BOOT_ADDR,"2ndflash");
    }
}

// Handle all hardware related initializations (except modem)


void non_smartconfig(){
    _wifi_conf_t wifi_conf;
    char myssid[33] ={"\0"};
    char mypass[65] ={"\0"};    
    // flash_read(wifi_conf.ssid,WIFI_SSID_ADDR);
    flash_read(myssid,WIFI_SSID_ADDR);
    // printf("SSID = %s     \n",myssid);
    // if(wifi_conf.ssid != 0 ) {
    // printf("SSID = %s     \n",wifi_conf.ssid);
    // }
    // flash_read(wifi_conf.password,WIFI_PASSWORD_ADDR);
    // printf("PASSWORD = %s     \n",wifi_conf.password);
    flash_read(mypass,WIFI_PASSWORD_ADDR);
    // printf("PASSWORD = %s     \n",mypass);
    wifi_config_t wifi_config;        
    bzero(&wifi_config, sizeof(wifi_config_t));
    // memcpy(wifi_config.sta.ssid, wifi_conf.ssid, sizeof(wifi_config.sta.ssid));
    // memcpy(wifi_config.sta.password, wifi_conf.password, sizeof(wifi_config.sta.password));
    memcpy(wifi_config.sta.ssid, myssid, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, mypass, sizeof(wifi_config.sta.password));
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    esp_wifi_connect();
        // Register a WiFi event handler to monitor connection status
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
}

void https_with_url(char *msg)
{
    esp_http_client_config_t config = {
        // .url = "https://52.37.194.44/api/consume-json?_format=json",
        .url = "https://rms.orbenergy.com/api/consume-json?_format=json",
        .event_handler = _http_event_handler,
        .port = 443,
        // .crt_bundle_attach = esp_crt_bundle_attach,
         .cert_pem = NULL,             // No certificate provided
        .skip_cert_common_name_check = true, // Skip common name verification
        .transport_type = HTTP_TRANSPORT_OVER_SSL, // Specify SSL transport        
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    // esp_err_t err = esp_http_client_perform(client);
    // const char *post_data ="{\"RDate\":\"13-8-2024 12:20\",\"RVolt\":\"16\",\"RIMAID\":\"esp112\",\"RSignal\":\"33\",\"ISth\":\"0\",\"ID\":\"0\",\"InvSlno\":\"0123456789venutest4\",\"MPPT1_DCV\":\"0\",\"MPPT1_DCA\":\"0\",\"MPPT2_DCV\":\"0\",\"MPPT2_DCA\":\"0\",\"Ph1ACV\":\"0\",\"Ph1ACA\":\"0\",\"Ph2ACV\":\"0\",\"Ph2ACA\":\"0\",\"Ph3ACV\":\"0\",\"Ph3ACA\":\"0\",\"InvAC_P\":\"0\",\"Inv_Fr\":\"0\",\"Inv_Wh\":\"0\",\"Inv_Nrg\":\"0\",\"Inv_Rt\":\"0\",\"Inv_LE\":\"0\",\"Inv_Sts\":\"0\",\"Inv_Tpr\":\"0\",\"Inv_Err\":\"0\",\"Inv_Wrn\":\"0\",\"Inv_Fnflt\":\"0\",\"Inv_StFlt\":\"0\"}" ;//"{\"field1\":\"value1\"}";
    // esp_http_client_set_url(client, "https://52.37.194.44/api/consume-json?_format=json");
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, msg, strlen(msg));
    esp_err_t err = esp_http_client_perform(client);
    printf("https msg data = %s \n",msg);

    if (err == ESP_OK) {
        ESP_LOGI(TAG5, "HTTPS Status = %d, content_length = %lld \n data posted successfully\n",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));

    } else {
        ESP_LOGE(TAG5, "Error perform http request %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

// static void http_test_task(void *pvParameters)
// {
//     // https_with_url();
//     ESP_LOGI(TAG, "Finish http example");
//     vTaskDelete(NULL);
// }

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    // char sw_data[2000];
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if (gpio_get_level(SW1) == 1){
                printf("switch presed\n");
                esp_restart();
            }
        }
    }
}

void green_led_action_callback(){
    if(WIFI_CONNECT_STATUS == FALSE){
        led_count ++;
        if((led_count % 2)!=0){
            gpio_set_level(LED12, LOW);
            ESP_ERROR_CHECK(esp_timer_stop(green_led_action_timer));
            ESP_ERROR_CHECK(esp_timer_start_periodic(green_led_action_timer,LED_INDICATION_TIMER_COUNT));
        }
        else{
            gpio_set_level(LED12, HIGH);
            ESP_ERROR_CHECK(esp_timer_stop(green_led_action_timer));
            ESP_ERROR_CHECK(esp_timer_start_periodic(green_led_action_timer,LED_INDICATION_TIMER_COUNT));
        }
    }
    if(WIFI_CONNECT_STATUS == TRUE){
        // count = 0;
        gpio_set_level(LED12, LOW);
        ESP_ERROR_CHECK(esp_timer_stop(green_led_action_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(green_led_action_timer,LED_TEST_TIMER_COUNT));
    }
}
void led_timer_init(){
    const esp_timer_create_args_t green_led_action= {
		.callback = &green_led_action_callback,
		/* name is optional, but may help identify the timer when debugging */
		.name = "amber_led_action"
	};
	ESP_ERROR_CHECK(esp_timer_create(&green_led_action, &green_led_action_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(green_led_action_timer,LED_INDICATION_TIMER_COUNT));
}
void init_gpio(){
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_POSEDGE; /***interrupt of any edge***/
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;	// gpio bit masking
    io_conf.mode = GPIO_MODE_INPUT;	//set as input mode
    io_conf.pull_up_en = HIGH;	//enable pull-up mode
    gpio_config(&io_conf);
    gpio_reset_pin(SW1);
    gpio_set_direction(SW1,GPIO_MODE_INPUT);

    gpio_reset_pin(LED12);
    gpio_set_direction(LED12,GPIO_MODE_OUTPUT);
    gpio_set_level(LED12,HIGH);

    gpio_set_intr_type(SW1, GPIO_INTR_ANYEDGE);
}

void bootup_init(){
    char flash_buff[20]={"\0"};
    char flash_buff1[20]={"\0"};
    char flash_buff2[20]={"\0"};
	init_gpio();
    led_timer_init();
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 4096, NULL, 10, NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(SW1, gpio_isr_handler, (void*) SW1);
    ESP_ERROR_CHECK(modbus_master_init());
    // init_uart();
    // ESP_ERROR_CHECK(nvs_flash_init());
    // factory_flash_config();
    // flash_read(flash_buff,INVERTER_TYPE_ADDR);
    // sprintf(inverter_type,"%s",flash_buff);
    // printf("inverter_type=%s\n",inverter_type);
    // flash_read(flash_buff1,NUMBER_OF_INVERTER_ADDR);
    // sscanf(flash_buff1,"%u",&numOfGoodweInverter);
    // printf("numofinverter=%u\n",numOfGoodweInverter);
    // flash_read(flash_buff2,NUMBER_OF_STRING_ADDR);
    // sscanf(flash_buff2,"%u",&numofstring);
    // printf("numofstring=%u\n",numofstring);
    init_datalogger();
}
// main code
void app_main(void)
{
    // esp_err_t err;
    boot_reason = esp_reset_reason();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    bootup_init(); // All board initializations
    event_group = xEventGroupCreate();
    // flash_write(WIFI_SSID_ADDR,"Venu");
    // flash_write(WIFI_PASSWORD_ADDR,"12345678");
    initialise_wifi();
    vTaskDelay(60000 / portTICK_PERIOD_MS);  
    if(smartconfig_mon_bit == FALSE || smartconfig_mon_bit == WAIT)
    {
        printf("------- trying to connect previously connected wifi -------\n");
        non_smartconfig();
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        if( wifi_monitor_bit == false){
            printf("------- trying to connect hard coded user id and password -------\n");
            wifi_init_sta();
            // vTaskDelay(30000 / portTICK_PERIOD_MS);
        }
    }
    // if(WIFI_CONNECT_STATUS == TRUE){
        printf("---------------sntp init-------------\n");
        init_sntp_service();    // Start sntp service to sync time with NTP
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        sntp_init_status =TRUE;
        if( sync_done == TRUE ){
            time_t now;
            time(&now); // Get the current time in seconds since epoch
            struct tm *timeInfo = gmtime(&now);// Convert the current time to UTC time
            int totalOffsetMinutes = TIMEZONE_OFFSET_HOURS * 60 + TIMEZONE_OFFSET_MINUTES; // Add the time zone offset in minutes
            timeInfo->tm_min += totalOffsetMinutes; // Add minutes offset
            mktime(timeInfo); // Normalize the tm structure
            datetime.hours = timeInfo->tm_hour;
            datetime.mins = timeInfo->tm_min;
            datetime.secs = timeInfo->tm_sec;
            datetime.day = timeInfo->tm_mday;
            datetime.month = timeInfo->tm_mon+1;
            datetime.year = timeInfo->tm_year+1900;
            ESP_LOGI(TAG3, "DATE time = %d-%d-%d   %d:%d:%d",datetime.day,datetime.month,datetime.year,datetime.hours,datetime.mins,datetime.secs);
            // struct tm *timeinfo;
            // char buffer[20];
            // time(&now);
            // setenv("TZ", "IST-5:30", 1);
            // tzset();
            // timeinfo=localtime(&now);
            // // strftime(buffer, sizeof(buffer), "%d/%m/%Y %I:%M:%S", timeinfo); //FOR 12 HOUR FORMAT
            // strftime(buffer, sizeof(buffer), "%d/%m/%Y %H:%M:%S", timeinfo);       
        }
    // }        
    init_mqtt();
    // xTaskCreate(&http_test_task, "http_test_task", 8192, NULL, 5, NULL);
    boardReady = true;
    printf("sntp...start..............9\n");
    init_ota();
}

esp_err_t send_meas_data(char *msg) {
    int ret = ESP_ERR_INVALID_STATE;
    char mqttPubTopic[30];
    if (mqtt_status == DISCONNECTED){
        esp_mqtt_client_reconnect(mqtt_client);
    }
    if (boardReady) {
        sprintf(mqttPubTopic, "%s/%s", mqttTopic, MQTT_PUB_DATA);
        ret = esp_mqtt_client_enqueue(mqtt_client, mqttPubTopic, msg, 0, MQTT_QOS, MQTT_MSG_RETAIN, MQTT_MSG_STORE);
        // sprintf(msg,"{\"RDate\":\"%u-%u-%u %u:%u\",\"RVolt\":\"%g\",\"RIMAID\":\"%s\",\"RSignal\":\"%s\",\"ISth\":\"%u\",\"ID\":\"%u\",\"InvSlno\":\"0%s123456789\",\"MPPT1_DCV\":\"%.2f\",\"MPPT1_DCA\":\"%.2f\",\"MPPT2_DCV\":\"%.2f\",\"MPPT2_DCA\":\"%.2f\",\"Ph1ACV\":\"%.2f\",\"Ph1ACA\":\"%.2f\",\"Ph2ACV\":\"%.2f\",\"Ph2ACA\":\"%.2f\",\"Ph3ACV\":\"%.2f\",\"Ph3ACA\":\"%.2f\",\"InvAC_P\":\"%.2f\",\"Inv_Fr\":\"%.2f\",\"Inv_Wh\":\"%.2f\",\"Inv_Nrg\":\"%.2Lf\",\"Inv_Rt\":\"%.2f\",\"Inv_LE\":\"%.2Lf\",\"Inv_Sts\":\"%u\",\"Inv_Tpr\":\"%.2f\",\"Inv_Err\":\"%u\",\"Inv_Wrn\":\"%u\",\"Inv_Fnflt\":\"%s\",\"Inv_StFlt\":\"%s\"}", date, month, year, hour, minute, bvol, IMEI, sigstrnt, inv_switch, id, serialno, pv1vol, pv1cur, pv2vol, pv2cur, acvolt1, accur1, acvolt2, accur2, acvolt3, accur3, acpower, frequency, watthours, today_energy, runtime, life_energy, status, invtemp,inverror, invwar, software_version, SIM_ID);
        // ret = esp_mqtt_client_publish(mqtt_client, "v1/devices/me/telemetry", msg, 0, MQTT_QOS, MQTT_MSG_RETAIN);
        printf("ret value = 0x%X   data =%s",ret,msg);
        if(ret == ESP_OK){
            ESP_LOGI(TAG6, "MQTT_EVENT_PUBLISHED");
            // set_led_indication(GREEN_LED_ON);
        }
        if (ret == ESP_FAIL) {
            ESP_LOGI(TAG6, "Message not enqued -> dropped!");
        }
    }
    return ret;
}

esp_err_t send_ACK(char *msg){
    int ret = ESP_ERR_INVALID_STATE;
    char mqttPubTopic[30];
    if (mqtt_status == DISCONNECTED){
        esp_mqtt_client_reconnect(mqtt_client);
    }
    if (boardReady) {
        sprintf(mqttPubTopic, "%s/%s", mqttTopic, "ACK");
        // ret = esp_mqtt_client_enqueue(mqtt_client, mqttPubTopic, msg, 0, MQTT_QOS, MQTT_MSG_RETAIN, MQTT_MSG_STORE);
        // sprintf(msg,"{\"RDate\":\"%u-%u-%u %u:%u\",\"RVolt\":\"%g\",\"RIMAID\":\"%s\",\"RSignal\":\"%s\",\"ISth\":\"%u\",\"ID\":\"%u\",\"InvSlno\":\"0%s123456789\",\"MPPT1_DCV\":\"%.2f\",\"MPPT1_DCA\":\"%.2f\",\"MPPT2_DCV\":\"%.2f\",\"MPPT2_DCA\":\"%.2f\",\"Ph1ACV\":\"%.2f\",\"Ph1ACA\":\"%.2f\",\"Ph2ACV\":\"%.2f\",\"Ph2ACA\":\"%.2f\",\"Ph3ACV\":\"%.2f\",\"Ph3ACA\":\"%.2f\",\"InvAC_P\":\"%.2f\",\"Inv_Fr\":\"%.2f\",\"Inv_Wh\":\"%.2f\",\"Inv_Nrg\":\"%.2Lf\",\"Inv_Rt\":\"%.2f\",\"Inv_LE\":\"%.2Lf\",\"Inv_Sts\":\"%u\",\"Inv_Tpr\":\"%.2f\",\"Inv_Err\":\"%u\",\"Inv_Wrn\":\"%u\",\"Inv_Fnflt\":\"%s\",\"Inv_StFlt\":\"%s\"}", date, month, year, hour, minute, bvol, IMEI, sigstrnt, inv_switch, id, serialno, pv1vol, pv1cur, pv2vol, pv2cur, acvolt1, accur1, acvolt2, accur2, acvolt3, accur3, acpower, frequency, watthours, today_energy, runtime, life_energy, status, invtemp,inverror, invwar, software_version, SIM_ID);
        ret = esp_mqtt_client_publish(mqtt_client, mqttPubTopic, msg, 0, MQTT_QOS, MQTT_MSG_RETAIN);
        printf("ret value = 0x%X   data =%s\n",ret,msg);
        if(ret == ESP_OK){
            ESP_LOGI(TAG6, "MQTT_EVENT_PUBLISHED");
            // set_led_indication(GREEN_LED_ON);
        }
        if (ret == ESP_FAIL) {
            ESP_LOGI(TAG6, "Message not enqued -> dropped!");
        }
    }
    return ret;
}

// esp_err_t send_system_data(char *msg) {
//     int ret = ESP_ERR_INVALID_STATE;
//     char mqttPubTopic[30];
    
//     if (boardReady) {
//         sprintf(mqttPubTopic, "%s/%s", mqttTopic, MQTT_PUB_STATUS);
//         printf("system health topic1:%s\n",mqttPubTopic);
//         // ret = esp_mqtt_client_enqueue(mqtt_client, mqttPubTopic, msg, 0, MQTT_QOS, MQTT_MSG_RETAIN, MQTT_MSG_STORE);
//         ret = esp_mqtt_client_publish(mqtt_client, "/test1", msg, 0, MQTT_QOS, MQTT_MSG_RETAIN);
//         printf("system health topic:%s\n",mqttPubTopic);
//         if (ret == ESP_FAIL) {
//             ESP_LOGI(TAG, "Message not enqued -> dropped!");
//         }
//     }
//     return ret;
// }
