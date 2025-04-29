#ifndef MAIN_DEFINES_H_
#define MAIN_DEFINES_H_

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include <sys/socket.h>
#include <freertos/queue.h>

// Feature specific defines (enable desired modules)


#define TIMEZONE_OFFSET_HOURS 5
#define TIMEZONE_OFFSET_MINUTES 30

#define ENABLE_LED					0
#define GOODWE                      0
#define SUNGROW                     0
#define SOLIS                       0
// #define DELTA                       0
// #define SMA                         0
#define GROWATT                     0
#define SINENG_ELECTRIC             0
#define HUAWEI                      0
#define SCHNE_CL20_3P               0
#define EMS                         1
#define TESTING                     0




// Internal message passing structure (between tasks)
typedef struct {
	int16_t msgId;
	char *data;
} interTaskMsg_t;
typedef struct {
	int16_t msgId1;
	char *data1;
} interTaskMsg_t1;

// Different message types
#define DATA_ACQ_CALIBRATION		9
#define DEV_CONFIG					15
#define DATA_ACQ_MSG				21
#define INV_DATA					1
#define WAIT                        2


typedef enum{
    Goodwe  =   1,
    Sungrow,      
    Solis,                      
    Delta,
    Sma,
    Growatt,
    Singeg_eletric,
    Huawei,
    Schnider,
    Ems,
}inverter_types_t;

// Abstract status of communication link that can be used to control LED
typedef enum {
	SERVER_CONNECTED = 1,
	SERVER_CONNECTING,
	SERVER_DISCONNECTED,
	NO_NETWORK
} commLinkStatus_t;

typedef void (*comm_link_status_handler_t)(commLinkStatus_t comm_status);

typedef void (*network_msg_handler_t)(char *msg, int msg_len, char *topic, int topic_len);

typedef enum net_msg_type_t {
	CALIBRATION = 1,
	SENSOR_ADDRESS,
	DEVICE_CONFIG,
	SET_SERIAL
} net_msg_type_t;

/*
 * Common Macro
 */
#define ON 												1
#define OFF 											0
#define HIGH											1
#define LOW												0
#define TRUE											1
#define FALSE											0
// #define GPIO_EXPANDER_INT								34
#define SW1                                             5
#define LED12                                           20
#define GPIO_INPUT_PIN_SEL  							((1ULL<<SW1))

/*
 * Task create Macro
 */
#define LOGGER_STACK_SIZE								8192
#define DATA_ACQ_STACK_SIZE								4096
#define RGB_LED_STACK_SIZE								2048
#define LOGGER_TASK_PRIORITY							5
#define DATA_ACQ_TASK_PRIOTIY							8
#define RGB_LED_TASK_PRIORITY							1

#define WAITING											1
#define CONNECTED										2
#define DISCONNECTED									3
#define PUBLISHED                                       4


/* LED module related macros */
// LED status
typedef enum {
	LED_BLANK = 0,
	GREEN_LED_ON,
	RED_LED_ON,
	BLUE_LED_ON,
	GREEN_LED_BLINK,
	BLUE_LED_BLINK
} led_indicator_t;



/*
 * Flash Macro
 */
#define FLASH_SECTOR_SIZE								0x1000

#define TIME_ZONE										("UTC-05:30")

#define FLASH_FACTORY_BASE_ADDR							0x6D0000
// Board data - generic - this will include things like, board id, server details, ntp details, ssid, etc.
// #define DEV_CONFIG_ADDR									(FLASH_FACTORY_BASE_ADDR)
// Project specific configuration details - this will include any calibration data
// #define CALIBRATION_ADDR								(FLASH_FACTORY_BASE_ADDR + FLASH_SECTOR_SIZE)
// #define TEMP_SENSOR_ADDR							(CALIBRATION_ADDR + FLASH_SECTOR_SIZE)

typedef enum {
	DEV_CONFIG_ADDR  												= (FLASH_FACTORY_BASE_ADDR),
	CALIBRATION_ADDR												= (FLASH_FACTORY_BASE_ADDR + FLASH_SECTOR_SIZE),
	WIFI_CONFIG_ADDR 												= (CALIBRATION_ADDR + FLASH_SECTOR_SIZE),
	WIFI_SSID_ADDR													= (WIFI_CONFIG_ADDR + FLASH_SECTOR_SIZE),
	WIFI_PASSWORD_ADDR												= (WIFI_SSID_ADDR + FLASH_SECTOR_SIZE),
	FIRST_BOOT_ADDR													= (WIFI_PASSWORD_ADDR + FLASH_SECTOR_SIZE),
	FACTORY_PERVIOUS_FIRMWARE_VERSION_ADDR							= (FIRST_BOOT_ADDR + FLASH_SECTOR_SIZE),
	FACTORY_CURRENT_FIRMWARE_VERSION_ADDR							= (FACTORY_PERVIOUS_FIRMWARE_VERSION_ADDR + FLASH_SECTOR_SIZE),
	FACTORY_FACTORY_FIRMWARE_VERSION_ADDR							= (FACTORY_CURRENT_FIRMWARE_VERSION_ADDR + FLASH_SECTOR_SIZE),
	INVERTER_TYPE_ADDR												= (FACTORY_FACTORY_FIRMWARE_VERSION_ADDR + FLASH_SECTOR_SIZE),
	NUMBER_OF_INVERTER_ADDR											= (INVERTER_TYPE_ADDR + FLASH_SECTOR_SIZE),
	NUMBER_OF_STRING_ADDR											= (NUMBER_OF_INVERTER_ADDR + FLASH_SECTOR_SIZE),
} flash_sector_t;

typedef struct {
	char *url;
	int32_t interval; // in milliseconds
} sntp_conf_t;

typedef	struct {
	char uri[50];
	char client_id[50];
	char username[20];
	char password[20];
	int16_t keepalive;
	int16_t timeout;
	int8_t qos;
} mqtt_conf_t;

typedef struct {
	char ssid[33];
	char password[65];
} _wifi_conf_t;

typedef struct {
	char apn[20];
	char username[20];
	char password[20];
} ppp_conf_t;

typedef struct{
    char setpercent[20];
}inv_conf_t;

typedef struct {
	char ssid[20];
	char username[20];
	char password[20];
} wifi_conf_t;

typedef struct {
	char model[20];
	char serial[20];
	char firmware[20];
} dev_data_t;


typedef struct{
    unsigned int hours;
    unsigned int mins;
    unsigned int secs;
    unsigned int day;
    unsigned int month;
    unsigned int year;
}date_time_t;


typedef enum{
    RUNTIME=1,
    RUN_STATUS,
    WARNING_CODE,
    WORKING_MODE,
    ERRCODE,
    STR_STATUS,
}inv_reg2_t;


typedef enum{
    PH_FREQ=1,
    PH1VOL,
    PH1CURR, 
    PH2VOL,
    PH2CURR, 
    PH3VOL,
    PH3CURR,
    PV1VOL,
    PV1CURR,
    PV2VOL,
    PV2CURR,
    PV3VOL,
    PV3CURR,
    PV4VOL,
    PV4CURR,
    STR1,
    STR2,
    STR3,
    STR4,
    STR5,
    STR6,
    STR7,
    STR8,
    STR9,
    STR10,
    STR11,
    STR12,
    STR13,
    STR14,
    STR15,
    STR16,
    AC_POWER,
    TODAY_ENERGY,
    LIFE_ENERGY,
    INV_TEMP,
    // RUNTIME,
    // RUN_STATUS,
    // WARNING_CODE,
    // WORKING_MODE,
    // ERRCODE,
    // STR_STATUS,
    EMS_R_Y_VOL,
    EMS_Y_B_VOL,
    EMS_B_R_VOL,
    EMS_PF_R,
    EMS_PF_Y,
    EMS_PF_B,
    EMS_PF_T,
    EMS_AP_R,
    EMS_AP_Y,
    EMS_AP_B,
    EMS_AP_T,
    EMS_IMPORT,
    EMS_EXPORT,
    EMS_TOTAL,

} inverter_reg_t;



// typedef struct{
//     float ph_freq;
//     float pv1_vol;
//     float pv1_curr;
//     float pv2_vol;
//     float pv2_curr;
//     float pv3_vol;
//     float pv3_curr;
//     float pv4_vol;
//     float pv4_curr;
//     float ph1_vol;
//     float ph1_curr;
//     float ph2_vol;
//     float ph2_curr;
//     float ph3_vol;
//     float ph3_curr;
//     float str1;
//     float str2;
//     float str3;
//     float str4;
//     float str5;
//     float str6;
//     float str7;
//     float str8;
//     float str9;
//     float str10;
//     float str11;
//     float str12;
//     float str13;
//     float str14;
//     float str15;
//     float str16;
//     float ac_power;
//     float life_energy;
//     float today_energy;
//     float inv_temp;
//     float Rphase_Yphase_V;
//     float Yphase_Bphase_V;
//     float Bphase_Rphase_V;
//     float PF_R;
//     float PF_Y;
//     float PF_B;
//     float TPF;
//     float ACT_P_R;
//     float ACT_P_Y;
//     float ACT_P_B;
//     float TACT_P;
//     float active_import;
//     float active_export;
//     float active_total;
 
// }ReturnType;



#endif /* MAIN_DEFINES_H_ */