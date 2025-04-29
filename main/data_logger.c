/* This file specific to the project. It is responsible for handling all project specific data aggregation,
   communication with the network, managing calibrations, etc.
*/
#include "rom/gpio.h"
#include "driver/gpio.h"
#include <cJSON.h>
#include "esp_sntp.h"
#include "Defines.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "esp_sleep.h"

#define INTERVAL 1000*60*1
#define SLEEP_DURATION_SEC 36000  // Sleep duration in seconds (e.g., 1 minutes)


extern date_time_t datetime;
extern uint8_t sync_done;
static const char *TAG = "LOGGER";
static const char *TAG1 = "DEEP_SLEEP";
#define tag "SSD1306"
extern esp_err_t  send_meas_data(char *msg);
char data1[8196];
TaskHandle_t   invTask_h;
TaskHandle_t 	loggerTask_h;
QueueHandle_t 	loggerQueue_h;
extern QueueHandle_t 	invQueue_h;
extern uint8_t mqtt_status;
extern esp_mqtt_client_handle_t mqtt_client;
extern uint8_t WIFI_CONNECT_STATUS;

#define DATA_TIMER_COUNT 1000000*30
#define POSTING_TIMER_COUNT 1000000*10 //1000000*60*12
esp_timer_handle_t data_collect_timer;

extern esp_err_t send_system_data(char *msg);
extern void register_comm_status_handler(comm_link_status_handler_t handler);
extern void register_network_msg_handler (network_msg_handler_t hanlder);
extern esp_err_t write_config_data(flash_sector_t sector, char *str);
extern esp_err_t set_serial_number (char *str);
int32_t seq = 0;
extern void inv_task();
extern void flash_read(char *buff ,flash_sector_t sector);
extern void flash_write(flash_sector_t sector, char *str);
extern void https_with_url(char *msg);
extern esp_err_t send_meas_data1(char *msg);
extern void init_sntp_service();
extern uint8_t MQ_status;
extern uint8_t sntp_init_status;
int call_bk=0;

extern uint8_t ems_read_status;

uint8_t callback_status = FALSE;
char contro_string[10];
uint8_t inv_type = FALSE;

void inverter_control(char *INV_CONTROL);

void logger_task() {
	interTaskMsg_t msg;
	loggerQueue_h = xQueueCreate(20, sizeof(interTaskMsg_t));
	if (loggerQueue_h == NULL) {
		ESP_LOGE(TAG, "Unable to create primary queue.");
		return;
	}
	for (;;) {
		if (xQueueReceive(loggerQueue_h, &msg, pdMS_TO_TICKS(10000)))
		{
			// printf("meas data-------------------------------------\n");
			time_t now;
			seq++;
			time(&now);
			struct tm *timeInfo = gmtime(&now);
			// Add the time zone offset in minutes
			int totalOffsetMinutes = TIMEZONE_OFFSET_HOURS * 60 + TIMEZONE_OFFSET_MINUTES;
			timeInfo->tm_min += totalOffsetMinutes; // Add minutes offset
			mktime(timeInfo); // Normalize the tm structure
			datetime.hours = timeInfo->tm_hour;
			datetime.mins = timeInfo->tm_min;
			datetime.secs = timeInfo->tm_sec;
			datetime.day = timeInfo->tm_mday;
			datetime.month = timeInfo->tm_mon+1;
			datetime.year = timeInfo->tm_year+1900;
			ESP_LOGI(TAG, "DATE time = %u-%u-%u %u:%u",datetime.day,datetime.month,datetime.year,datetime.hours,datetime.mins);
			sprintf(data1, "{\"RDate\":\"%u-%u-%u %u:%u:%u\"",datetime.day,datetime.month,datetime.year,datetime.hours,datetime.mins,datetime.secs);
			strcat(data1, ",");
			strcat(data1,msg.data);
			strcat(data1, "}");
			#if (EMS)
				if( sync_done == TRUE){
					send_meas_data1(data1);
				}
			#else
			if( sync_done == TRUE && (datetime.hours > 5 && datetime.hours < 19)){// sending data based on time
				https_with_url(data1);
			}
			#endif
		}
	}
	vQueueDelete(loggerQueue_h);
}

void on_network_msg (char *msg, int msg_len, char *topic, int topic_len) {
	cJSON *json = cJSON_Parse(msg);
	net_msg_type_t id;
	interTaskMsg_t proc_msg;
	// printf("mqtt msg = %s\n",msg);
	if(strstr(msg,"RESETRMS")){
		esp_restart();
	}
	if (json) {
		char *tmp = malloc(topic_len+1);
		strncpy(tmp, topic, topic_len);
		tmp[topic_len] = 0;		
		char *msgId = strrchr(tmp, '/');		
		if (msgId != NULL) {
			msgId = msgId + 1;
			if (strcmp(msgId, "/CONFIG") == 0) {
				id = CALIBRATION;
			}
			proc_msg.data = malloc(msg_len);
			strncpy(proc_msg.data, msg, msg_len);
			switch (id) {
				case CALIBRATION:
					proc_msg.msgId = DATA_ACQ_CALIBRATION;
					if (write_config_data(CALIBRATION_ADDR, proc_msg.data) != ESP_OK) {
						ESP_LOGE(TAG, "Calibration could not be saved!");
					}
					else{
						printf("sussefully stored in flash memory\n");
						cJSON *name = cJSON_GetObjectItem(json,"SET");
						if (cJSON_IsString(name) && (name->valuestring != NULL)) {
							printf("Name: %s\n", name->valuestring);
							strcpy(contro_string,name->valuestring);
							inverter_control(contro_string);
						}
					}
					free(proc_msg.data);
					break;
				default:
					ESP_LOGI(TAG, "Message topic (%s) not recognized!", tmp);
					break;
			}
		}
		else {
			ESP_LOGI(TAG, "Error in reading message id or there is no id field!");
			cJSON_Delete(json);
			return;
		}
	}
	else {
		ESP_LOGI(TAG, "Error in reading network message or the message is not in json format!");
		return;
	}
}

void collect_data_callback(){
	interTaskMsg_t msg;
	char data[70];
	msg.msgId = INV_DATA;
    msg.data = data;	
	call_bk ++;
	printf("callback_collect =%d\n ",call_bk);
	if(MQ_status == DISCONNECTED && mqtt_status == DISCONNECTED ){
		printf("mqtt disconnected................................\n");
		esp_mqtt_client_reconnect(mqtt_client);
	}
	if(MQ_status == CONNECTED ){
		printf("mqtt connected................................\n");
		call_bk =0;
	}	
	if (WIFI_CONNECT_STATUS == TRUE && sync_done == TRUE && callback_status == TRUE){
		printf("https connection\n");
		call_bk =0;
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
		ESP_LOGI(TAG, "DATE time = %d-%d-%d   %d:%d:%d",datetime.day,datetime.month,datetime.year,datetime.hours,datetime.mins,datetime.secs);
		if(datetime.hours >= 5 && datetime.hours <19){
			xQueueSend(invQueue_h, (void *) &msg, 0);
			ESP_ERROR_CHECK(esp_timer_stop(data_collect_timer));
			ESP_ERROR_CHECK(esp_timer_start_periodic(data_collect_timer,POSTING_TIMER_COUNT));
			callback_status = FALSE;
		}
		else{
			esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SEC * 1000000ULL);  // Convert seconds to microseconds
		// Log and enter deep sleep
			ESP_LOGI(TAG1, "Going to deep sleep for %d seconds...", SLEEP_DURATION_SEC);
			esp_deep_sleep_start();
		}
	}
	if(sync_done == FALSE && sntp_init_status == TRUE){
		printf("loger sntp init ------------------------------\n");
		esp_sntp_stop();
		init_sntp_service();
	}
}
void init_common_timer(){
	callback_status = TRUE;
	const esp_timer_create_args_t data_collect= {
						.callback = collect_data_callback,
						/* name is optional, but may help identify the timer when debugging */
						.name = "collect_data_callback"
	};
	ESP_ERROR_CHECK(esp_timer_create(&data_collect, &data_collect_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(data_collect_timer,DATA_TIMER_COUNT));
}

void init_datalogger() {
	xTaskCreate(logger_task, "Main logger task", LOGGER_STACK_SIZE, NULL, LOGGER_TASK_PRIORITY, &loggerTask_h);
	xTaskCreate(inv_task, "inv_task", DATA_ACQ_STACK_SIZE, NULL, 1, &invTask_h);
	init_common_timer();
#if (ENABLE_LED)
	init_led();
#endif
	printf("init datalogor------------------------------1\n");
	register_network_msg_handler (&on_network_msg);
}