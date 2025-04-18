/* This file handles all Flash storage of configuration data, calibration, etc.
   This file is a set of library functions that provide storage, retrieval of information in Flash.
   All configurations must be in json format and each set is stored in a separate sector.
   The individual modules handle their own data (e.g. calibration is handled in data_acquisition task).
   Only the device configuration is decoded in this file.
*/

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_flash.h"
#include <cJSON.h>
#include "esp_app_desc.h"
#include "Defines.h"
#include "esp_sntp.h"



static const char *TAG = "FLASH";
extern esp_err_t send_system_data(char *msg);
extern esp_err_t send_config_status(char *msg);


void flash_write(flash_sector_t sector, char *str){
	esp_err_t err;
	uint16_t len = strlen(str);
	ESP_ERROR_CHECK(esp_flash_erase_region (NULL, sector, FLASH_SECTOR_SIZE));
	vTaskDelay(1000/portTICK_PERIOD_MS);
	ESP_ERROR_CHECK(esp_flash_write(NULL, &len, sector, sizeof(uint16_t)));     	// First two bytes are length of data
	ESP_ERROR_CHECK(esp_flash_write(NULL, str, sector+sizeof(uint16_t), len));     // Rest of the data will be config string
	printf("writing at 0x%X =%s\n",sector+sizeof(uint16_t),str);
	vTaskDelay(1000/portTICK_PERIOD_MS);
}
void flash_read(char *buff , flash_sector_t sector){
	uint16_t len;
	// char *msg;
	ESP_ERROR_CHECK(esp_flash_read(NULL, &len, sector, sizeof(uint16_t)));     	// First two bytes are length of data
	if (len != 0 && len != 0xFFFF) { // the config data is present
		// buff = malloc(len);
		printf("data length =%d\n",len);
		ESP_ERROR_CHECK(esp_flash_read(NULL, buff, sector+sizeof(uint16_t), len));     	// Rest of the data will be config string
		printf("reading data at:0x%X =%s       \n",sector+sizeof(uint16_t),buff);
		// free(msg);
	}
}

esp_err_t write_config_data(flash_sector_t sector, char *str) {
	esp_err_t err;
	uint16_t len = strlen(str);
	ESP_ERROR_CHECK(esp_flash_erase_region (NULL, sector, FLASH_SECTOR_SIZE));
	vTaskDelay(1000/portTICK_PERIOD_MS);
	ESP_ERROR_CHECK(esp_flash_write(NULL, &len, sector, sizeof(uint16_t)));     	// First two bytes are length of data
	ESP_ERROR_CHECK(esp_flash_write(NULL, str, sector+sizeof(uint16_t), len));     // Rest of the data will be config string
	vTaskDelay(1000/portTICK_PERIOD_MS);

	uint16_t check;
	char *check_msg;
	ESP_ERROR_CHECK(esp_flash_read(NULL, &check, sector, sizeof(uint16_t)));
	check_msg = (char *)malloc(check);
	ESP_ERROR_CHECK(esp_flash_read(NULL, check_msg, sector+sizeof(uint16_t), check));
	if ((check != len) || (strncmp(str, check_msg, len) != 0)) 
	{
		printf("\nReceived: %d:%d->%s", len, strlen(str), str);
		printf("\nStored: %d:%d->%s", check, strlen(check_msg), check_msg);
		ESP_LOGE(TAG, "Config storage failed.");
        err = ESP_FAIL;
		char msg[50];
		time_t now;
		time(&now);
	}
	else 
	{
		ESP_LOGI(TAG, "Config stored successfully.");
		err = ESP_OK;
		char msg[50];
		time_t now;
		time(&now);
	}
	free(check_msg);
	vTaskDelay(2000/portTICK_PERIOD_MS);
	return err;
}

cJSON *read_config_data(flash_sector_t sector) {
	uint16_t len;
	char *msg;
	cJSON *json;

	ESP_ERROR_CHECK(esp_flash_read(NULL, &len, sector, sizeof(uint16_t)));     	// First two bytes are length of data
	if (len != 0 && len != 0xFFFF) { // the config data is present
		msg = malloc(len);
		ESP_ERROR_CHECK(esp_flash_read(NULL, msg, sector+sizeof(uint16_t), len));     	// Rest of the data will be config string
		json = cJSON_Parse(msg);
		free(msg);
	}
	else { // there is no config data
		json = NULL;
	}
	return (json);
}

// void get_inv_conf(inv_conf_t *p){
// 	nvs_handle_t nvs_handle;
// 	esp_err_t err;
// 	cJSON *json = read_config_data(DEV_CONFIG_ADDR);
// 	cJSON *ppp = cJSON_GetObjectItem(json, "ppp");

// 	err = nvs_open("net_conf", NVS_READONLY, &nvs_handle);

// 	cJSON *tmp = cJSON_GetObjectItem(ppp, "apn");
// 	if (tmp != NULL) {
// 		strcpy(p->apn, tmp->valuestring);
// 	}
// 	else { // Take default value from NVS
// 		size_t size = 0;
// 		err = nvs_get_str(nvs_handle, "ppp_apn", NULL, &size);
// 	    if (err == ESP_OK) {
// 			err = nvs_get_str(nvs_handle, "ppp_apn", p->apn, &size);
// 		}
// 	}
// 	tmp = cJSON_GetObjectItem(ppp, "username");
// 	if (tmp != NULL) {
// 		strcpy(p->username, tmp->valuestring);
// 	}
// 	else { // Take default value from NVS
// 		size_t size = 0;
// 		err = nvs_get_str(nvs_handle, "ppp_username", NULL, &size);
// 	    if (err == ESP_OK) {
// 			err = nvs_get_str(nvs_handle, "ppp_username", p->username, &size);
// 		}
// 	}
// 	tmp = cJSON_GetObjectItem(ppp, "password");
// 	if (tmp != NULL) {
// 		strcpy(p->password, tmp->valuestring);
// 	}
// 	else { // Take default value from NVS
// 		size_t size = 0;
// 		err = nvs_get_str(nvs_handle, "ppp_password", NULL, &size);
// 	    if (err == ESP_OK) {
// 			err = nvs_get_str(nvs_handle, "ppp_password", p->password, &size);
// 		}
// 	}
// 	nvs_close(nvs_handle);
// 	cJSON_Delete(json);
// }

void get_ppp_conf (ppp_conf_t *p) {
	nvs_handle_t nvs_handle;
	esp_err_t err;
	cJSON *json = read_config_data(DEV_CONFIG_ADDR);
	cJSON *ppp = cJSON_GetObjectItem(json, "ppp");

	err = nvs_open("net_conf", NVS_READONLY, &nvs_handle);

	cJSON *tmp = cJSON_GetObjectItem(ppp, "apn");
	if (tmp != NULL) {
		strcpy(p->apn, tmp->valuestring);
	}
	else { // Take default value from NVS
		size_t size = 0;
		err = nvs_get_str(nvs_handle, "ppp_apn", NULL, &size);
	    if (err == ESP_OK) {
			err = nvs_get_str(nvs_handle, "ppp_apn", p->apn, &size);
		}
	}

	tmp = cJSON_GetObjectItem(ppp, "username");
	if (tmp != NULL) {
		strcpy(p->username, tmp->valuestring);
	}
	else { // Take default value from NVS
		size_t size = 0;
		err = nvs_get_str(nvs_handle, "ppp_username", NULL, &size);
	    if (err == ESP_OK) {
			err = nvs_get_str(nvs_handle, "ppp_username", p->username, &size);
		}
	}

	tmp = cJSON_GetObjectItem(ppp, "password");
	if (tmp != NULL) {
		strcpy(p->password, tmp->valuestring);
	}
	else { // Take default value from NVS
		size_t size = 0;
		err = nvs_get_str(nvs_handle, "ppp_password", NULL, &size);
	    if (err == ESP_OK) {
			err = nvs_get_str(nvs_handle, "ppp_password", p->password, &size);
		}
	}

	nvs_close(nvs_handle);
	cJSON_Delete(json);
}

void get_sntp_conf (sntp_conf_t *p) {
	nvs_handle_t nvs_handle;
	esp_err_t err;
	cJSON *json = read_config_data(DEV_CONFIG_ADDR);
	cJSON *sntp = cJSON_GetObjectItem(json, "sntp");

	err = nvs_open("net_conf", NVS_READONLY, &nvs_handle);

	cJSON *tmp = cJSON_GetObjectItem(sntp, "url");
	if (tmp != NULL) {
		strcpy(p->url, tmp->valuestring);
	}
	else { // Take default value from NVS
		size_t size = 0;
		err = nvs_get_str(nvs_handle, "sntp_url", NULL, &size);
		char *str = malloc(size);
	    if (err == ESP_OK) {
			err = nvs_get_str(nvs_handle, "sntp_url", str, &size);
			p->url = str;
		}
		// free(str);
	}

	tmp = cJSON_GetObjectItem(sntp, "interval");
	if (tmp != NULL) {
		p->interval = tmp->valuedouble;
	}
	else { // Take default value from NVS
		err = nvs_get_i32(nvs_handle, "sntp_interval", &(p->interval));
		if (err != ESP_OK) p->interval = (int32_t)3600000;
	}
	nvs_close(nvs_handle);
	cJSON_Delete(json);
}


void get_mqtt_conf (mqtt_conf_t *p) {
	nvs_handle_t nvs_handle;
	esp_err_t err;
	cJSON *json = read_config_data(DEV_CONFIG_ADDR);
	cJSON *mqtt = cJSON_GetObjectItem(json, "mqtt");

	err = nvs_open("net_conf", NVS_READONLY, &nvs_handle);

	cJSON *tmp = cJSON_GetObjectItem(mqtt, "uri");
	if (tmp != NULL) {
		strcpy(p->uri, tmp->valuestring);
	}
	else { // Take default value from NVS
		size_t size = 0;
		err = nvs_get_str(nvs_handle, "mqtt_uri", NULL, &size);
	    if (err == ESP_OK) {
			err = nvs_get_str(nvs_handle, "mqtt_uri", p->uri, &size);
		}
	}

	tmp = cJSON_GetObjectItem(mqtt, "username");
	if (tmp != NULL) {
		strcpy(p->username, tmp->valuestring);
	}
	else { // Take default value from NVS
		size_t size = 0;
		err = nvs_get_str(nvs_handle, "mqtt_username", NULL, &size);
	    if (err == ESP_OK) {
			err = nvs_get_str(nvs_handle, "mqtt_username", p->username, &size);
		}
	}

	tmp = cJSON_GetObjectItem(mqtt, "password");
	if (tmp != NULL) {
		strcpy(p->password, tmp->valuestring);
	}
	else { // Take default value from NVS
		size_t size = 0;
		err = nvs_get_str(nvs_handle, "mqtt_password", NULL, &size);
	    if (err == ESP_OK) {
			err = nvs_get_str(nvs_handle, "mqtt_password", p->password, &size);
		}
	}

	tmp = cJSON_GetObjectItem(mqtt, "keepalive");
	if (tmp != NULL) {
		p->keepalive = tmp->valuedouble;
	}
	else { // Take default value from NVS
		err = nvs_get_i16(nvs_handle, "mqtt_keepalive", &(p->keepalive));
		if (err != ESP_OK) p->keepalive = 15; // in seconds
	}

	tmp = cJSON_GetObjectItem(mqtt, "timeout");
	if (tmp != NULL) {
		p->timeout = tmp->valuedouble;
	}
	else { // Take default value from NVS
		err = nvs_get_i16(nvs_handle, "mqtt_timeout", &(p->timeout));
		if (err != ESP_OK) p->timeout = 1000; // in ms
	}

	tmp = cJSON_GetObjectItem(mqtt, "qos");
	if (tmp != NULL) {
		p->qos = tmp->valuedouble;
	}
	else { // Take default value from NVS
		err = nvs_get_i8(nvs_handle, "mqtt_qos", &(p->qos));
		if (err != ESP_OK) p->qos = 1;
	}

	nvs_close(nvs_handle);
	cJSON_Delete(json);
}



void get_dev_data (dev_data_t *p) {
	nvs_handle_t nvs_handle;
	esp_err_t err;

	err = nvs_open("dev_conf", NVS_READONLY, &nvs_handle);

	size_t size = 0;
	err = nvs_get_str(nvs_handle, "model", NULL, &size);
	if (err == ESP_OK) {
		err = nvs_get_str(nvs_handle, "model", p->model, &size);
	}
	else {
		strcpy(p->model, "NA");
	}

	err = nvs_get_str(nvs_handle, "serial", NULL, &size);
	if (err == ESP_OK) {
		err = nvs_get_str(nvs_handle, "serial", p->serial, &size);
	}
	else {
		strcpy(p->serial, "NA");
	}
	nvs_close(nvs_handle);

	const esp_app_desc_t *app = esp_app_get_description();
	if (app) {
		strcpy(p->firmware, app->version);
	}
	else {
		strcpy(p->firmware, "NA");
	}
}

esp_err_t set_serial_number (char *str) {
	nvs_handle_t nvs_handle;
	esp_err_t err;
	cJSON *json;
	char serial[20];

	err = nvs_open("dev_conf", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return err;

	size_t size = 0;
	err = nvs_get_str(nvs_handle, "serial", NULL, &size);

	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
		nvs_close(nvs_handle);
		return err;
	}

	if (err != ESP_ERR_NVS_NOT_FOUND) err = nvs_get_str(nvs_handle, "serial", serial, &size);

	if (strcmp(serial, "0000000000") != 0) { // the device already has some serial number -> cannot be overwritten.
		nvs_close(nvs_handle);
		return ESP_FAIL;
	}

	json = cJSON_Parse(str);
	cJSON *tmp = cJSON_GetObjectItem(json, "serial");
	if (tmp != NULL) {
		err = nvs_set_str(nvs_handle, "serial", tmp->valuestring);

	    if (err == ESP_OK) err = nvs_commit(nvs_handle);
	}

	nvs_close(nvs_handle);
	cJSON_Delete(json);

	return err;
}




// void get_WiFi_conf (wifi_conf_t *p) {
// 	nvs_handle_t nvs_handle;
// 	esp_err_t err;
// 	cJSON *json = read_config_data(WIFI_CONFIG_ADDR);
// 	cJSON *_wifi = cJSON_GetObjectItem(json, "_wifi");

// 	err = nvs_open("net_conf", NVS_READONLY, &nvs_handle);


// 	tmp = cJSON_GetObjectItem(_wifi, "username");
// 	if (tmp != NULL) {
// 		strcpy(p->username, tmp->valuestring);
// 	}
// 	else { // Take default value from NVS
// 		size_t size = 0;
// 		err = nvs_get_str(nvs_handle, "wifi_username", NULL, &size);
// 	    if (err == ESP_OK) {
// 			err = nvs_get_str(nvs_handle, "wifi_username", p->username, &size);
// 		}
// 	}

// 	tmp = cJSON_GetObjectItem(_wifi, "password");
// 	if (tmp != NULL) {
// 		strcpy(p->password, tmp->valuestring);
// 	}
// 	else { // Take default value from NVS
// 		size_t size = 0;
// 		err = nvs_get_str(nvs_handle, "_wifi_password", NULL, &size);
// 	    if (err == ESP_OK) {
// 			err = nvs_get_str(nvs_handle, "_wifi_password", p->password, &size);
// 		}
// 	}
// 	nvs_close(nvs_handle);
// 	cJSON_Delete(json);
// }