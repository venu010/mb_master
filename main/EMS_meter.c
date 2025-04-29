#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "math.h"
#include "esp_timer.h"
#include "Defines.h"

#include "modbus_params.h"  // for modbus parameters structures
#include "mbcontroller.h"

#define TAG   "EMS-01"

#define BUF_SIZE1 (127)


static mb_param_request_t request5;

extern esp_err_t err;

#define PACKET_READ_TICS        (100 / portTICK_PERIOD_MS)
extern char dev_mac[30],str_status1[100];
extern uint16_t crc_fun(unsigned char *nData, uint16_t wLength);
extern char serialno[50],modelno[20], mload, mresult, SIM_ID[30],IMEI[22], date_time[30], version[5],mresult1,error_code2[20],warning_code1[20];
extern int error_code,str_status,warning_code,working_mode,run_time,run_status;
extern float ph_freq,pv1_vol,pv1_curr,pv2_vol,pv2_curr,pv3_vol,pv3_curr,pv4_vol,pv4_curr,ph1_vol,ph1_curr,ph2_vol,ph2_curr,ph3_vol,ph3_curr,str1,str2,str3,str4,str5,str6,str7,str8,str9,str10,str11,str12,str13,str14,str15,str16,ac_power,life_energy,today_energy,inv_temp,rtime,watt_hr;
extern int reading1, reading2,reading3;
extern uint8_t goodwe_greater_100kwp_status;
extern uint8_t got_response;
extern QueueHandle_t loggerQueue_h;
extern int32_t rssi;
extern uint8_t sync_done;
extern uint8_t callback_status;
extern date_time_t datetime;

uint8_t dev_id=0;

char ems_data1[4096];


extern float avrage_value;
int i,var = 0;
float v1 =0;
float Rphase_V,Yphase_V,Bphase_V,Rphase_C,Yphase_C,Bphase_C,Rphase_Yphase_V,Yphase_Bphase_V,Bphase_Rphase_V,PF_R,PF_Y,PF_B,TPF,ACT_P_R,ACT_P_Y,ACT_P_B,TACT_P,active_import,active_export,active_total=0;
float frequency;
// uint16_t power_factor;
uint16_t alram_status;

int get_EMS_meter_readings( uint16_t reg_add,uint16_t reg_len);
void get_frequency(const int port);
void get_phase_voltage(const int port);
void get_phase_current(const int port);
void get_phase_to_phase_voltage(const int port);
void get_power_factor(const int port);
void get_active_power(const int port);
void get_active_import(const int port);
void get_active_export(const int port);
void get_active_total(const int port);
void all_reg_data(const int port);
extern uint16_t crc_fun(unsigned char *nData, uint16_t wLength);
extern void echo_send(const int port, uint8_t *data, uint8_t length);

// extern esp_err_t send_meas_data1(char *msg);

// static const uint16_t wCRCTable[] = {
//     0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
//     0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
//     0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
//     0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
//     0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
//     0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
//     0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
//     0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
//     0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
//     0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
//     0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
//     0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
//     0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
//     0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
//     0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
//     0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
//     0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
//     0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
//     0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
//     0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
//     0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
//     0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
//     0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
//     0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
//     0X2000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
//     0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
//     0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
//     0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
//     0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
//     0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
//     0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
//     0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };
    
    
// uint16_t crc_fun(unsigned char *nData, uint16_t wLength)
// {
//     uint8_t nTemp;
//     uint16_t wCRCWord = 0xFFFF;
    
//     while (wLength--)
//     {
//         nTemp = *nData++ ^ wCRCWord;
//         wCRCWord >>= 8;
//         wCRCWord ^= wCRCTable[nTemp];
//     }
//     return wCRCWord;
    
// }
    
// static void echo_send(const int port, uint8_t *data, uint8_t length)
// {
//     if (uart_write_bytes(port, (const char *)data, length) != length) {
//         ESP_LOGE(TAG, "Send data critical failure.");
//         // add your code to handle sending failure here
//         abort();
//     }
// }

float convert_data(uint32_t array){
    // printf("array=%lx\n",array);
    for (int i = 0; i < 9; i++){
        if(((array>>(i+23))&1) == 1){
            var = (var + pow(2,i));
        }
    }
    for(int i = 0; i<23; i++){
        if(((array>>(23-i)&1)== 1)){
            v1 = (v1 + (1/(pow(2,i))));
        }
    }
    var = var -127;
    v1 = (1+v1);
    avrage_value = v1 *(pow(2,var));
    // printf("avrage_value=%.2f\n",avrage_value);
    v1 =0;var = 0;
return avrage_value;
}


void get_frequency(const int port){
    // printf("data==========\n");
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE1);
    int crc;
    frequency =0;
    uint8_t modbus_request[8];
    modbus_request[0]= 0X01;
    modbus_request[1]= 0X03;
    modbus_request[2]= 0X00;
    modbus_request[3]= 0X01;
    modbus_request[4]= 0X00;
    modbus_request[5]= 0X02;
    crc =crc_fun(modbus_request, 6);
    modbus_request[6]= crc;
    modbus_request[7]= crc>>8;
    while(1){
        vTaskDelay(2000 / portTICK_PERIOD_MS); 
        echo_send(port, modbus_request, sizeof(modbus_request));
        while(1) {
            //Read data from UART
            int len = uart_read_bytes(port, data, BUF_SIZE1, PACKET_READ_TICS);
            // i++; 
            //Write data back to UART
            if (len > 0) {
                ESP_LOGI(TAG, "Frequency Received %u bytes:\n  ", len);
                reading1 = (data[4]&0x000000FF);
                reading1 |= (data[3] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[6] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[5] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                frequency=convert_data(reading3);
                // printf("frequency =%.2f\n",frequency);
                got_response =TRUE;
                break;
            } 
        }
        break;

    }
    
}

void get_phase_voltage(const int port){
    // printf("data==========\n");
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE1);
    int crc;
    Rphase_V=0;
    Yphase_V=0;
    Bphase_V=0;
    uint8_t modbus_request[8];
    modbus_request[0]= 0X01;
    modbus_request[1]= 0X03;
    modbus_request[2]= 0X00;
    modbus_request[3]= 0X03;
    modbus_request[4]= 0X00;
    modbus_request[5]= 0X06;
    crc =crc_fun(modbus_request, 6);
    modbus_request[6]= crc;
    modbus_request[7]= crc>>8;
    // while(1){
        echo_send(port, modbus_request, sizeof(modbus_request));
        while(1) {
            //Read data from UART
            int len = uart_read_bytes(port, data, BUF_SIZE1, PACKET_READ_TICS);
            // i++; 
            //Write data back to UART
            if (len > 0) {
                ESP_LOGI(TAG, "Phase voltage Received %u bytes:\n  ", len);
                reading1 = (data[4]&0x000000FF);
                reading1 |= (data[3] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[6] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[5] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Rphase_V=convert_data(reading3);
                // printf("Rphase_voltage =%.2f\n",Rphase_V);
                reading1 = (data[8]&0x000000FF);
                reading1 |= (data[7] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[10] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[9] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Yphase_V=convert_data(reading3);
                // printf("Yphase_voltage =%.2f\n",Yphase_V);
                reading1 = (data[12]&0x000000FF);
                reading1 |= (data[11] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[14] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[13] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Bphase_V=convert_data(reading3);
                // printf("Bphase_voltage =%.2f\n",Bphase_V);
                break;
            } 
        // }
        // break;
    }    
}

void get_phase_current(const int port){
    // printf("data==========\n");
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE1);
    int crc;
    Rphase_C=0;
    Yphase_C=0;
    Bphase_C=0;
    uint8_t modbus_request[8];
    modbus_request[0]= 0X01;
    modbus_request[1]= 0X03;
    modbus_request[2]= 0X00;
    modbus_request[3]= 0X0F;
    modbus_request[4]= 0X00;
    modbus_request[5]= 0X06;
    crc =crc_fun(modbus_request, 6);
    modbus_request[6]= crc;
    modbus_request[7]= crc>>8;
    // while(1){
        echo_send(port, modbus_request, sizeof(modbus_request));
        // while(1) {
            //Read data from UART
            int len = uart_read_bytes(port, data, BUF_SIZE1, PACKET_READ_TICS);
            i++; 
            //Write data back to UART
            if (len > 0) {
                ESP_LOGI(TAG, "Phase current Received %u bytes:\n  ", len);
                
                reading1 = (data[4]&0x000000FF);
                reading1 |= (data[3] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[6] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[5] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Rphase_C=convert_data(reading3);
                // printf("Rphase_current =%.2f\n",Rphase_C);
                reading1 = (data[8]&0x000000FF);
                reading1 |= (data[7] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[10] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[9] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Yphase_C=convert_data(reading3);
                // printf("Yphase_current =%.2f\n",Yphase_C);
                reading1 = (data[12]&0x000000FF);
                reading1 |= (data[11] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[14] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[13] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Bphase_C=convert_data(reading3);
                // printf("Bphase_current =%.2f\n",Bphase_C);
        //         break;
        //     } 
        // }
        // break;
    }
}

void get_phase_to_phase_voltage(const int port){
    // printf("data==========\n");
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE1);
    int crc;
    Rphase_Yphase_V=0;
    Yphase_Bphase_V=0;
    Bphase_Rphase_V=0;

    uint8_t modbus_request[8];
    modbus_request[0]= 0X01;
    modbus_request[1]= 0X03;
    modbus_request[2]= 0X00;
    modbus_request[3]= 0X09;
    modbus_request[4]= 0X00;
    modbus_request[5]= 0X06;
    crc =crc_fun(modbus_request, 6);
    modbus_request[6]= crc;
    modbus_request[7]= crc>>8;
    // while(1){
        echo_send(port, modbus_request, sizeof(modbus_request));
        // while(1) {
            //Read data from UART
            int len = uart_read_bytes(port, data, BUF_SIZE1, PACKET_READ_TICS);
            i++; 
            //Write data back to UART
            if (len > 0) {
                ESP_LOGI(TAG, "phase to phase voltage Received %u bytes:\n  ", len);
                
                reading1 = (data[4]&0x000000FF);
                reading1 |= (data[3] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[6] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[5] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Rphase_Yphase_V=convert_data(reading3);
                // printf("Rphase_Yphase_V =%.2f\n",Rphase_Yphase_V);
                reading1 = (data[8]&0x000000FF);
                reading1 |= (data[7] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[10] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[9] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Yphase_Bphase_V=convert_data(reading3);
                // printf("Yphase_Bphase_V =%.2f\n",Yphase_Bphase_V);
                reading1 = (data[12]&0x000000FF);
                reading1 |= (data[11] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[14] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[13] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Bphase_Rphase_V=convert_data(reading3);
                // printf("Bphase_Rphase_V =%.2f\n",Bphase_Rphase_V);
        //         break;
        //     } 
        // }
        // break;
    }
}

void get_power_factor(const int port){
    // printf("data==========\n");
uint8_t* data = (uint8_t*) malloc(BUF_SIZE1);
    int crc;
    PF_R=0;
    PF_Y=0;
    PF_B=0;
    TPF=0;    
    uint8_t modbus_request[8];
    modbus_request[0]= 0X01;
    modbus_request[1]= 0X03;
    modbus_request[2]= 0X00;
    modbus_request[3]= 0X15;
    modbus_request[4]= 0X00;
    modbus_request[5]= 0X08;
    crc =crc_fun(modbus_request, 6);
    modbus_request[6]= crc;
    modbus_request[7]= crc>>8;
    echo_send(port, modbus_request, sizeof(modbus_request));
    //Read data from UART
    int len = uart_read_bytes(port, data, BUF_SIZE1, PACKET_READ_TICS);
    i++; 
    //Write data back to UART
    if (len > 0) {
        ESP_LOGI(TAG, "phase power factor Received %u bytes:\n  ", len);
        
        reading1 = (data[4]&0x000000FF);
        reading1 |= (data[3] << 8) & 0x0000FF00; 
        reading1 = reading1 & 0x0000FFFF;
        reading2 = data[6] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data[5] << 8) & 0x0000FF00;
        reading2 = reading2 << 16 & 0xFFFF0000;
        reading3 = reading1 | reading2;
        // printf("reading3 =%lx\n",reading3);
        PF_R=convert_data(reading3);
        // printf("PF_R =%.2f\n",PF_R);
        reading1 = (data[8]&0x000000FF);
        reading1 |= (data[7] << 8) & 0x0000FF00; 
        reading1 = reading1 & 0x0000FFFF;
        reading2 = data[10] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data[9] << 8) & 0x0000FF00;
        reading2 = reading2 << 16 & 0xFFFF0000;
        reading3 = reading1 | reading2;
        // printf("reading3 =%lx\n",reading3);
        PF_Y=convert_data(reading3);
        // printf("PF_Y =%.2f\n",PF_Y);
        reading1 = (data[12]&0x000000FF);
        reading1 |= (data[11] << 8) & 0x0000FF00; 
        reading1 = reading1 & 0x0000FFFF;
        reading2 = data[14] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data[13] << 8) & 0x0000FF00;
        reading2 = reading2 << 16 & 0xFFFF0000;
        reading3 = reading1 | reading2;
        // printf("reading3 =%lx\n",reading3);
        PF_B=convert_data(reading3);
        // printf("PF_B =%.2f\n",PF_B);
        reading1 = (data[16]&0x000000FF);
        reading1 |= (data[15] << 8) & 0x0000FF00; 
        reading1 = reading1 & 0x0000FFFF;
        reading2 = data[18] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data[17] << 8) & 0x0000FF00;
        reading2 = reading2 << 16 & 0xFFFF0000;
        reading3 = reading1 | reading2;
        // printf("reading3 =%lx\n",reading3);
        TPF=convert_data(reading3);
        // printf("TPF =%.2f\n",TPF);

    }
}

void get_active_power(const int port){
    // printf("data==========\n");
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE1);
    int crc;
    ACT_P_R=0;
    ACT_P_Y=0;
    ACT_P_B=0;
    TACT_P=0;

    uint8_t modbus_request[8];
    modbus_request[0]= 0X01;
    modbus_request[1]= 0X03;
    modbus_request[2]= 0X00;
    modbus_request[3]= 0X1D;
    modbus_request[4]= 0X00;
    modbus_request[5]= 0X08;
    crc =crc_fun(modbus_request, 6);
    modbus_request[6]= crc;
    modbus_request[7]= crc>>8;
    echo_send(port, modbus_request, sizeof(modbus_request));
    //Read data from UART
    int len = uart_read_bytes(port, data, BUF_SIZE1, PACKET_READ_TICS);
    i++; 
    //Write data back to UART
    if (len > 0) {
        ESP_LOGI(TAG, "active power Received %u bytes:\n  ", len);
        
        reading1 = (data[4]&0x000000FF);
        reading1 |= (data[3] << 8) & 0x0000FF00; 
        reading1 = reading1 & 0x0000FFFF;
        reading2 = data[6] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data[5] << 8) & 0x0000FF00;
        reading2 = reading2 << 16 & 0xFFFF0000;
        reading3 = reading1 | reading2;
        // printf("reading3 =%lx\n",reading3);
        ACT_P_R=convert_data(reading3);
        // printf("ACT_P_R =%.2f\n",ACT_P_R);
        reading1 = (data[8]&0x000000FF);
        reading1 |= (data[7] << 8) & 0x0000FF00; 
        reading1 = reading1 & 0x0000FFFF;
        reading2 = data[10] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data[9] << 8) & 0x0000FF00;
        reading2 = reading2 << 16 & 0xFFFF0000;
        reading3 = reading1 | reading2;
        // printf("reading3 =%lx\n",reading3);
        ACT_P_Y=convert_data(reading3);
        // printf("ACT_P_Y =%.2f\n",ACT_P_Y);
        reading1 = (data[12]&0x000000FF);
        reading1 |= (data[11] << 8) & 0x0000FF00; 
        reading1 = reading1 & 0x0000FFFF;
        reading2 = data[14] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data[13] << 8) & 0x0000FF00;
        reading2 = reading2 << 16 & 0xFFFF0000;
        reading3 = reading1 | reading2;
        // printf("reading3 =%lx\n",reading3);
        ACT_P_B=convert_data(reading3);
        // printf("ACT_P_B =%.2f\n",ACT_P_B);
        reading1 = (data[16]&0x000000FF);
        reading1 |= (data[15] << 8) & 0x0000FF00; 
        reading1 = reading1 & 0x0000FFFF;
        reading2 = data[18] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data[17] << 8) & 0x0000FF00;
        reading2 = reading2 << 16 & 0xFFFF0000;
        reading3 = reading1 | reading2;
        // printf("reading3 =%lx\n",reading3);
        TACT_P=convert_data(reading3);
        // printf("TACT_P =%.2f\n",TACT_P);
    }
}
void get_active_import(const int port){
    // printf("data==========\n");
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE1);
    int crc;
    active_import=0;
    uint8_t modbus_request[8];
    modbus_request[0]= 0X01;
    modbus_request[1]= 0X03;
    modbus_request[2]= 0X00;
    modbus_request[3]= 0X35;
    modbus_request[4]= 0X00;
    modbus_request[5]= 0X02;
    crc =crc_fun(modbus_request, 6);
    modbus_request[6]= crc;
    modbus_request[7]= crc>>8;
    echo_send(port, modbus_request, sizeof(modbus_request));
    int len = uart_read_bytes(port, data, BUF_SIZE1, PACKET_READ_TICS);
    i++; 
    //Write data back to UART
    if (len > 0) {
        ESP_LOGI(TAG, "active import Received %u bytes:\n  ", len);
        reading1 = (data[4]&0x000000FF);
        reading1 |= (data[3] << 8) & 0x0000FF00; 
        reading1 = reading1 & 0x0000FFFF;
        reading2 = data[6] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data[5] << 8) & 0x0000FF00;
        reading2 = reading2 << 16 & 0xFFFF0000;
        reading3 = reading1 | reading2;
        // printf("reading3 =%lx\n",reading3);
        active_import=convert_data(reading3);
        // printf("active_import =%.2f\n",active_import);
    }
}
void get_active_export(const int port){
    // printf("data==========\n");
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE1);
    int crc;
    active_export=0;

    uint8_t modbus_request[8];
    modbus_request[0]= 0X01;
    modbus_request[1]= 0X03;
    modbus_request[2]= 0X00;
    modbus_request[3]= 0X37;
    modbus_request[4]= 0X00;
    modbus_request[5]= 0X02;
    crc =crc_fun(modbus_request, 6);
    modbus_request[6]= crc;
    modbus_request[7]= crc>>8;
    echo_send(port, modbus_request, sizeof(modbus_request));
    int len = uart_read_bytes(port, data, BUF_SIZE1, PACKET_READ_TICS);
    i++; 
    //Write data back to UART
    if (len > 0) {
        ESP_LOGI(TAG, "active export Received %u bytes:\n  ", len);
        reading1 = (data[4]&0x000000FF);
        reading1 |= (data[3] << 8) & 0x0000FF00; 
        reading1 = reading1 & 0x0000FFFF;
        reading2 = data[6] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data[5] << 8) & 0x0000FF00;
        reading2 = reading2 << 16 & 0xFFFF0000;
        reading3 = reading1 | reading2;
        // printf("reading3 =%lx\n",reading3);
        active_export=convert_data(reading3);
        // printf("active_import =%.2f\n",active_export);
    }
}

void get_active_total(const int port){
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE1);
    int crc;
    active_total=0;

    uint8_t modbus_request[8];
    modbus_request[0]= 0X01;
    modbus_request[1]= 0X03;
    modbus_request[2]= 0X00;
    modbus_request[3]= 0X35;
    modbus_request[4]= 0X00;
    modbus_request[5]= 0X02;
    crc =crc_fun(modbus_request, 6);
    modbus_request[6]= crc;
    modbus_request[7]= crc>>8;
    echo_send(port, modbus_request, sizeof(modbus_request));
    int len = uart_read_bytes(port, data, BUF_SIZE1, PACKET_READ_TICS);
    i++; 
    //Write data back to UART
    if (len > 0) {
        ESP_LOGI(TAG, "active Total Received %u bytes:\n  ", len);
        reading1 = (data[4]&0x000000FF);
        reading1 |= (data[3] << 8) & 0x0000FF00; 
        reading1 = reading1 & 0x0000FFFF;
        reading2 = data[6] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data[5] << 8) & 0x0000FF00;
        reading2 = reading2 << 16 & 0xFFFF0000;
        reading3 = reading1 | reading2;
        // printf("reading3 =%lx\n",reading3);
        active_total=convert_data(reading3);
        // printf("active_total =%.2f\n",active_total);

        printf("1.Frequency =%.2f\n2.Rphase_V =%.2f\n3.Yphase_V =%.2f\n4.Bphase_V =%.2f\n5.Rphase_C =%.2f\n6.Yphase_C =%.2f\n7.Bphase_C =%.2f\n8.Rphase_Yphase_V =%.2f\n9.Yphase_Bphase_V =%.2f\n10.Bphase_Rphase_V =%.2f\n11.PF_R =%.2f\n12.PF_Y =%.2f\n13.PF_B =%.2f\n14.TPF =%.2f\n15.ACT_P_R =%.2f\n16.ACT_P_Y =%.2f\n17.ACT_P_B =%.2f\n18.TACT_P =%.2f\n19.active_import= %.2f\n20.active_export= %.2f\n21.active_total= %.2f\n",frequency,Rphase_V,Yphase_V,Bphase_V,Rphase_C,Yphase_C,Bphase_C,Rphase_Yphase_V,Yphase_Bphase_V,Bphase_Rphase_V,PF_R,PF_Y,PF_B,TPF,ACT_P_R,ACT_P_Y,ACT_P_B,TACT_P,active_import,active_export,active_total);
    }
}

void all_reg_data(const int port){
    // printf("data==========\n");
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE1);
    int crc;
    frequency=0;
    dev_id = 0x01;
    uint8_t modbus_request[8];
    modbus_request[0]= 0X01;
    modbus_request[1]= 0X03;
    modbus_request[2]= 0X00;
    modbus_request[3]= 0X01;
    modbus_request[4]= 0X00;
    modbus_request[5]= 0X3A;
    crc =crc_fun(modbus_request, 6);
    modbus_request[6]= crc;
    modbus_request[7]= crc>>8;
    while(1){
        vTaskDelay(2000 / portTICK_PERIOD_MS); 
        echo_send(port, modbus_request, sizeof(modbus_request));
        while(1) {
            //Read data from UART
            int len = uart_read_bytes(port, data, BUF_SIZE1, PACKET_READ_TICS);
            // i++; 
            //Write data back to UART
            if (len > 0) {
                ESP_LOGI(TAG, "all reg data Received %u bytes:\n  ", len);
                reading1 = (data[4]&0x000000FF);
                reading1 |= (data[3] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[6] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[5] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                frequency=convert_data(reading3);
                // printf("frequency =%.2f\n",frequency);

                reading1 = (data[8]&0x000000FF);
                reading1 |= (data[7] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[10] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[9] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Rphase_V=convert_data(reading3);
                // printf("Rphase_voltage =%.2f\n",Rphase_V);

                reading1 = (data[12]&0x000000FF);
                reading1 |= (data[11] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[14] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[13] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Yphase_V=convert_data(reading3);
                // printf("Yphase_voltage =%.2f\n",Yphase_V);

                reading1 = (data[16]&0x000000FF);
                reading1 |= (data[15] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[18] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[17] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Bphase_V=convert_data(reading3);
                // printf("Bphase_voltage =%.2f\n",Bphase_V);
                
                reading1 = (data[20]&0x000000FF);
                reading1 |= (data[19] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[22] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[21] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Rphase_Yphase_V=convert_data(reading3);
                // printf("Rphase_Yphase_V =%.2f\n",Rphase_Yphase_V);
                // printf("Rphase_current =%.2f\n",Rphase_C);

                reading1 = (data[24]&0x000000FF);
                reading1 |= (data[23] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[26] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[25] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Yphase_Bphase_V =convert_data(reading3);
                // printf("Yphase_Bphase_V =%.2f\n",Yphase_Bphase_V);
                // printf("Yphase_current =%.2f\n",Yphase_C);

                reading1 = (data[28]&0x000000FF);
                reading1 |= (data[27] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[30] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[29] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Bphase_Rphase_V =convert_data(reading3);
                // printf("Bphase_Rphase_V =%.2f\n",Bphase_Rphase_V);

                reading1 = (data[32]&0x000000FF);
                reading1 |= (data[31] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[34] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[33] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Rphase_C=convert_data(reading3);
                // printf("Rphase_current =%.2f\n",Rphase_C);

                reading1 = (data[36]&0x000000FF);
                reading1 |= (data[35] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[38] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[37] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Yphase_C=convert_data(reading3);
                // printf("Yphase_current =%.2f\n",Yphase_C);

                reading1 = (data[40]&0x000000FF);
                reading1 |= (data[39] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[42] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[41] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                Bphase_C=convert_data(reading3);
                // printf("Bphase_current =%.2f\n",Bphase_C);

                reading1 = (data[44]&0x000000FF);
                reading1 |= (data[43] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[46] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[45] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                PF_R=convert_data(reading3);
                // printf("PF_R =%.2f\n",PF_R);

                reading1 = (data[48]&0x000000FF);
                reading1 |= (data[47] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[50] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[49] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                PF_Y=convert_data(reading3);
                // printf("PF_Y =%.2f\n",PF_Y);

                reading1 = (data[52]&0x000000FF);
                reading1 |= (data[51] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[54] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[53] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                PF_B=convert_data(reading3);
                // printf("PF_B =%.2f\n",PF_B);

                reading1 = (data[56]&0x000000FF);
                reading1 |= (data[55] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[58] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[57] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                TPF=convert_data(reading3);
                // printf("TPF =%.2f\n",TPF);

                reading1 = (data[60]&0x000000FF);
                reading1 |= (data[59] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[62] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[61] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                ACT_P_R=convert_data(reading3);
                // printf("ACT_P_R =%.2f\n",ACT_P_R);

                reading1 = (data[64]&0x000000FF);
                reading1 |= (data[63] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[66] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[65] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                ACT_P_Y=convert_data(reading3);
                // printf("ACT_P_Y =%.2f\n",ACT_P_Y);

                reading1 = (data[68]&0x000000FF);
                reading1 |= (data[67] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[70] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[69] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                ACT_P_B=convert_data(reading3);
                // printf("ACT_P_B =%.2f\n",ACT_P_B);

                reading1 = (data[72]&0x000000FF);
                reading1 |= (data[71] << 8) & 0x0000FF00; 
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[74] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[73] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                TACT_P=convert_data(reading3);
                // printf("TACT_P =%.2f\n",TACT_P);

                reading1 = (data[108]&0x000000FF);
                reading1 |= (data[107] << 8) & 0x0000FF00;
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[110] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[109] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                active_import=convert_data(reading3);
                // printf("active_import =%.2f\n",active_import);

                reading1 = (data[112]&0x000000FF);
                reading1 |= (data[111] << 8) & 0x0000FF00;
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[114] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[113] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                active_export=convert_data(reading3);
                // printf("active_export =%.2f\n",active_export);

                reading1 = (data[116]&0x000000FF);
                reading1 |= (data[115] << 8) & 0x0000FF00;
                reading1 = reading1 & 0x0000FFFF;
                reading2 = data[118] & 0x000000FF; //Load LSB of higher 16bit
                reading2 |= (data[117] << 8) & 0x0000FF00;
                reading2 = reading2 << 16 & 0xFFFF0000;
                reading3 = reading1 | reading2;
                // printf("reading3 =%lx\n",reading3);
                active_total=convert_data(reading3);
                // printf("active_total =%.2f\n",active_total);

                // printf("1.Frequency =%.2f\n2.Rphase_V =%.2f\n3.Yphase_V =%.2f\n4.Bphase_V =%.2f\n5.Rphase_C =%.2f\n6.Yphase_C =%.2f\n7.Bphase_C =%.2f\n8.Rphase_Yphase_V =%.2f\n9.Yphase_Bphase_V =%.2f\n10.Bphase_Rphase_V =%.2f\n11.PF_R =%.2f\n12.PF_Y =%.2f\n13.PF_B =%.2f\n14.TPF =%.2f\n15.ACT_P_R =%.2f\n16.ACT_P_Y =%.2f\n17.ACT_P_B =%.2f\n18.TACT_P =%.2f\n19.active_import= %.2f\n20.active_export= %.2f\n21.active_total= %.2f\n",frequency,Rphase_V,Yphase_V,Bphase_V,Rphase_C,Yphase_C,Bphase_C,Rphase_Yphase_V,Yphase_Bphase_V,Bphase_Rphase_V,PF_R,PF_Y,PF_B,TPF,ACT_P_R,ACT_P_Y,ACT_P_B,TACT_P,active_import,active_export,active_total);

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
                    ESP_LOGI(TAG, "DATE time = %d-%d-%d   %d:%d:%d",datetime.day,datetime.month,datetime.year,datetime.hours,datetime.mins,datetime.secs);
                    interTaskMsg_t msg;
                    strcpy(ems_data1,"");
                    sprintf(ems_data1,"\"MACID\":\"%s\",\"RSignal\":\"%d\",\"ID\":\"%d\",\"frequency\":\"%.2f\",\"Rphase_V\":\"%.2f\",\"Yphase_V\":\"%.2f\",\"Bphase_V\":\"%.2f\",\"Rphase_C\":\"%.2f\",\"Yphase_C\":\"%.2f\",\"Bphase_C\":\"%.2f\",\"Rphase_Yphase_V\":\"%.2f\",\"Yphase_Bphase_V\":\"%.2f\",\"Bphase_Rphase_V\":\"%.2f\",\"PF_R\":\"%.2f\",\"PF_Y\":\"%.2f\",\"PF_B\":\"%.2f\",\"TPF\":\"%.2f\",\"ACT_P_R\":\"%.2f\",\"ACT_P_Y\":\"%.2f\",\"ACT_P_B\":\"%.2f\",\"TACT_P\":\"%.2f\",\"active_import\": \"%.2f\", \"active_export\": \"%.2f\", \"active_total\": \"%.2f\"",dev_mac,rssi,dev_id,frequency,Rphase_V,Yphase_V,Bphase_V,Rphase_C,Yphase_C,Bphase_C,Rphase_Yphase_V,Yphase_Bphase_V,Bphase_Rphase_V,PF_R,PF_Y,PF_B,TPF,ACT_P_R,ACT_P_Y,ACT_P_B,TACT_P,active_import,active_export,active_total);
                    msg.msgId = DATA_ACQ_MSG;
                    msg.data = ems_data1;
                    // printf("ems_data1 = %s\n", ems_data1);
                    // send_meas_data1(msg.data);
                    xQueueSend(loggerQueue_h, (void *) &msg, 0);
                    
                    // got_response =FALSE;                    
                    // memset(ems_data1, 0, sizeof(ems_data1));
                }

                // printf("1.Frequency =%.2f\n2.Rphase_V =%.2f\n3.Yphase_V =%.2f\n4.Bphase_V =%.2f\n5.Rphase_C =%.2f\n6.Yphase_C =%.2f\n7.Bphase_C =%.2f\n8.Rphase_Yphase_V =%.2f\n9.Yphase_Bphase_V =%.2f\n10.Bphase_Rphase_V =%.2f\n11.PF_R =%.2f\n12.PF_Y =%.2f\n13.PF_B =%.2f\n14.TPF =%.2f\n15.ACT_P_R =%.2f\n16.ACT_P_Y =%.2f\n17.ACT_P_B =%.2f\n18.TACT_P =%.2f\n19.active_import= %.2f\n20.active_export= %.2f\n21.active_total= %.2f\n",frequency,Rphase_V,Yphase_V,Bphase_V,Rphase_C,Yphase_C,Bphase_C,Rphase_Yphase_V,Yphase_Bphase_V,Bphase_Rphase_V,PF_R,PF_Y,PF_B,TPF,ACT_P_R,ACT_P_Y,ACT_P_B,TACT_P,active_import,active_export,active_total);
                break;
            } 
        }
        break;

    }
}


int get_EMS_meter_readings(uint16_t reg_add,uint16_t reg_len){
    uint8_t data200[100];
    request5.slave_addr = 1;
	request5.command = 0x03;
	request5.reg_start = reg_add;
	request5.reg_size = reg_len;
    while(1){
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
        err = mbc_master_send_request(&request5,&data200);
        if (err== ESP_OK){
            break;
        }
    }

    reading1 = data200[1]&0x000000FF;
    reading1 |= (data200[0] << 8) & 0x0000FF00;
    reading1 = reading1  & 0xFFFF0000;
    reading2 = data200[3] & 0x000000FF; //Load LSB of higher 16bit
    reading2 |= (data200[2] << 8) & 0x0000FF00;
    reading2 = (reading2 << 16) & 0x0000FFFF;
    reading3 = reading1 | reading2;
    ac_power=convert_data(reading3);
    printf("ac_power=%.2f\n",ac_power);
   return 0;
}