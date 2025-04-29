/* Uart Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
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
#include "esp_system.h"

// #include "modbus_params.h"  // for modbus parameters structures
#include "mbcontroller.h"



/**
 * This is a example which echos any data it receives on UART back to the sender using RS485 interface in half duplex mode.
*/
#define TAG10 "INV_GEN"
#define TAG11 "MODBUS"

// Note: Some pins on target chip cannot be assigned for UART communication.
// Please refer to documentation for selected board and target to configure pins using Kconfig.
// #define ECHO_TEST_TXD   (CONFIG_ECHO_UART_TXD)
// #define ECHO_TEST_RXD   (CONFIG_ECHO_UART_RXD)
// #define ECHO_TEST_TXD   (19)
// #define ECHO_TEST_RXD   (18)
// RTS for RS485 Half-Duplex Mode manages DE/~RE
// #define ECHO_TEST_RTS   (CONFIG_ECHO_UART_RTS)
// #define ECHO_TEST_RTS   (23)

#define MB_TEST_TXD   (45)//esp32 with rs485 roard
#define MB_TEST_RXD   (48)
#define MB_TEST_RTS   (10)
#define RS485_5V_PIN_EN (16)
#define RS485_SE_PIN    (19)
#define HIGH               1
#define LED1            (04)

#define MB_PORT_NUM        2
#define MB_DEV_SPEED    9600  // The communication speed of the UART





extern char inverter_type[20];
extern char dev_mac[30];
extern uint8_t numOfGoodweInverter,numOfDeltaInverter,numOfSungrowInverter,numOfSolisInverter,numOfSingegInverter,numOfGrowattInverter,numOfschniderinverter,numOfSmaInverter,numOfHuaweiInverter,numOfEnergyMeter;
extern uint8_t numofstring;

// inv_flash_data_t inv_data;
esp_err_t err;
uint8_t goodwe_greater_100kwp_status =FALSE;
uint8_t got_response = FALSE;
uint8_t data_recv = FALSE;

interTaskMsg_t msg;
extern QueueHandle_t loggerQueue_h;
QueueHandle_t 	invQueue_h;
unsigned long reading1, reading2,reading3;

float avrage_value;
char data[4096],str_status1[100];
unsigned int gframe, mframe,year, month, date, hour, minute,second; //Frame varaible for giving command
char serialno[50],modelno[20], mload, mresult, SIM_ID[30],IMEI[22], date_time[30], version[5],mresult1,error_code2[20],warning_code1[20];
int power_factor,error_code,str_status,func_code,warning_code,working_mode,run_time,run_status,Inv_Fnflt;
float ph_freq,pv1_vol,pv1_curr,pv2_vol,pv2_curr,pv3_vol,pv3_curr,pv4_vol,pv4_curr,ph1_vol,ph1_curr,ph2_vol,ph2_curr,ph3_vol,ph3_curr,str1,str2,str3,str4,str5,str6,str7,str8,str9,str10,str11,str12,str13,str14,str15,str16,str17,str18,str19,str20,str21,str22,str23,str24,ac_power,life_energy,today_energy,inv_temp,rtime,watt_hr;
// int i,j,k=0;
unsigned int systemcount, supply_voltage, inv_switch, maxsyscount; //
char software_version[10];
unsigned int  adcresult, inverror, invwar, invfalt, fanfalt, strnfalt,status,id;
// long double life_energy, today_energy;	
char sigstrnt[20];
extern void flash_read(char *buff ,flash_sector_t sector);
extern void flash_write(flash_sector_t sector, char *str);
const char *empty_str1 = "";
uint8_t ems_read_status = TRUE;
extern int32_t rssi;
extern uint8_t sync_done;
extern uint8_t callback_status;
extern date_time_t datetime;


// ReturnType result;
// CTS is not used in RS485 Half-Duplex Mode
#define ECHO_TEST_CTS   (UART_PIN_NO_CHANGE)
#define BUF_SIZE        (127)
// #define BAUD_RATE       (CONFIG_ECHO_UART_BAUD_RATE)
#define BAUD_RATE       (9600)
// Read packet timeout
#define PACKET_READ_TICS        (100 / portTICK_PERIOD_MS)
#define ECHO_TASK_STACK_SIZE    (2048)
#define ECHO_TASK_PRIO          (10)
// #define ECHO_UART_PORT          (CONFIG_ECHO_UART_PORT_NUM)
#define ECHO_UART_PORT          (2)
// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define ECHO_READ_TOUT          (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks

const int uart_num = ECHO_UART_PORT;

extern void get_frequency(const int port);
extern void get_phase_voltage(const int port);
extern void get_phase_current(const int port);
extern void get_phase_to_phase_voltage(const int port);
extern void get_power_factor(const int port);
extern void get_active_power(const int port);
extern void get_active_import(const int port);
extern void get_active_export(const int port);
extern void get_active_total(const int port);
extern void all_reg_data(const int port);


static mb_param_request_t request12;
// void init_uart();
void inv_task();
esp_err_t modbus_master_init();
esp_err_t get_goodwe_serialnum(int id);
void get_goodwe_etotal(int id);
void get_goodwe_inv_status(int id);
void get_goodwe_eday(int id);
void get_goodwe_str_status(int id);
int get_goodwe_mppt_voltage_current(int id);
int get_greater_100kwp_goodwe_inverter_data(uint8_t id, uint16_t reg_add,uint16_t reg_len, uint8_t reg_type);
void set_goodwe_10per();
void set_goodwe_30per();
void set_goodwe_60per();
void set_goodwe_100per();
esp_err_t send_ACK(char *msg);
void testing();

extern esp_err_t get_sungrow_inverter_serial_number(uint8_t id, uint16_t reg_add,uint16_t reg_len);
extern int get_sungrow_inverter_data_at_onetime(uint8_t id,uint16_t reg_add,uint16_t reg_len,uint8_t reg_type);

extern esp_err_t get_solis_inverter_serial_number(uint8_t id, uint16_t reg_add,uint16_t reg_len);
extern int get_solis_inverter_data_at_onetime(uint8_t id, uint16_t reg_add,uint16_t reg_len,uint8_t reg_type);

extern esp_err_t get_growatt_inverter_serial_number(uint8_t id, uint16_t reg_add,uint16_t reg_len);
extern int get_growatt_inverter_data_at_onetime(uint8_t id, uint16_t reg_add,uint16_t reg_len,uint8_t reg_type);
extern int get_EMS_meter_readings(uint16_t reg_add,uint16_t reg_len);
extern void inverter_control(char *INV_CONTROL);

uint16_t crc_fun(unsigned char *nData, uint16_t wLength);
void echo_send(const int port, uint8_t *data, uint8_t length);

static const uint16_t wCRCTable[] = {
0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
0X2000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };


uint16_t crc_fun(unsigned char *nData, uint16_t wLength)
{
uint8_t nTemp;
uint16_t wCRCWord = 0xFFFF;

   while (wLength--)
   {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord ^= wCRCTable[nTemp];
   }
   return wCRCWord;

}

void echo_send(const int port, uint8_t *data, uint8_t length)
{
    if (uart_write_bytes(port, (const char *)data, length) != length) {
        ESP_LOGE(TAG10, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}

// float convert_data(uint32_t array){
//     int var = 0;
//     float v1 = 0;
//     // printf("array=%lx\n",array);
//     for (int i = 0; i < 9; i++){
//         if(((array>>(i+23))&1) == 1){
//             var = (var + pow(2,i));
//         }
//     }
//     for(int i = 0; i<23; i++){
//         if(((array>>(23-i)&1)== 1)){
//             v1 = (v1 + (1/(pow(2,i))));
//         }
//     }
//     var = var -127;
//     v1 = (1+v1);
//     avrage_value = v1 *(pow(2,var));
//     // printf("avrage_value=%.2f\n",avrage_value);
//     v1 =0;var = 0;
//     return avrage_value;
// }

void control_CMD1(const int port,uint8_t id,uint8_t fcode, uint16_t reg_add,uint16_t wreg)
{
   uint8_t data100[100];
    request12.slave_addr = id;
	request12.command = fcode;
	request12.reg_start = reg_add;
    request12.reg_size =wreg;// how it changed   what happened
    // while(1){
        vTaskDelay(2000 / portTICK_PERIOD_MS); 
        err = mbc_master_send_request(&request12,&data100);
        // if (err== ESP_OK){
        //     break;
        // }
        // for(int i=0; i<3;i++){
        //     break;
        // }
    // }
}


void control_CMD(const int port,uint8_t id,uint8_t fcode, uint8_t addH, uint8_t addL,uint8_t regH,uint8_t regL,uint8_t Nbit,uint8_t wregH,uint8_t wregL)
{
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    int crc;
    reading1=0;
    reading2=0;
    reading3=0;
    int len =0;
    printf("control command\n");
    uint8_t modbus_request[11];
    modbus_request[0]= id;
    modbus_request[1]= fcode;
    modbus_request[2]= addH;
    modbus_request[3]= addL;
    modbus_request[4]= regH;
    modbus_request[5]= regL;
    modbus_request[6]= Nbit;
    modbus_request[7]= wregH;
    modbus_request[8]= wregL;

    crc =crc_fun(modbus_request, 9);
    modbus_request[9]= crc;
    modbus_request[10]= crc>>8;
    printf("modbus command = 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%2X 0x%.2X 0x%.2X\n",modbus_request[0],modbus_request[1],modbus_request[2],modbus_request[3],modbus_request[4],modbus_request[5],modbus_request[6],modbus_request[7],modbus_request[8],modbus_request[9],modbus_request[10]);
    for(int i = 0;i<5;i++ ){

        echo_send(port, modbus_request, sizeof(modbus_request));
        len = uart_read_bytes(port, data, BUF_SIZE, PACKET_READ_TICS);
        printf("len=%d\n",len);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        if(len == 8){
            break;
        }
    }
    if (len > 7 ) {
        // got_response = TRUE;
        ESP_LOGI(TAG10, "control Received %u bytes:\n  resopnse : 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X \n", len,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
        if (regL == 0x02){
            printf("inverter 2 reg\n");
            reading1 = (data[4]&0x000000FF);
            reading1 |= (data[3] << 8) & 0x0000FF00; 
            reading1 = reading1 << 16 & 0xFFFF0000;
            reading2 = data[6] & 0x000000FF; //Load LSB of higher 16bit
            reading2 |= (data[5] << 8) & 0x0000FF00;
            reading2 = reading2 & 0x0000FFFF;
            reading3 = reading1 | reading2;
        }
        else{
            reading1=0;
            printf("inverter reg\n");
            reading1 = (data[4]&0x000000FF);
            reading1 |= (data[3] << 8) & 0x0000FF00; 
            reading3 = reading1 & 0x0000FFFF;
        }  
        send_ACK("DONE"); 
        printf("inverter configuration done successfully\n");
    }
    else{
        send_ACK("NOT_DONE"); 
        printf("inverter configuration failed\n");
    }

}

// void inverter_off(){
//     for(int i = 1; i<=5;i++ ){
//         #if (GOODWE)
//             if ( goodwe_greater_100kwp_status == TRUE){
//                 control_CMD(uart_num,i,0x10,0xA2,0x08,0x00,0x02,0x00,0x3C);
//             }
//             else{
//                 control_CMD(uart_num,i,0x10,0xA2,0x08,0x00,0x02,0x00,0x3C);
//             }
            
//         #endif
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }    
// }

// void inverter_on(){
//     for(int i = 1; i<=5;i++ ){
//         #if (GOODWE)
//             control_CMD(uart_num,i,0x10,0xA2,0x08,0x00,0x02,0x00,0x3C);
//         #endif
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }  
// }

void inverter_stop10(){
    printf("stop10----------------------------------------\n");
    for(int i = 1; i<=5;i++ ){
        #if (GOODWE)
        if ( goodwe_greater_100kwp_status == TRUE){
                control_CMD(uart_num,i,0x10,0xA2,0x08,0x00,0x01,0x02,0x00,0x0A);
            }
            else{
                control_CMD(uart_num,i,0x10,0x01,0x00,0x00,0x01,0x02,0x00,0x0A);
            }
        #endif
        #if (SOLIS)
        control_CMD(uart_num,i,0x10,0x0B,0xEB,0x00,0x01,0x02,0x03,0xE8);
        #endif 
        #if(SUNGROW)
        control_CMD(uart_num,i,0x10,0x13,0x96,0x00,0x01,0x02,0x00,0x64);
        #endif
        #if(GROWATT)
        control_CMD1(uart_num,i,0x10,0x0003,0x000A);
        #endif 
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }  
}
void inverter_stop30(){
    printf("stop30----------------------------------------\n");
    for(int i = 1; i<=5;i++ ){
        #if(GOODWE)
        if ( goodwe_greater_100kwp_status == TRUE){
                control_CMD(uart_num,i,0x10,0xA2,0x08,0x00,0X01,0x02,0x00,0x1E);
        }
        else{
            control_CMD(uart_num,i,0x10,0x01,0x00,0x00,0X01,0x02,0x00,0x1E);
        }
        #endif
        #if (SOLIS)
        control_CMD(uart_num,i,0x10,0x0B,0xEB,0x00,0x01,0x02,0x0B,0xB8);
        #endif 
        #if(SUNGROW)
        control_CMD(uart_num,i,0x10,0x13,0x96,0x00,0x01,0x02,0x01,0x2C);
        #endif
        #if(GROWATT)
        control_CMD(uart_num,i,0x10,0x00,0x03,0x00,0x01,0x02,0x00,0x1E);
        #endif 
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }  
}
void inverter_stop60(){
    printf("stop60----------------------------------------\n");
    for(int i = 1; i<=5;i++ ){
        #if(GOODWE)
        if ( goodwe_greater_100kwp_status == TRUE){
            control_CMD(uart_num,i,0x10,0xA2,0x08,0x00,0x01,0x02,0x00,0x3C);
        }
        else{
            control_CMD(uart_num,i,0x10,0x01,0x00,0x00,0x01,0x02,0x00,0x3C);
        }
        #endif
        #if(SOLIS)
        control_CMD(uart_num,i,0x10,0x0B,0xEB,0x00,0x01,0x02,0x17,0x10);
        #endif
        #if(SUNGROW)
        control_CMD(uart_num,i,0x10,0x13,0x96,0x00,0x01,0x02,0x02,0x58);
        #endif
        #if(GROWATT)
        control_CMD(uart_num,i,0x10,0x00,0x03,0x00,0x01,0x02,0x00,0x3C);
        #endif 
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }  
}

void inverter_relese(){
    printf("relese----------------------------------------\n");
    for(int i = 1; i<=5;i++ ){
        #if(GOODWE)
        if ( goodwe_greater_100kwp_status == TRUE){
            control_CMD(uart_num,i,0x10,0xA2,0x08,0x00,0x01,0x02,0x00,0x64);
        }
        else{
            control_CMD(uart_num,i,0x10,0x01,0x00,0x00,0x01,0x02,0x00,0x64);
        }
        #endif
        #if(SOLIS)
        control_CMD(uart_num,i,0x10,0x0B,0xEB,0x00,0x01,0x02,0x27,0x10);
        #endif
        #if(SUNGROW)
        control_CMD(uart_num,i,0x10,0x13,0x96,0x00,0x01,0x02,0x03,0xE8);
        #endif
        #if(GROWATT)
        control_CMD(uart_num,i,0x10,0x00,0x03,0x00,0x01,0x02,0x00,0x64);
        #endif 
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }  
}

void inverter_control(char *INV_CONTROL){
    // if(strcmp(INV_CONTROL,"ON")){
    //     inverter_off();
    // }
    // else if(strcmp(INV_CONTROL,"OFF")){
    //     inverter_on();
    // }
    printf("control = %s\n", INV_CONTROL);
    if(strcmp(INV_CONTROL,"STOP10") == 0){
        printf("stop10----------------\n");
        inverter_stop10();
    }
    else if(strcmp(INV_CONTROL,"STOP30") == 0){
        printf("stop30----------------\n");
        inverter_stop30();
    }
    else if(strcmp(INV_CONTROL,"STOP60") == 0){
        printf("stop60----------------\n");
        inverter_stop60();
    }
    else if(strcmp(INV_CONTROL,"RELEASE") == 0){
        printf("stop100----------------\n");
        inverter_relese();
    }
    else if(strcmp(INV_CONTROL,"RESETRMS") == 0){
        ESP_LOGE(TAG10, "$$$$$$$ Please wait restarting RMS  $$$$$$$\n");
        esp_restart();
    }
    else if (strcmp(INV_CONTROL,"RESEND") == 0){
        printf("implimentation required\n");
    }
    // else if (strcmp(INV_CONTROL,"SUNGROWGOODWE") == 0){
    //     printf("SUNGROWGOODWE\n");
    //     #define GROWATT     0
    //     #define SOLIS       0
    //     #define GOODWE      1
    //     #define SUNGROW     1
    // }
    // else if (strcmp(INV_CONTROL,"SUNGROWGROWATT") == 0){
    //     printf("SUNGROWGROWATT\n");
    //     #define GROWATT     1
    //     #define SOLIS       0
    //     #define GOODWE      0
    //     #define SUNGROW     1
    // }
    // else if (strcmp(INV_CONTROL,"SUNGROWSOLIS") == 0){
    //     printf("SUNGROWSOLIS\n");
    //     #define GROWATT     0
    //     #define SOLIS       1
    //     #define GOODWE      0
    //     #define SUNGROW     1
    // }
    // else if (strcmp(INV_CONTROL,"GROWATTSOLIS") == 0){
    //     printf("GROWATTSOLIS\n");
    //     #define GROWATT     1
    //     #define SOLIS       1
    //     #define GOODWE      0
    //     #define SUNGROW     0
    // }
    // else if (strcmp(INV_CONTROL,"GROWATTGOODWE") == 0){
    //     printf("GROWATTGOODWE\n");
    //     #define GROWATT     1
    //     #define SOLIS       0
    //     #define GOODWE      1
    //     #define SUNGROW     0
    // }
    // else if (strcmp(INV_CONTROL,"SOLISGOODWE") == 0){
    //     printf("SOLISGOODWE\n");
    //     #define GROWATT     0
    //     #define SOLIS       1
    //     #define GOODWE      1
    //     #define SUNGROW     0
    // }
}




// void inverter_control(char *INV_CONTROL){
//     printf("control = %s\n", INV_CONTROL);
//     #if (GOODWE)
//     if(strcmp(INV_CONTROL,"STOP10") == 0){
//         printf("stop10----------------\n");
//         set_goodwe_10per();
//     }
//     else if(strcmp(INV_CONTROL,"STOP30") == 0){
//         printf("stop30----------------\n");
//         set_goodwe_30per();
//     }
//     else if(strcmp(INV_CONTROL,"STOP60") == 0){
//         printf("stop60----------------\n");
//         set_goodwe_60per();
//     }
//     else if(strcmp(INV_CONTROL,"RELESE") == 0){
//         printf("stop100----------------\n");
//         set_goodwe_100per();
//     }
//     else if(strcmp(INV_CONTROL,"RESETRMS") == 0){
//         ESP_LOGE(TAG10, "$$$$$$$ Please wait restarting RMS  $$$$$$$\n");
//         esp_restart();
//     }
//     else if (strcmp(INV_CONTROL,"RESEND") == 0){
//         printf("implimentation required\n");
//     }
//     #endif
// }


esp_err_t modbus_master_init(void)
{
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
            .port = MB_PORT_NUM,
// #if CONFIG_MB_COMM_MODE_ASCII
//             .mode = MB_MODE_ASCII,
// #elif CONFIG_MB_COMM_MODE_RTU
            .mode = MB_MODE_RTU,
// #endif
            .baudrate = MB_DEV_SPEED,
            .parity = MB_PARITY_NONE
    };
    void* master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MB_RETURN_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, TAG11,
                                "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG11,
                            "mb controller initialization fail, returns(0x%x).",
                            (uint32_t)err);
    err = mbc_master_setup((void*)&comm);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG11,
                            "mb controller setup fail, returns(0x%x).",
                            (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, MB_TEST_TXD, MB_TEST_RXD,
                              MB_TEST_RTS, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG11,
            "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);

    err = mbc_master_start();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG11,
                            "mb controller start fail, returns(0x%x).",
                            (uint32_t)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG11,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    // err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    // MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG11,
    //                             "mb controller set descriptor fail, returns(0x%x).",
    //                             (uint32_t)err);
    ESP_LOGI(TAG11, "Modbus master stack initialized...");
    return err;
}


void init_uart(){
    gpio_reset_pin(LED1);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_level(LED1,HIGH);
    gpio_reset_pin(MB_TEST_RTS);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(MB_TEST_RTS, GPIO_MODE_OUTPUT);
    gpio_set_level(MB_TEST_RTS,HIGH);
    gpio_reset_pin(RS485_5V_PIN_EN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(RS485_5V_PIN_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(RS485_5V_PIN_EN,HIGH);
    gpio_reset_pin(RS485_SE_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(RS485_SE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RS485_SE_PIN,HIGH);

    
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Set UART log level
    esp_log_level_set(TAG10, ESP_LOG_INFO);

    ESP_LOGI(TAG10, "Start RS485 application test and configure UART.");

    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_LOGI(TAG10, "UART set pins, mode and install driver.");

    // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_pin(uart_num, MB_TEST_TXD, MB_TEST_RXD, MB_TEST_RTS, ECHO_TEST_CTS));

    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));

    // Set read timeout of UART TOUT feature
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, ECHO_READ_TOUT));

    // Allocate buffers for UART
    // uint8_t* data = (uint8_t*) malloc(BUF_SIZE);

    ESP_LOGI(TAG10, "UART start recieve loop.\r\n");
}



void inv_task(){
    invQueue_h = xQueueCreate(20, sizeof(interTaskMsg_t));
    printf("inverter task  ----------------------------------\n");
	if (invQueue_h == NULL) {
		ESP_LOGE(TAG10, "Unable to create primary queue.");
		return;
	}
	for (;;) {
		if (xQueueReceive(invQueue_h, &msg, pdMS_TO_TICKS(20000)))
        {
            if( sync_done == TRUE ){
                printf("inverter generation\n");
                // char flash_buff[20]={"\0"};
                // char flash_buff1[20]={"\0"};
                // char flash_buff2[20]={"\0"};
                // flash_read(flash_buff,INVERTER_TYPE_ADDR);
                // sprintf(inverter_type,"%s",flash_buff);
                // printf("inverter_type=%s\n",inverter_type);
                // flash_read(flash_buff1,NUMBER_OF_INVERTER_ADDR);
                // sscanf(flash_buff1,"%hhd",&numOfGoodweInverter);
                // printf("numofinverter=%hhd\n",numOfGoodweInverter);
                // flash_read(flash_buff2,NUMBER_OF_STRING_ADDR);
                // sscanf(flash_buff2,"%hhd",&numofstring);
                // printf("numofstring=%hhd\n",numofstring);
                // printf("numofinverter=%d\n",numofinverter); 
                // send_cmd_MB(uart_num,1,0x03,0x00,0x01,0x00,0x02);
                #if (GOODWE)
                for(int i=1;i<6;i++){
                    // while(1){
                        esp_err_t err=get_goodwe_serialnum( i);
                        if (got_response == FALSE){
                            // break;
                            printf("not responding\n");
                        }
                        if (got_response == TRUE && goodwe_greater_100kwp_status == FALSE){
                            get_goodwe_etotal(i); //life energy and runtime     
                            get_goodwe_eday(i); 
                            // get_goodwe_inv_status(i);
                            get_goodwe_str_status(i);
                            int rt_value =get_goodwe_mppt_voltage_current(i);
                            got_response =  FALSE;
                            if (rt_value == 1 ){
                                // break;
                                printf("read successfully\n");
                            }
                        }
                        if(got_response == TRUE && goodwe_greater_100kwp_status == TRUE){ // OSWAL
                            get_greater_100kwp_goodwe_inverter_data(i,0x7D10,0x0020,1);// MPPT & STRING
                            get_greater_100kwp_goodwe_inverter_data(i,0x7D45,0x0009,2);//GRID VLTG &CURR
                            get_greater_100kwp_goodwe_inverter_data(i,0x7D50,0x0002,3);//AC PWR
                            get_greater_100kwp_goodwe_inverter_data(i,0x7D55,0x0004,4);//FREQ &TEMP
                            get_greater_100kwp_goodwe_inverter_data(i,0x7D6A,0x0002,5);//TOTAL GEN
                            get_greater_100kwp_goodwe_inverter_data(i,0x7D72,0x0002,6);//EDAY
                            get_greater_100kwp_goodwe_inverter_data(i,0x8BA5,0x0002,7);//WORK HOUR
                           int rt_value = get_greater_100kwp_goodwe_inverter_data(i,0x8BAE,0x0001,8);//WORK MODE
                            got_response =  FALSE;
                            if (rt_value == 1 ){
                                // break;
                                printf("read successfully\n");
                            }
                        }
                        // vTaskDelay(10000/portTICK_PERIOD_MS);
                    // }
                }
                #endif
                #if (SUNGROW)
                for(int i = 1; i <= 5 ;i++){
                    get_sungrow_inverter_serial_number(i,4989,10);
                    if(got_response == TRUE){
                        get_sungrow_inverter_data_at_onetime(i,5002,6,1);
                        get_sungrow_inverter_data_at_onetime(i,5010,14,2);
                        get_sungrow_inverter_data_at_onetime(i,5030,6,3);
                        get_sungrow_inverter_data_at_onetime(i,5037,8,4);
                        get_sungrow_inverter_data_at_onetime(i,7012,24,5);
                        got_response =  FALSE;
                    }
                }
                #endif
                #if (SOLIS)
                for(int i = 1; i <= 5 ;i++){
                    get_solis_inverter_serial_number(i,0x0BF4,0x0004);
                    if(got_response == TRUE){
                        get_solis_inverter_data_at_onetime(i,0X0BBC,0x0028,1);
                        get_solis_inverter_data_at_onetime(i,0X0BCD,0x0004,2);
                        get_solis_inverter_data_at_onetime(i,0X0BD9,0x000B,3);
                        get_solis_inverter_data_at_onetime(i,0X0BFF,0x0001,4);
                        get_solis_inverter_data_at_onetime(i,0X0CE4,0x0010,5);
                        got_response =  FALSE;
                    }
                }
                #endif
                // #if (DELTA)
                // get_delta_inverter_data(inverter_type,numofinverter); 
                // #`
                // #if (SMA)
                // get_sma_inverter_data(inverter_type,numofinverter); 
                // #endif
                #if (GROWATT)
                for(int i = 1; i <= 5 ;i++){
                    printf("reading from GROWATT\n");
                    get_growatt_inverter_serial_number(i,0x0017,0x0005);
                    if(got_response == TRUE){
                        get_growatt_inverter_data_at_onetime(i,0x0000,0x0009,1);
                        get_growatt_inverter_data_at_onetime(i,0x0023,0x000D,2);
                        get_growatt_inverter_data_at_onetime(i,0x0035,0x0006,3);
                        get_growatt_inverter_data_at_onetime(i,0x005D,0x0001,4);
                        get_growatt_inverter_data_at_onetime(i,0x0069,0x0001,5);
                        get_growatt_inverter_data_at_onetime(i,0x006E,0x0002,6);
                        get_growatt_inverter_data_at_onetime(i,0x008F,0x0031,7);
                        // get_growatt_inverter_data_at_onetime(i,0x0C24,0x0001,8);
                        got_response =  FALSE;
                    }
                }
                #endif
                #if (SINENG_ELECTRIC)
                #endif
                #if (HUAWEI)
                #endif
                #if (MODE_SCHNE_CL20_3P)
                #endif
                #if (EMS)
                // for(int i =0; i<5; i++){
                    printf("EMS meter1\n");
                    get_frequency(uart_num);
                    if(got_response == TRUE){
                        // get_phase_voltage(uart_num);
                        // get_phase_current(uart_num);
                        // get_phase_to_phase_voltage(uart_num);
                        // get_power_factor(uart_num);
                        // get_active_power(uart_num);
                        // get_active_import(uart_num);
                        // get_active_export(uart_num);
                        // get_active_total(uart_num);
                        all_reg_data(uart_num);
                        got_response =  FALSE;
                    }
                    // get_phase_voltage(uart_num);
                    // get_phase_current(uart_num);
                    // get_phase_to_phase_voltage(uart_num);
                    // get_power_factor(uart_num);
                    // get_active_power(uart_num);
                    // get_active_import(uart_num);
                    // get_active_export(uart_num);
                    // get_active_total(uart_num);
                    // get_EMS_meter_readings(0x0001,0x0002); 
                    printf("EMS meter2\n"); 
                // }                
                #endif
                #if(TESTING )
                testing();
                #endif
                callback_status = TRUE;
            }
        }
    }
}