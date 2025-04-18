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

#define TAG7   "GOODWE"




static mb_param_request_t request;

extern esp_err_t err;

#define PACKET_READ_TICS        (100 / portTICK_PERIOD_MS)
extern char dev_mac[30],data[4096],str_status1[100];
extern uint16_t crc_fun(unsigned char *nData, uint16_t wLength);
extern char serialno[50],modelno[20], mload, mresult, SIM_ID[30],IMEI[22], date_time[30];
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
// uint8_t j=0;

extern esp_err_t get_goodwe_serialnum(int id);
extern void get_goodwe_etotal(int id);
extern void get_goodwe_inv_status(int id);
extern void get_goodwe_eday(int id);
extern void get_goodwe_str_status(int id);
extern int get_goodwe_mppt_voltage_current(int id);
extern void set_goodwe_10per();
extern void set_goodwe_30per();
extern void set_goodwe_60per();
extern void set_goodwe_100per();
extern int get_greater_100kwp_goodwe_inverter_data(uint8_t id, uint16_t reg_add,uint16_t reg_len, uint8_t reg_type);
extern void testing();
esp_err_t send_ACK(char *msg);


#if(TESTING)
void testing(){
    printf("testing-------------------------------------------------\n");
    strcpy(serialno,"testing123456");
    rssi = -20;
    pv1_curr=2.1;
    pv1_vol =670.2;
    pv2_curr=3.9;
    pv2_vol =698.6;
    ph1_curr =8.9;
    ph1_vol =235.3;
    ph2_curr =9.1;
    ph2_vol =230.6;
    ph3_curr=8.5;
    ph3_vol =228.5;
    working_mode=0;
    warning_code =0;
    error_code =0;
    // strcpy(warning_code,"Normal");
    // strcpy(error_code,"Normal");
    inv_temp =50;
    run_status = 1;
    life_energy =1020;
    today_energy =26;
    run_status =1;
    ac_power =37;
    run_time=190;
    strcpy(str_status1,"0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1"); 
    int id = 1;

    if( sync_done == TRUE && ((pv1_vol < 1000) && (pv2_vol <1000) && (ph1_vol < 500) && (ph2_vol < 500) && (ph3_vol < 500) && (ac_power < 100))){
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
        ESP_LOGI(TAG7, "DATE time = %d-%d-%d   %d:%d:%d",datetime.day,datetime.month,datetime.year,datetime.hours,datetime.mins,datetime.secs);
        interTaskMsg_t msg;
        strcpy(data,"");
        sprintf(data,"\"RVolt\":\"0\",\"RIMAID\":\"%s\",\"RSignal\":\"%d\",\"ISth\":\"0\",\"ID\":\"%d\",\"InvSlno\":\"0%s\",\"MPPT1_DCV\":\"%.2f\",\"MPPT1_DCA\":\"%.2f\",\"MPPT2_DCV\":\"%.2f\",\"MPPT2_DCA\":\"%.2f\",\"Ph1ACV\":\"%.2f\",\"Ph1ACA\":\"%.2f\",\"Ph2ACV\":\"%.2f\",\"Ph2ACA\":\"%.2f\",\"Ph3ACV\":\"%.2f\",\"Ph3ACA\":\"%.2f\",\"InvAC_P\":\"%.2f\",\"Inv_Fr\":\"%.2f\",\"Inv_Wh\":\"%.2f\",\"Inv_Nrg\":\"%.2f\",\"Inv_Rt\":\"%d\",\"Inv_LE\":\"%.2f\",\"Inv_Sts\":\"%d\",\"Inv_Tpr\":\"%.2f\",\"Inv_Err\":\"%d\",\"Inv_Wrn\":\"%d\",\"Inv_Fnflt\":\"0X%X\",\"Inv_StFlt\":\"%s\"",dev_mac,rssi,id,serialno,pv1_vol,pv1_curr,pv2_vol,pv2_curr,ph1_vol,ph1_curr,ph2_vol,ph2_curr,ph3_vol,ph3_curr,ac_power,ph_freq,ac_power,today_energy,run_time,life_energy,run_status,inv_temp,error_code,warning_code,working_mode,str_status1);
        printf("\n\n\n0.Rdate = %u-%u-%u %u:%u\n1.RVolt = 0\n2.RIMAID = %s\n3.RSignal = %d\n4.ISth = 0\n5.ID = %d\n6.InvSlno = 0%s\n7.MPPT1_DCV = %.2f\n8.MPPT1_DCA = %.2f\n9.MPPT2_DCV = %.2f\n10.MPPT2_DCA = %.2f\n11.Ph1ACV = %.2f\n12.Ph1ACA = %.2f\n13.Ph2ACV = %.2f\n14.Ph2ACA = %.2f\n15.Ph3ACV = %.2f\n16.Ph3ACA = %.2f\n17.InvAC_P = %.2f\n18.Inv_Fr = %.2f\n19.Inv_Wh = %.2f\n20.Inv_Nrg = %.2f\n21.Inv_Rt = %d\n22.Inv_LE = %.2f\n23.Inv_Sts = %d\n24.Inv_Tpr = %.2f\n25.Inv_Err = %d\n26.Inv_Wrn = %d\n27.Inv_Fnflt = 0X%X\n28.Inv_StFlt = %s\n",datetime.day,datetime.month,datetime.year,datetime.hours,datetime.mins,dev_mac,rssi,id,serialno,pv1_vol,pv1_curr,pv2_vol,pv2_curr,ph1_vol,ph1_curr,ph2_vol,ph2_curr,ph3_vol,ph3_curr,ac_power,ph_freq,ac_power,today_energy,run_time,life_energy,run_status,inv_temp,error_code,warning_code,working_mode,str_status1);
        msg.msgId = DATA_ACQ_MSG;
        msg.data = data;
        xQueueSend(loggerQueue_h, (void *) &msg, 0);
        got_response =FALSE;
        memset(serialno, 0, sizeof(serialno));
    }
}
#endif




void set_goodwe_10per(){
    uint8_t stop10[10];
    for(int i=1;i<6;i++){
        request.slave_addr = i;
        request.command = 0x10;
        request.reg_start = 0x0100;
        request.reg_size = 0x0001;
        while(1){
            vTaskDelay(10000 / portTICK_PERIOD_MS); 
            err = mbc_master_send_request(&request,&stop10);
            if (err== ESP_OK){
                send_ACK("DONE"); 
                printf("inverter configured successfully\n");
                break;
            }
            else{
                send_ACK("NOT_DONE"); 
                printf("inverter configured failed\n");
                break;
            }
        }
    }
}

void set_goodwe_30per(){
    uint8_t stop30[10];
    for(int i=1;i<6;i++){
        request.slave_addr = i;
        request.command = 0x10;
        request.reg_start = 0x0100;
        request.reg_size = 0x0001;
        while(1){
            vTaskDelay(10000 / portTICK_PERIOD_MS); 
            err = mbc_master_send_request(&request,&stop30);
            if (err== ESP_OK){
                send_ACK("DONE"); 
                printf("inverter configured successfully\n");
                break;
            }
            else{
                send_ACK("NOT_DONE"); 
                printf("inverter configured failed\n");
                break;
            }
        }
    }
}
void set_goodwe_60per(){
    uint8_t stop60[10];
    for(int i=1;i<6;i++){
        request.slave_addr = i;
        request.command = 0x10;
        request.reg_start = 0x0100;
        request.reg_size = 0x0001;
        while(1){
            vTaskDelay(10000 / portTICK_PERIOD_MS); 
            err = mbc_master_send_request(&request,&stop60);
            if (err== ESP_OK){
                send_ACK("DONE"); 
                printf("inverter configured successfully\n");
                break;
            }
            else{
                send_ACK("NOT_DONE"); 
                printf("inverter configured failed\n");
                break;
            }
        }
    }
}

void set_goodwe_100per(){
    uint8_t stop100[10];
    for(int i=1;i<6;i++){
        request.slave_addr = i;
        request.command = 0x10;
        request.reg_start = 0x0100;
        request.reg_size = 0x0001;
        while(1){
            vTaskDelay(10000 / portTICK_PERIOD_MS); 
            err = mbc_master_send_request(&request,&stop100);
            if (err== ESP_OK){
                send_ACK("DONE"); 
                printf("inverter configured successfully\n");
                break;
            }
            else{
                send_ACK("NOT_DONE"); 
                printf("inverter configured failed\n");
                break;
            }
        }
    }
}

esp_err_t get_goodwe_serialnum(int id){
    uint8_t j=0;
    uint8_t slnum[50]={0};
    request.slave_addr = id;
	request.command = 0x03;
	request.reg_start = 0x0200;
	request.reg_size = 0x0008;
    while(1){
        j++;
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
        memset(slnum, 0, sizeof(slnum));
        err = mbc_master_send_request(&request,&slnum);
        sprintf(serialno,"%C%C%C%C%C%C%C%C%C%C%C%C%C%C%C%C",slnum[1],slnum[0],slnum[3],slnum[2],slnum[5],slnum[4],slnum[7],slnum[6],slnum[9],slnum[8],slnum[11],slnum[10],slnum[13],slnum[12],slnum[15],slnum[14]);
        printf("j = %d inv id = %d  serial num = %S\n",j,id,serialno);
        if (strlen(serialno)>8 && err == ESP_OK){
            j=0;
            got_response =TRUE;
            if ((strstr(serialno,"9100")!= NULL) || (strstr(serialno,"9120")!= NULL) || (strstr(serialno,"9200")!= NULL) || (strstr(serialno,"5100")!= NULL) || (strstr(serialno,"5120")!= NULL) || (strstr(serialno,"9200")!= NULL) ){
                goodwe_greater_100kwp_status = TRUE;
            }
            else{
                goodwe_greater_100kwp_status = FALSE;
            }
            return err;
            break;
        }
        // if(err == ESP_ERR_TIMEOUT || err == ESP_ERR_NOT_FOUND || j >= 3 || err == ESP_ERR_INVALID_RESPONSE){
        if(err == ESP_ERR_TIMEOUT || err == ESP_ERR_NOT_FOUND || j >= 3 ){
            printf("error =%X\n",err);
            j=0;
            got_response =FALSE;
            return err;
            break;
        }
        got_response =FALSE;
    }
}

void get_goodwe_etotal(int id){
    uint8_t data6[10];
    request.slave_addr = id;
	request.command = 0x03;
	request.reg_start = 0x0222;
	request.reg_size = 0x0004;
    while(1){
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
        err = mbc_master_send_request(&request,&data6);
        if (err== ESP_OK){
            break;
        }
    }
    
    reading1 = data6[0]&0x000000FF;
    reading1 |= (data6[1] << 8) & 0x0000FF00;
    reading1 = (reading1 << 16) & 0xFFFF0000;
    reading2 = data6[2] & 0x000000FF; //Load LSB of higher 16bit
    reading2 |= (data6[3] << 8) & 0x0000FF00;
    reading2 = reading2 & 0x0000FFFF;
    reading3 = reading1 | reading2;
    life_energy = (float)reading3/10;
    printf("life energy =%.2f\n",life_energy);
    
    reading1 = data6[4]&0x000000FF;
    reading1 |= (data6[5] << 8) & 0x0000FF00;
    reading1 = (reading1 << 16) & 0xFFFF0000;
    reading2 = data6[6] & 0x000000FF; //Load LSB of higher 16bit
    reading2 |= (data6[7] << 8) & 0x0000FF00;
    reading2 = reading2 & 0x0000FFFF;
    reading3 = reading1 | reading2;
    run_time = (float)reading3;
    printf("runtime=%d\n",run_time);
    memset(data6, 0, sizeof(data6));
}
void get_goodwe_inv_status(int id){
    uint8_t data3[10]; 
    request.slave_addr = id;
	request.command = 0x03;
	request.reg_start = 0x0234;
	request.reg_size = 0x0001;
    while(1){
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
        err = mbc_master_send_request(&request,&data3);
        if (err== ESP_OK){
            break;
        }
    }
    reading1 = data3[0]&0x000000FF;
    reading1 |= (data3[1] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;   
    int warn =reading1;
    reading1=0;
    if(warn == 2){
        warning_code = warn;
    }
    if (warn == 0){
        warning_code = 1;
    }
    if (warn == 1){
        warning_code = 0;
    }
    printf("warning_code=%d\n",warning_code);//status from data sheet
}
void get_goodwe_eday(int id){
    uint8_t data4[10]; 
    request.slave_addr = id;
	request.command = 0x03;
	request.reg_start = 0x0236;
	request.reg_size = 0x0001;

    while(1){
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
        err = mbc_master_send_request(&request,&data4);
        if (err== ESP_OK){
            break;
        }
    }
    
    
    reading1 = data4[0]&0x000000FF;
    reading1 |= (data4[1] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
   
    today_energy = (float)reading1/10;
    printf("today energy =%.2f\n",today_energy);
     memset(data4, 0, sizeof(data4));
}
void get_goodwe_str_status(int id){
    uint8_t data5[40]; 
    request.slave_addr = id;
	request.command = 0x03;
	request.reg_start = 0x035B;
	request.reg_size = 0x0011;
    while(1){
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
        err = mbc_master_send_request(&request,&data5);
        if (err== ESP_OK){
            break;
        }
    }
    
    reading1 = data5[0]&0x000000FF;
    reading1 |= (data5[1] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str1 = (float)reading1/10;
    if(str1 > 100){
        str1=0;
    }
    reading1=0;
    reading1 = data5[2]&0x000000FF;
    reading1 |= (data5[3] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str2 = (float)reading1/10;
    if(str2 > 100){
        str2=0;
    }
    reading1=0;
    reading1 = data5[4]&0x000000FF;
    reading1 |= (data5[5] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str3 = (float)reading1/10;
    if(str3 > 100){
        str3=0;
    }
    reading1=0;
    reading1 = data5[6]&0x000000FF;
    reading1 |= (data5[7] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str4 = (float)reading1/10;
    if(str4 > 100){
        str4=0;
    }
    reading1=0;
    reading1 = data5[8]&0x000000FF;
    reading1 |= (data5[9] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str5 = (float)reading1/10;
    if(str5 > 100){
        str5=0;
    }
    reading1=0;
    reading1 = data5[10]&0x000000FF;
    reading1 |= (data5[11] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str6 = (float)reading1/10;
    if(str6 > 100){
        str6=0;
    }
    reading1=0;
    reading1 = data5[12]&0x000000FF;
    reading1 |= (data5[13] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str7 = (float)reading1/10;
    if(str7 > 100){
        str7=0;
    }
    reading1=0;
    reading1 = data5[14]&0x000000FF;
    reading1 |= (data5[15] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str8 = (float)reading1/10;
    if(str8 > 100){
        str8=0;
    }
    reading1=0;
    reading1 = data5[16]&0x000000FF;
    reading1 |= (data5[17] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str9 = (float)reading1/10;
    if(str9 > 100){
        str9=0;
    }
    reading1=0;
    reading1 = data5[18]&0x000000FF;
    reading1 |= (data5[19] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str10 = (float)reading1/10;
    if(str10 > 100){
        str10=0;
    }
    reading1=0;
    reading1 = data5[20]&0x000000FF;
    reading1 |= (data5[21] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str11 = (float)reading1/10;
    if(str11 > 100){
        str11=0;
    }
    reading1=0;
    reading1 = data5[22]&0x000000FF;
    reading1 |= (data5[23] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str12 = (float)reading1/10;
    if(str12 > 100){
        str12=0;
    }
    reading1=0;
    reading1 = data5[24]&0x000000FF;
    reading1 |= (data5[25] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str13 = (float)reading1/10;
    if(str13 > 100){
        str13=0;
    }
    reading1=0;
    reading1 = data5[26]&0x000000FF;
    reading1 |= (data5[27] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str14 = (float)reading1/10;
    if(str14 > 100){
        str14=0;
    }
    reading1=0;
    reading1 = data5[28]&0x000000FF;
    reading1 |= (data5[29] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str15 = (float)reading1/10;
    if(str15 > 100){
        str15=0;
    }
    reading1=0;
    reading1 = data5[30]&0x000000FF;
    reading1 |= (data5[31] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    str16 = (float)reading1/10;
    if(str16 > 100){
        str16=0;
    }
    reading1=0;
    sprintf(str_status1,"%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",str1,str2,str3,str4,str5,str6,str7,str8,str9,str10,str11,str12,str12,str14,str15,str16);
    // str_status = reading1;
    printf("str_status =%S\n",str_status1);
    memset(data5, 0, sizeof(data5));
}
int get_goodwe_mppt_voltage_current(int id){
    uint8_t data2[80]; 

    request.slave_addr = id;
	request.command = 0x03;
	request.reg_start = 0x0300;
	request.reg_size = 0x0021;
    while(1){
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
        err = mbc_master_send_request(&request,&data2);
        if (err== ESP_OK){
            break;
        }
    }
    reading1 = data2[0]&0x000000FF;
    reading1 |= (data2[1] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    pv1_vol =(float)reading1/10;
    reading1=0;
    printf("Vpv1=%.2f\n",pv1_vol);
    reading1 = data2[2]&0x000000FF;
    reading1 |= (data2[3] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    pv2_vol =(float)reading1/10;
    reading1=0;
    printf("Vpv2=%.2f\n",pv2_vol);
    reading1 = data2[4]&0x000000FF;
    reading1 |= (data2[5] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    pv1_curr =(float)reading1/10;
    reading1=0;
    printf("Ipv1=%.2f\n",pv1_curr);
    reading1 = data2[6]&0x000000FF;
    reading1 |= (data2[7] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    pv2_curr =(float)reading1/10;
    reading1=0;
    printf("Ipv2=%.2f\n",pv2_curr);
    reading1 = data2[8]&0x000000FF;
    reading1 |= (data2[9] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    ph1_vol =(float)reading1/10;
    reading1=0;
    printf("acV1=%.2f\n",ph1_vol);
    reading1 = data2[10]&0x000000FF;
    reading1 |= (data2[11] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    ph2_vol =(float)reading1/10;
    reading1=0;
    printf("acV2=%.2f\n",ph2_vol);
    reading1 = data2[12]&0x000000FF;
    reading1 |= (data2[13] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    ph3_vol =(float)reading1/10;
    reading1=0;
    printf("acV3=%.2f\n",ph3_vol);
    reading1 = data2[14]&0x000000FF;
    reading1 |= (data2[15] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    ph1_curr =(float)reading1/10;
    reading1=0;
    printf("acI1=%.2f\n",ph1_curr);
    reading1 = data2[16]&0x000000FF;
    reading1 |= (data2[17] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    ph2_curr =(float)reading1/10;
    reading1=0;
    printf("acI=%.2f\n",ph2_curr);
    reading1 = data2[18]&0x000000FF;
    reading1 |= (data2[19] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    ph3_curr =(float)reading1/10;
    reading1=0;
    printf("acI3=%.2f\n",ph3_curr);
    reading1 = data2[20]&0x000000FF;
    reading1 |= (data2[21] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    ph_freq =(float)reading1/100;
    reading1=0;
    printf("freq=%.2f\n",ph_freq);
    reading1 = data2[26]&0x000000FF;
    reading1 |= (data2[27] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    ac_power =(float)reading1/1000;
    reading1=0;
    printf("ac_power=%.2f\n",ac_power);
    reading1 = data2[28]&0x000000FF;
    reading1 |= (data2[29] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    run_status =reading1;
    reading1=0;
    printf("run_status=%X\n",run_status);// work mode from datasheet
    reading1 = data2[30]&0x000000FF;
    reading1 |= (data2[31] << 8) & 0x0000FF00;
    reading1 = reading1 & 0x0000FFFF;
    inv_temp =(float)reading1/10;
    reading1=0;
    printf("temp=%.2f\n",inv_temp);

    reading1 = data2[32]&0x000000FF;
    reading1 |= (data2[33] << 8) & 0x0000FF00;
    reading1 = reading1 << 16 & 0xFFFF0000;
    reading2 = data2[34] & 0x000000FF; //Load LSB of higher 16bit
    reading2 |= (data2[35] << 8) & 0x0000FF00;
    reading2 = reading2 & 0x0000FFFF;
    reading3 = reading1 | reading2;
    int error= reading3; //error from data sheet
    reading1=0;reading2=0;reading3=0;
    if (((error>>0)&1)==1){
        error_code =300; 
    }
    if (((error>>1)&1)==1){
        error_code =301; 
    }
    if (((error>>2)&1)==1){
        error_code =302;  
    }
    if (((error>>3)&1)==1){
        error_code =303; 
    }
    if (((error>>4)&1)==1){
        error_code =304;  
    }
    if (((error>>6)&1)==1){
        error_code =305;  
    }
    if (((error>>7)&1)==1){
        error_code =306;
    }
    if (((error>>8)&1)==1){
        error_code =307;
    }
    if (((error>>9)&1)==1){
        error_code =308; 
    }
    if (((error>>10)&1)==1){
        error_code =309;
    }
    if (((error>>11)&1)==1){
        error_code =310;
    }
    if (((error>>12)&1)==1){
        error_code =311;
    }
    if (((error>>13)&1)==1){
        error_code =312;
    }
    if (((error>>14)&1)==1){
        error_code =313;
    }
    if (((error>>15)&1)==1){
        error_code =314;
    }
    if (((error>>16)&1)==1){
        error_code =315;
    }
    if (((error>>17)&1)==1){
        error_code =316;
    }
    if (((error>>18)&1)==1){
        error_code =317;
    }
    if (((error>>19)&1)==1){
        error_code =318;
    }

    if (((error>>21)&1)==1){
        error_code =319;
    }
    if (((error>>22)&1)==1){
        error_code =320;
    }
    if (((error>>23)&1)==1){
        error_code =321;
    }
    if (((error>>25)&1)==1){
        error_code =322;
    }
    if (((error>>29)&1)==1){
        error_code =323;
    }
    if (((error>>31)&1)==1){
        error_code =324;
    }
    vTaskDelay(2000/portTICK_PERIOD_MS);
    // ESP_ERROR_CHECK(mbc_master_destroy());
    if( sync_done == TRUE && ((pv1_vol < 1000) && (pv2_vol <1000) && (ph1_vol < 500) && (ph2_vol < 500) && (ph3_vol < 500) && (ac_power < 100))){
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
        ESP_LOGI(TAG7, "DATE time = %d-%d-%d   %d:%d:%d",datetime.day,datetime.month,datetime.year,datetime.hours,datetime.mins,datetime.secs);
        interTaskMsg_t msg;
        strcpy(data,"");
        sprintf(data,"\"RVolt\":\"0\",\"RIMAID\":\"%s\",\"RSignal\":\"%d\",\"ISth\":\"0\",\"ID\":\"%d\",\"InvSlno\":\"0%s\",\"MPPT1_DCV\":\"%.2f\",\"MPPT1_DCA\":\"%.2f\",\"MPPT2_DCV\":\"%.2f\",\"MPPT2_DCA\":\"%.2f\",\"Ph1ACV\":\"%.2f\",\"Ph1ACA\":\"%.2f\",\"Ph2ACV\":\"%.2f\",\"Ph2ACA\":\"%.2f\",\"Ph3ACV\":\"%.2f\",\"Ph3ACA\":\"%.2f\",\"InvAC_P\":\"%.2f\",\"Inv_Fr\":\"%.2f\",\"Inv_Wh\":\"%.2f\",\"Inv_Nrg\":\"%.2f\",\"Inv_Rt\":\"%d\",\"Inv_LE\":\"%.2f\",\"Inv_Sts\":\"%d\",\"Inv_Tpr\":\"%.2f\",\"Inv_Err\":\"%d\",\"Inv_Wrn\":\"%d\",\"Inv_Fnflt\":\"%X\",\"Inv_StFlt\":\"%s\"",dev_mac,rssi,id,serialno,pv1_vol,pv1_curr,pv2_vol,pv2_curr,ph1_vol,ph1_curr,ph2_vol,ph2_curr,ph3_vol,ph3_curr,ac_power,ph_freq,ac_power,today_energy,run_time,life_energy,run_status,inv_temp,error_code,warning_code,working_mode,str_status1);
        printf("\n\n\n0.Rdate = %u-%u-%u %u:%u\n1.RVolt = 0\n2.RIMAID = %s\n3.RSignal = %d\n4.ISth = 0\n5.ID = %d\n6.InvSlno = 0%s\n7.MPPT1_DCV = %.2f\n8.MPPT1_DCA = %.2f\n9.MPPT2_DCV = %.2f\n10.MPPT2_DCA = %.2f\n11.Ph1ACV = %.2f\n12.Ph1ACA = %.2f\n13.Ph2ACV = %.2f\n14.Ph2ACA = %.2f\n15.Ph3ACV = %.2f\n16.Ph3ACA = %.2f\n17.InvAC_P = %.2f\n18.Inv_Fr = %.2f\n19.Inv_Wh = %.2f\n20.Inv_Nrg = %.2f\n21.Inv_Rt = %d\n22.Inv_LE = %.2f\n23.Inv_Sts = %d\n24.Inv_Tpr = %.2f\n25.Inv_Err = %d\n26.Inv_Wrn = %d\n27.Inv_Fnflt = %X\n28.Inv_StFlt = %s\n",datetime.day,datetime.month,datetime.year,datetime.hours,datetime.mins,dev_mac,rssi,id,serialno,pv1_vol,pv1_curr,pv2_vol,pv2_curr,ph1_vol,ph1_curr,ph2_vol,ph2_curr,ph3_vol,ph3_curr,ac_power,ph_freq,ac_power,today_energy,run_time,life_energy,run_status,inv_temp,error_code,warning_code,working_mode,str_status1);
        msg.msgId = DATA_ACQ_MSG;
        msg.data = data;
        xQueueSend(loggerQueue_h, (void *) &msg, 0);
        got_response =FALSE;
        memset(serialno, 0, sizeof(serialno));
        // memset(warning_code1, 0, sizeof(warning_code1));
         memset(data2, 0, sizeof(data2));
        return 1;  
    }
    else{
        return 2;
    }
    
}

int get_greater_100kwp_goodwe_inverter_data(uint8_t id, uint16_t reg_add,uint16_t reg_len, uint8_t reg_type){
    uint8_t data100[100]; 

    request.slave_addr = id;
	request.command = 0x03;
	request.reg_start = reg_add;
	request.reg_size = reg_len;
    while(1){
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
        err = mbc_master_send_request(&request,&data100);
        if (err== ESP_OK){
            break;
        }
    }
    if(reg_type == 1){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        pv1_vol =(float)reading1/10;
        reading1=0;
        printf("Vpv1=%.2f\n",pv1_vol);
        reading1 = data100[2]&0x000000FF;
        reading1 |= (data100[3] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        pv1_curr =(float)reading1/1000;
        str1 = pv1_curr;
        reading1=0;
        printf("Ipv1=%.2f\n",pv1_curr);
        reading1 = data100[4]&0x000000FF;
        reading1 |= (data100[5] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        pv2_vol =(float)reading1/10;
        reading1=0;
        printf("Vpv2=%.2f\n",pv2_vol);
        reading1 = data100[6]&0x000000FF;
        reading1 |= (data100[7] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        pv2_curr =(float)reading1/1000;
        reading1=0;
        printf("Ipv2=%.2f\n",pv2_curr);
        str2 = pv2_curr;
        reading1 = data100[10]&0x000000FF;
        reading1 |= (data100[11] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str3 =(float)reading1/10;
        reading1=0;
        printf("str3=%.2f\n",str3);
        reading1 = data100[14]&0x000000FF;
        reading1 |= (data100[15] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str4 =(float)reading1/1000;
        reading1=0;
        printf("str4=%.2f\n",str4);
        reading1 = data100[18]&0x000000FF;
        reading1 |= (data100[19] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str5 =(float)reading1/1000;
        reading1=0;
        printf("str5=%.2f\n",str5);
        reading1 = data100[22]&0x000000FF;
        reading1 |= (data100[23] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str6 =(float)reading1/1000;
        reading1=0;
        printf("str6=%.2f\n",str6);
        reading1 = data100[26]&0x000000FF;
        reading1 |= (data100[27] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str7 =(float)reading1/1000;
        reading1=0;
        printf("str7=%.2f\n",str7);
        reading1 = data100[30]&0x000000FF;
        reading1 |= (data100[31] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str8 =(float)reading1/1000;
        reading1=0;
        printf("str8=%.2f\n",str8);
        reading1 = data100[34]&0x000000FF;
        reading1 |= (data100[35] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str9 =(float)reading1/1000;
        reading1=0;
        printf("str9=%.2f\n",str9);
        reading1 = data100[38]&0x000000FF;
        reading1 |= (data100[39] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str10 =(float)reading1/1000;
        reading1=0;
        printf("str10=%.2f\n",str10);
        reading1 = data100[42]&0x000000FF;
        reading1 |= (data100[43] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str11 =(float)reading1/1000;
        reading1=0;
        printf("str11=%.2f\n",str11);
        reading1 = data100[46]&0x000000FF;
        reading1 |= (data100[47] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str12 =(float)reading1/1000;
        reading1=0;
        printf("str12=%.2f\n",str12);
        reading1 = data100[50]&0x000000FF;
        reading1 |= (data100[51] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str13 =(float)reading1/1000;
        reading1=0;
        printf("str13=%.2f\n",str13);
        reading1 = data100[54]&0x000000FF;
        reading1 |= (data100[55] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str14 =(float)reading1/1000;
        reading1=0;
        printf("str14=%.2f\n",str14);
        reading1 = data100[58]&0x000000FF;
        reading1 |= (data100[59] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str15 =(float)reading1/1000;
        reading1=0;
        printf("str15=%.2f\n",str15);
        reading1 = data100[62]&0x000000FF;
        reading1 |= (data100[63] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str16 =(float)reading1/1000;
        reading1=0;
        printf("str16=%.2f\n",str16);

        sprintf(str_status1,"%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",str1,str2,str3,str4,str5,str6,str7,str8,str9,str10,str11,str12,str12,str14,str15,str16);
        // str_status = reading1;
        printf("str_status =%S\n",str_status1);
        memset(data100,0,sizeof(data100));
    }
    if(reg_type == 2){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph1_vol =(float)reading1/10;
        reading1=0;
        printf("ph1_vol=%.2f\n",ph1_vol);
        reading1 = data100[2]&0x000000FF;
        reading1 |= (data100[3] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph2_vol =(float)reading1/10;
        reading1=0;
        printf("ph2_vol=%.2f\n",ph2_vol);
        reading1 = data100[4]&0x000000FF;
        reading1 |= (data100[5] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph3_vol =(float)reading1/10;
        reading1=0;
        printf("ph3_vol=%.2f\n",ph3_vol);
        reading1 = data100[6]&0x000000FF;
        reading1 |= (data100[7] << 8) & 0x0000FF00;
        reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[8] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[9] << 8) & 0x0000FF00;
        reading2 = reading2 & 0x0000FFFF;
        reading3 = reading1 | reading2;
        ph1_curr = (float)reading3/1000;
        printf("ph1_curr=%.2f\n",ph1_curr);
        reading1 = data100[10]&0x000000FF;
        reading1 |= (data100[11] << 8) & 0x0000FF00;
        reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[12] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[13] << 8) & 0x0000FF00;
        reading2 = reading2 & 0x0000FFFF;
        reading3 = reading1 | reading2;
        ph2_curr = (float)reading3/1000;
        printf("ph2_curr=%.2f\n",ph2_curr);
        reading1 = data100[14]&0x000000FF;
        reading1 |= (data100[15] << 8) & 0x0000FF00;
        reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[16] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[17] << 8) & 0x0000FF00;
        reading2 = reading2 & 0x0000FFFF;
        reading3 = reading1 | reading2;
        ph3_curr = (float)reading3/1000;
        printf("ph3_curr=%.2f\n",ph3_curr);
        memset(data100, 0, sizeof(data100));
    }
    if(reg_type == 3){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[2] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[3] << 8) & 0x0000FF00;
        reading2 = reading2 & 0x0000FFFF;
        reading3 = reading1 | reading2;
        ac_power = (float)reading3/1000;
        printf("ac_power=%.2f\n",ac_power);
        memset(data100, 0, sizeof(data100));
    }
    if(reg_type == 4){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[2] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[3] << 8) & 0x0000FF00;
        reading2 = reading2 & 0x0000FFFF;
        reading3 = reading1 | reading2;
        ph_freq = (float)reading3/100;
        printf("ph_freq=%.2f\n",ph_freq);
        reading1 = data100[4]&0x000000FF;
        reading1 |= (data100[5] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        inv_temp = (float)reading1/10;
        printf("inv_temp=%.2f\n",inv_temp);
        memset(data100, 0, sizeof(data100));
    }
    if(reg_type == 5){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[2] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[3] << 8) & 0x0000FF00;
        reading2 = reading2 & 0x0000FFFF;
        reading3 = reading1 | reading2;
        life_energy = (float)reading3/100;
        printf("life energy=%.2f\n",life_energy);
    }
    if(reg_type == 6){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[2] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[3] << 8) & 0x0000FF00;
        reading2 = reading2 & 0x0000FFFF;
        reading3 = reading1 | reading2;
        today_energy = (float)reading3/100;
        printf("today energy=%.2f\n",today_energy);
    }
    if(reg_type == 7){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[2] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[3] << 8) & 0x0000FF00;
        reading2 = reading2 & 0x0000FFFF;
        reading3 = reading1 | reading2;
        run_time = (float)reading3;
        printf("runtime=%d\n",run_time);
    }
    if(reg_type == 8){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        working_mode = reading1;
        reading1=0;
        printf("working mode=%d\n",working_mode);
        if( sync_done == TRUE && ((pv1_vol < 1000) && (pv2_vol <1000) && (ph1_vol < 500) && (ph2_vol < 500) && (ph3_vol < 500) && (ac_power < 100))){
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
            ESP_LOGI(TAG7, "DATE time = %d-%d-%d   %d:%d:%d",datetime.day,datetime.month,datetime.year,datetime.hours,datetime.mins,datetime.secs);
            interTaskMsg_t msg;
            strcpy(data,"");
            sprintf(data,"\"RVolt\":\"0\",\"RIMAID\":\"%s\",\"RSignal\":\"%d\",\"ISth\":\"0\",\"ID\":\"%d\",\"InvSlno\":\"0%s\",\"MPPT1_DCV\":\"%.2f\",\"MPPT1_DCA\":\"%.2f\",\"MPPT2_DCV\":\"%.2f\",\"MPPT2_DCA\":\"%.2f\",\"Ph1ACV\":\"%.2f\",\"Ph1ACA\":\"%.2f\",\"Ph2ACV\":\"%.2f\",\"Ph2ACA\":\"%.2f\",\"Ph3ACV\":\"%.2f\",\"Ph3ACA\":\"%.2f\",\"InvAC_P\":\"%.2f\",\"Inv_Fr\":\"%.2f\",\"Inv_Wh\":\"%.2f\",\"Inv_Nrg\":\"%.2f\",\"Inv_Rt\":\"%d\",\"Inv_LE\":\"%.2f\",\"Inv_Sts\":\"%d\",\"Inv_Tpr\":\"%.2f\",\"Inv_Err\":\"%d\",\"Inv_Wrn\":\"%d\",\"Inv_Fnflt\":\"0X%X\",\"Inv_StFlt\":\"%s\"",dev_mac,rssi,id,serialno,pv1_vol,pv1_curr,pv2_vol,pv2_curr,ph1_vol,ph1_curr,ph2_vol,ph2_curr,ph3_vol,ph3_curr,ac_power,ph_freq,ac_power,today_energy,run_time,life_energy,run_status,inv_temp,error_code,warning_code,working_mode,str_status1);
            printf("\n\n\n0.Rdate = %u-%u-%u %u:%u\n1.RVolt = 0\n2.RIMAID = %s\n3.RSignal = %d\n4.ISth = 0\n5.ID = %d\n6.InvSlno = 0%s\n7.MPPT1_DCV = %.2f\n8.MPPT1_DCA = %.2f\n9.MPPT2_DCV = %.2f\n10.MPPT2_DCA = %.2f\n11.Ph1ACV = %.2f\n12.Ph1ACA = %.2f\n13.Ph2ACV = %.2f\n14.Ph2ACA = %.2f\n15.Ph3ACV = %.2f\n16.Ph3ACA = %.2f\n17.InvAC_P = %.2f\n18.Inv_Fr = %.2f\n19.Inv_Wh = %.2f\n20.Inv_Nrg = %.2f\n21.Inv_Rt = %d\n22.Inv_LE = %.2f\n23.Inv_Sts = %d\n24.Inv_Tpr = %.2f\n25.Inv_Err = %d\n26.Inv_Wrn = %d\n27.Inv_Fnflt = 0X%X\n28.Inv_StFlt = %s\n",datetime.day,datetime.month,datetime.year,datetime.hours,datetime.mins,dev_mac,rssi,id,serialno,pv1_vol,pv1_curr,pv2_vol,pv2_curr,ph1_vol,ph1_curr,ph2_vol,ph2_curr,ph3_vol,ph3_curr,ac_power,ph_freq,ac_power,today_energy,run_time,life_energy,run_status,inv_temp,error_code,warning_code,working_mode,str_status1);
            msg.msgId = DATA_ACQ_MSG;
            msg.data = data;
            xQueueSend(loggerQueue_h, (void *) &msg, 0);
            got_response =FALSE;
            memset(serialno, 0, sizeof(serialno));
            // memset(warning_code1, 0, sizeof(warning_code1));
            // memset(error_code2, 0, sizeof(error_code2));
            memset(data100, 0, sizeof(data100));
            return 1;  
        }
        else{
            return 2;
        }
    } 
    return 0;
}