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

#define TAG9   "GROWATT"




static mb_param_request_t request3;

extern esp_err_t err;

#define PACKET_READ_TICS        (100 / portTICK_PERIOD_MS)
extern char dev_mac[30],data[4096],str_status1[100];
extern uint16_t crc_fun(unsigned char *nData, uint16_t wLength);
extern char serialno[50],modelno[20], mload, mresult, SIM_ID[30],IMEI[22], date_time[30], version[5],mresult1,error_code2[20],warning_code1[20];
extern int error_code,str_status,warning_code,working_mode,run_time,run_status;
extern float ph_freq,pv1_vol,pv1_curr,pv2_vol,pv2_curr,pv3_vol,pv3_curr,pv4_vol,pv4_curr,ph1_vol,ph1_curr,ph2_vol,ph2_curr,ph3_vol,ph3_curr,str1,str2,str3,str4,str5,str6,str7,str8,str9,str10,str11,str12,str13,str14,str15,str16,ac_power,life_energy,today_energy,inv_temp,rtime;
extern int reading1, reading2,reading3;
extern uint8_t goodwe_greater_100kwp_status;
extern uint8_t got_response;
extern QueueHandle_t loggerQueue_h;
extern char data[4096];
extern int32_t rssi;
extern uint8_t sync_done;
extern uint8_t callback_status;
extern date_time_t datetime;

esp_err_t get_growatt_inverter_serial_number(uint8_t id, uint16_t reg_add,uint16_t reg_len);
int get_growatt_inverter_data_at_onetime(uint8_t id, uint16_t reg_add,uint16_t reg_len,uint8_t reg_type);


esp_err_t get_growatt_inverter_serial_number(uint8_t id, uint16_t reg_add,uint16_t reg_len){
    // uint8_t* data = (uint8_t*) malloc(data_buff_size);
    uint8_t slnum[50]={0};
    request3.slave_addr = id;
	request3.command = 0x03;
	request3.reg_start = reg_add;
	request3.reg_size = reg_len;
    int j =0 ;
    while(1){
        j++;
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
        memset(slnum, 0, sizeof(slnum));
        err = mbc_master_send_request(&request3,&slnum);
        printf("slnum =%s",slnum);
        
        printf("sl num :%C %C %X %X %X %X %X %X %X %X %X %X %X %X %X %X  \n",slnum[0],slnum[1],slnum[2],slnum[3],slnum[4],slnum[5],slnum[6],slnum[7],slnum[8],slnum[9],slnum[10],slnum[11],slnum[12],slnum[13],slnum[14],slnum[15]);
        // for(int i =0;i<=15;i++){
        //     serialno[i]=slnum[i];
        // }
        sprintf(serialno,"%C%C%C%C%C%C%C%C%C%C%C%C%C%C%C%C",slnum[1],slnum[0],slnum[3],slnum[2],slnum[5],slnum[4],slnum[7],slnum[6],slnum[9],slnum[8],slnum[11],slnum[10],slnum[13],slnum[12],slnum[15],slnum[14]);
        printf("serial num = %S\n",serialno);
        if (strlen(serialno)>8 && err == ESP_OK){
            got_response =TRUE;
            return err;
            break;
        }
        else if(err == ESP_ERR_TIMEOUT || err == ESP_ERR_NOT_FOUND || j >= 3 ){
            j=0;
            got_response =FALSE;
            return err;
            break;
        }
        got_response =FALSE;
    }    
}

int get_growatt_inverter_data_at_onetime(uint8_t id,uint16_t reg_add,uint16_t reg_len,uint8_t reg_type){
    uint8_t data100[100];
    request3.slave_addr = id;
	request3.command = 0x04;
	request3.reg_start = reg_add;
	request3.reg_size = reg_len;
    while(1){
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
        err = mbc_master_send_request(&request3,&data100);
        if (err== ESP_OK){
            break;
        }
    }
    if(reg_type ==1){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        run_status = reading1;
        if(run_status == 0){
            warning_code =2;
        }
        if(run_status == 1){
            warning_code = 0;
        }
        if(run_status == 2){
            warning_code = 1;
        }
        printf("warning_code = %d \n",warning_code);
        reading1 = data100[6]&0x000000FF;
        reading1 |= (data100[7] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        pv1_vol = (float)reading1/10;
        printf("pv1_vol = %.2f \n",pv1_vol);
        reading1 = data100[8]&0x000000FF;
        reading1 |= (data100[9] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        pv1_curr = (float)reading1/10;
        printf("pv1_curr = %.2f \n",pv1_curr);
        reading1 = data100[14]&0x000000FF;
        reading1 |= (data100[15] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        pv2_vol = (float)reading1/10;
        printf("pv2_vol = %.2f \n",pv2_vol);
        reading1 = data100[16]&0x000000FF;
        reading1 |= (data100[17] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        pv2_curr = (float)reading1/10;
        printf("pv2_curr = %.2f \n",pv2_curr);
    }
    if(reg_type == 2){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[2] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[3] << 8) & 0x0000FF00;
        reading2 = reading2 & 0x0000FFFF;
        reading3 = reading1 | reading2;
        ac_power = (float)reading3/10000;
        printf("ac_power=%.2f\n",ac_power);
        reading1 = data100[4]&0x000000FF;
        reading1 |= (data100[5] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph_freq = (float)reading1/100;
        printf("ph_freq=%.2f\n",ph_freq);
        reading1 = data100[6]&0x000000FF;
        reading1 |= (data100[7] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph1_vol =(float)reading1/10;
        reading1=0;
        printf("ph1_vol=%.2f\n",ph1_vol);
        reading1 = data100[6]&0x000000FF;
        reading1 |= (data100[7] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph1_curr =(float)reading1/10;
        reading1=0;
        printf("ph1_curr=%.2f\n",ph1_curr);

        reading1 = data100[14]&0x000000FF;
        reading1 |= (data100[15] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph2_vol =(float)reading1/10;
        reading1=0;
        printf("ph2_vol=%.2f\n",ph2_vol);
        reading1 = data100[16]&0x000000FF;
        reading1 |= (data100[17] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph2_curr =(float)reading1/10;
        reading1=0;
        printf("ph2_curr=%.2f\n",ph2_curr);
        reading1 = data100[22]&0x000000FF;
        reading1 |= (data100[23] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph3_vol =(float)reading1/10;
        reading1=0;
        printf("ph3_vol=%.2f\n",ph3_vol);
        reading1 = data100[24]&0x000000FF;
        reading1 |= (data100[25] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph3_curr =(float)reading1/10;
        reading1=0;
        printf("ph3_curr=%.2f\n",ph3_curr);
    }
    if (reg_type == 3){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[2] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[3] << 8) & 0x0000FF00;
        reading2 = reading2 & 0x0000FFFF;
        reading3 = reading1 | reading2;
        today_energy = (float)reading3/10;
        printf("today energy=%.2f\n",today_energy);
        reading1 = data100[4]&0x000000FF;
        reading1 |= (data100[5] << 8) & 0x0000FF00;
        reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[6] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[7] << 8) & 0x0000FF00;
        reading2 = reading2 & 0x0000FFFF;
        reading3 = reading1 | reading2;
        life_energy = (float) reading3/10;
        printf("life_energy =%.2f\n",life_energy);
        reading1 = data100[8]&0x000000FF;
        reading1 |= (data100[9] << 8) & 0x0000FF00;
        reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[10] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[11] << 8) & 0x0000FF00;
        reading2 = reading2 & 0x0000FFFF;
        reading3 = reading1 | reading2;
        run_time = reading3/500;
        printf("runtime=%d\n",run_time);
    }
    if (reg_type==4){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        inv_temp = (float)reading1/10;
        printf("inv_temp=%.2f\n",inv_temp);
    }
    if(reg_type == 5){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        run_status =reading1;
        printf("run_status = %d \n",run_status);       
    }
    if(reg_type == 6){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[2] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[3] << 8) & 0x0000FF00;
        reading2 = reading2 & 0x0000FFFF;
        reading3 = reading1 | reading2;
        int error =reading3;
        printf("error=%X\t", error);
        if(error == 0X0000){
            error_code = 400;
        }
        if(error == 0X0001){
            error_code = 401;
        }
        if(error == 0X0002){
            error_code = 402;
        }
        if(error == 0X0004){
            error_code = 403;
        }
        if(error == 0X0010){
            error_code = 404;
        }
        if(error == 0X0040){
            error_code = 405;
        }
        if(error == 0X0080){
            error_code = 406;
        }
        if(error == 0X0100){
            error_code = 407;
        }
        if(error == 0X0200){
            error_code = 408;
        }
        printf("error_code = %d \n",error_code);  
    }
    if(reg_type ==7){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str1 =(float)reading1/10;
        printf("str1 = %.2f \n",str1); 
        reading1 = data100[4]&0x000000FF;
        reading1 |= (data100[5] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str2 =(float)reading1/10;
        printf("str2 = %.2f \n",str2); 
        reading1 = data100[8]&0x000000FF;
        reading1 |= (data100[9] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str3 =(float)reading1/10;
        printf("str3 = %.2f \n",str3); 
        reading1 = data100[12]&0x000000FF;
        reading1 |= (data100[13] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str4 =(float)reading1/10;
        printf("str4 = %.2f \n",str4); 
        reading1 = data100[16]&0x000000FF;
        reading1 |= (data100[17] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str5 =(float)reading1/10;
        printf("str5 = %.2f \n",str5); 
        reading1 = data100[20]&0x000000FF;
        reading1 |= (data100[21] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str6 =(float)reading1/10;
        printf("str6 = %.2f \n",str6); 
        reading1 = data100[24]&0x000000FF;
        reading1 |= (data100[25] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str7 =(float)reading1/10;
        printf("str7 = %.2f \n",str7); 
        reading1 = data100[28]&0x000000FF;
        reading1 |= (data100[29] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str8 =(float)reading1/10;
        printf("str8 = %.2f \n",str8);
        reading1 = data100[32]&0x000000FF;
        reading1 |= (data100[33] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str9 =(float)reading1/10;
        printf("str9 = %.2f \n",str9); 
        reading1 = data100[36]&0x000000FF;
        reading1 |= (data100[37] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str10 =(float)reading1/10;
        printf("str10 = %.2f \n",str10); 
        reading1 = data100[40]&0x000000FF;
        reading1 |= (data100[41] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str11 =(float)reading1/10;
        printf("str11 = %.2f \n",str11); 
        reading1 = data100[44]&0x000000FF;
        reading1 |= (data100[45] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str12 =(float)reading1/10;
        printf("str12 = %.2f \n",str12); 
        reading1 = data100[48]&0x000000FF;
        reading1 |= (data100[49] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str13 =(float)reading1/10;
        printf("str13 = %.2f \n",str13); 
        reading1 = data100[52]&0x000000FF;
        reading1 |= (data100[53] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str14 =(float)reading1/10;
        printf("str14 = %.2f \n",str14); 
        reading1 = data100[56]&0x000000FF;
        reading1 |= (data100[57] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str15 =(float)reading1/10;
        printf("str15 = %.2f \n",str15); 
        reading1 = data100[60]&0x000000FF;
        reading1 |= (data100[61] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str16 =(float)reading1/10;
        printf("str16 = %.2f \n",str16); 

        sprintf(str_status1,"%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",str1,str2,str3,str4,str5,str6,str7,str8,str9,str10,str11,str12,str12,str14,str15,str16);
        // str_status = reading1;
        printf("str_status =%S\n",str_status1);
    // }
    // if(reg_type == 8){
    //     reading1 = data100[0]&0x000000FF;
    //     reading1 |= (data100[1] << 8) & 0x0000FF00;
    //     reading1 = reading1 & 0x0000FFFF;
    //     int fnflt = reading1;
    //     reading1=0;
    //     // if(((fnflt>>0)&1)==1){
    //     //     working_mode = 
    //     // }
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
            ESP_LOGI(TAG9, "DATE time = %d-%d-%d   %d:%d:%d",datetime.day,datetime.month,datetime.year,datetime.hours,datetime.mins,datetime.secs);
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