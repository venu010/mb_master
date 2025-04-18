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

#define TAG8   "SUNGROW"

#define PACKET_READ_TICS        (100 / portTICK_PERIOD_MS)
static mb_param_request_t request1;

extern esp_err_t err;

extern char dev_mac[30], data[4096],str_status1[100];
extern uint16_t crc_fun(unsigned char *nData, uint16_t wLength);
extern char serialno[50],modelno[20], mload, mresult, SIM_ID[30],IMEI[22], date_time[30], version[5],mresult1,error_code2[20],warning_code1[20];
extern int error_code,str_status,warning_code,working_mode,run_time,run_status;
extern float ph_freq,pv1_vol,pv1_curr,pv2_vol,pv2_curr,pv3_vol,pv3_curr,pv4_vol,pv4_curr,ph1_vol,ph1_curr,ph2_vol,ph2_curr,ph3_vol,ph3_curr,str1,str2,str3,str4,str5,str6,str7,str8,str9,str10,str11,str12,str13,str14,str15,str16,str17,str18,str19,str20,str21,str22,str23,str24,ac_power,life_energy,today_energy,inv_temp,rtime,watt_hr;
extern int reading1, reading2,reading3;
extern uint8_t goodwe_greater_100kwp_status;
extern uint8_t got_response;
extern QueueHandle_t loggerQueue_h;
extern int32_t rssi;
extern uint8_t sync_done;
extern uint8_t callback_status;
extern date_time_t datetime;
int get_sungrow_inverter_data_at_onetime(uint8_t id,uint16_t reg_add,uint16_t reg_len,uint8_t reg_type);
esp_err_t get_sungrow_inverter_serial_number(uint8_t id, uint16_t reg_add,uint16_t reg_len);


esp_err_t get_sungrow_inverter_serial_number(uint8_t id, uint16_t reg_add,uint16_t reg_len){
    // uint8_t* data = (uint8_t*) malloc(data_buff_size);
    uint8_t slnum[50]={0};
    request1.slave_addr = id;
	request1.command = 0x04;
	request1.reg_start = reg_add;
	request1.reg_size = reg_len;
    int j =0 ;
    while(1){
        j++;
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
        memset(slnum, 0, sizeof(slnum));
        err = mbc_master_send_request(&request1,&slnum);
        printf("slnum =%s \t",slnum);
        
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

int get_sungrow_inverter_data_at_onetime(uint8_t id,uint16_t reg_add,uint16_t reg_len,uint8_t reg_type){
    uint8_t data100[100];
    request1.slave_addr = id;
	request1.command = 0x04;
	request1.reg_start = reg_add;
	request1.reg_size = reg_len;
    while(1){
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
        err = mbc_master_send_request(&request1,&data100);
        if (err== ESP_OK){
            break;
        }
    }
    if(reg_type==1 ){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        today_energy =(float)reading1/10;
        reading1=0;
        printf("today_energy=%.2f\n",today_energy);

        // reading1 = data100[2]&0x000000FF;
        // reading1 |= (data100[3] << 8) & 0x0000FF00;
        // reading1 = reading1  & 0xFFFF0000;
        // reading2 = data100[4] & 0x000000FF; //Load LSB of higher 16bit
        // reading2 |= (data100[5] << 8) & 0x0000FF00;
        // // reading2 = reading2 & 0x0000FFFF;
        // reading2 = (reading2 << 16) & 0xFFFF0000;


        reading1 = data100[2]&0x000000FF;
        reading1 |= (data100[3] << 8) & 0x0000FF00;
        // reading1 = reading1 << 16 & 0xFFFF0000;
        reading2 = data100[4] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[5] << 8) & 0x0000FF00;
        reading2 = (reading2 << 16) & 0xFFFF0000;
        reading3 = reading1 | reading2;
        life_energy = (float)reading3;
        printf("life energy=%.2f\n",life_energy);

        reading1 = data100[6]&0x000000FF;
        reading1 |= (data100[7] << 8) & 0x0000FF00;
        reading1 = reading1  & 0x0000FFFF;
        // reading2 = data100[8] & 0x000000FF; //Load LSB of higher 16bit
        // reading2 |= (data100[9] << 8) & 0x0000FF00;
        // reading2 = reading2 & 0x0000FFFF;
        // reading2 = (reading2 << 16) & 0xFFFF0000;
        // reading3 = reading1 | reading2;
        run_time = (float)reading1;
        printf("runtime=%d\n",run_time);
        reading1 = data100[10]&0x000000FF;
        reading1 |= (data100[11] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        inv_temp = (float)reading1/10;
        printf("inv_temp=%.2f\n",inv_temp);
    }
    if(reg_type ==2){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        pv1_vol =(float)reading1/10;
        reading1=0;
        printf("Vpv1=%.2f\n",pv1_vol);
        reading1 = data100[2]&0x000000FF;
        reading1 |= (data100[3] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        pv1_curr =(float)reading1/10;
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
        pv2_curr =(float)reading1/10;
        reading1=0;
        printf("Ipv2=%.2f\n",pv2_curr);

        // reading1 = data100[12]&0x000000FF;
        // reading1 |= (data100[13] << 8) & 0x0000FF00;
        // reading1 = reading1  & 0x0000FFFF;
        // reading2 = data100[14] & 0x000000FF; //Load LSB of higher 16bit
        // reading2 |= (data100[15] << 8) & 0x0000FF00;
        // // reading2 = reading2 & 0x0000FFFF;
        // reading2 = reading2 << 16 & 0xFFFF0000;
        // reading3 = reading1 | reading2;
        // watt_hr = (float)reading3/1000;
        // printf("watt_hr=%d\n",watt_hr);

        reading1 = data100[16]&0x000000FF;
        reading1 |= (data100[17] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph1_vol =(float)reading1/10;
        reading1=0;
        printf("ph1_vol=%.2f\n",ph1_vol);

        reading1 = data100[18]&0x000000FF;
        reading1 |= (data100[19] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph2_vol =(float)reading1/10;
        reading1=0;
        printf("ph2_vol=%.2f\n",ph2_vol);
        reading1 = data100[20]&0x000000FF;
        reading1 |= (data100[21] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph3_vol =(float)reading1/10;
        reading1=0;
        printf("ph3_vol=%.2f\n",ph3_vol);

        reading1 = data100[22]&0x000000FF;
        reading1 |= (data100[23] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph1_curr =(float)reading1/10;
        reading1=0;
        printf("ph1_curr=%.2f\n",ph1_curr);

        reading1 = data100[24]&0x000000FF;
        reading1 |= (data100[25] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph2_curr =(float)reading1/10;
        reading1=0;
        printf("ph2_curr=%.2f\n",ph2_curr);

        reading1 = data100[26]&0x000000FF;
        reading1 |= (data100[27] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph3_curr =(float)reading1/10;
        reading1=0;
        printf("ph3_curr=%.2f\n",ph3_curr);
    }
    if(reg_type ==3){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        // reading1 = reading1  & 0x0000FFFF;
        reading2 = data100[2] & 0x000000FF; //Load LSB of higher 16bit
        reading2 |= (data100[3] << 8) & 0x0000FF00;
        // reading2 = reading2 & 0x0000FFFF;
        reading2 = (reading2 << 16) & 0xFFFF0000;
        reading3 =  reading1 | reading2 ;
        ac_power = (float)reading3/1000;
        printf("ac_power=%.2f\n",ac_power);

        reading1 = data100[10]&0x000000FF;
        reading1 |= (data100[11] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        ph_freq =(float)reading1/10;
        reading1=0;
        printf("ph_freq=%.2f\n",ph_freq);
    }
    if(reg_type == 4){

        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        int status =(float)reading1;
        reading1=0;
        if(status == 0x0000){
            run_status =0;
        }
        if(status == 0x8000){
            run_status =1;
        }
        if(status == 0x1300){
            run_status =2;
        }
        if(status == 0x1500){
            run_status =3;
        }
        if(status == 0x1400){
            run_status =4;
        }
        if(status == 0x1200){
            run_status =5;
        }
        if(status == 0x1600){
            run_status =6;
        }
        if(status == 0x9100){
            run_status =7;
        }
        if(status == 0x8100){
            run_status =8;
        }
        if(status == 0x8200){
            run_status =9;
        }
        if(status == 0x5500){
            run_status =10;
        }
        if(status == 0x2500){
            run_status =11;
        }
        printf("run_status=%d\n",run_status);

        reading1 = data100[14]&0x000000FF;
        reading1 |= (data100[15] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        int error1 =(float)reading1;
        reading1=0;
        printf("error1=%d\t",error1);
        if (error1 == 105){
            error_code = 1;
        }
        if (error1 == 106){
            error_code = 18;
        }
        if (error1 == 116){
            error_code = 26;
        }
        if (error1 == 117){
            error_code = 27;
        }
        if (error1 == 220){
            error_code = 28;
        }
        if (error1 == 221){
            error_code = 29;
        }
        if (error1 == 222){
            error_code = 35;
        }
        if (error1 == 223){
            error_code = 51;
        }
        if (error1 == 224){
            error_code = 52;
        }
        if (error1 == 225){
            error_code = 57;
        }
        if (error1 == 226){
            error_code = 58;
        }
        if (error1 == 227){
            error_code = 61;
        }
        if (error1 == 514){
            error_code = 62;
        }
        if (error1 == 532){
            error_code = 63;
        }
        if (error1 == 533){
            error_code = 64;
        }
        if (error1 == 534){
            error_code = 65;
        }
        if (error1 == 535){
            error_code = 66;
        }
        if (error1 == 536){
            error_code = 67;
        }
        if (error1 == 537){
            error_code = 68;
        }
        if (error1 == 538){
            error_code = 69;
        }
        if (error1 == 539){
            error_code = 73;
        }
        if (error1 == 540){
            error_code = 75;
        }
        if (error1 == 541){
            error_code = 77;
        }
        if (error1 == 542){
            error_code = 82;
        }
        if (error1 == 543){
            error_code = 83;
        }
        if (error1 == 544){
            error_code = 84;
        }
        if (error1 == 545){
            error_code = 85;
        }
        if (error1 == 546){
            error_code = 86;
        }
        if (error1 == 547){
            error_code = 90;
        }
        if (error1 == 548){
            error_code = 91;
        }
        if (error1 == 549){
            error_code = 92;
        }
        if (error1 == 550){
            error_code = 93;
        }
        if (error1 == 551){
            error_code = 94;
        }
        if (error1 == 552){
            error_code = 95;
        }
        if (error1 == 553){
            error_code = 96;
        }
        if (error1 == 554){
            error_code = 97;
        }
        if (error1 == 555){
            error_code = 98;
        }
        if (error1 == 556){
            error_code = 99;
        }
        if (error1 == 557){
            error_code = 100;
        }
        if (error1 == 558){
            error_code = 101;
        }
        if (error1 == 559){
            error_code = 102;
        }
        if (error1 == 560){
            error_code = 103;
        }
        if (error1 == 561){
            error_code = 104;
        }
        if (error1 == 562){
            error_code = 105;
        }
        if (error1 == 563){
            error_code = 106;
        }
        if (error1 == 448){
            error_code = 107;
        }
        if (error1 == 449){
            error_code = 108;
        }
        if (error1 == 450){
            error_code = 109;
        }
        if (error1 == 451){
            error_code = 110;
        }
        if (error1 == 452){
            error_code = 111;
        }
        if (error1 == 453){
            error_code = 112;
        }
        if (error1 == 454){
            error_code = 113;
        }
        if (error1 == 455){
            error_code = 114;
        }
        if (error1 == 456){
            error_code = 115;
        }
        if (error1 == 457){
            error_code = 116;
        }
        if (error1 == 458){
            error_code = 117;
        }
        if (error1 == 459){
            error_code = 118;
        }
        if (error1 == 460){
            error_code = 119;
        }
        if (error1 == 461){
            error_code = 120;
        }
        if (error1 == 462){
            error_code = 121;
        }
        if (error1 == 463){
            error_code = 122;
        }
        else{
            error_code =error1;
        }
        printf("error_code=%d\n",error_code);
    }
    if(reg_type == 5){
        reading1 = data100[0]&0x000000FF;
        reading1 |= (data100[1] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str1 =(float)reading1/100;
        reading1=0;
        printf("str1=%.2f\n",str1);
        reading1 = data100[2]&0x000000FF;
        reading1 |= (data100[3] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str2 =(float)reading1/100;
        reading1=0;
        printf("str2=%.2f\n",str2);
        reading1 = data100[4]&0x000000FF;
        reading1 |= (data100[5] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str3 =(float)reading1/100;
        reading1=0;
        printf("str3=%.2f\n",str3);
        reading1 = data100[6]&0x000000FF;
        reading1 |= (data100[7] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str4 =(float)reading1/100;
        reading1=0;
        printf("str4=%.2f\n",str4);
        reading1 = data100[8]&0x000000FF;
        reading1 |= (data100[9] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str5 =(float)reading1/100;
        reading1=0;
        printf("str5=%.2f\n",str5);
        reading1 = data100[10]&0x000000FF;
        reading1 |= (data100[11] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str6 =(float)reading1/100;
        reading1=0;
        printf("str6=%.2f\n",str6);
        reading1 = data100[12]&0x000000FF;
        reading1 |= (data100[13] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str7 =(float)reading1/100;
        reading1=0;
        printf("str7=%.2f\n",str7);
        reading1 = data100[14]&0x000000FF;
        reading1 |= (data100[15] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str8 =(float)reading1/100;
        reading1=0;
        printf("str8=%.2f\n",str8);
        reading1 = data100[15]&0x000000FF;
        reading1 |= (data100[17] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str9 =(float)reading1/100;
        reading1=0;
        printf("str9=%.2f\n",str9);
        reading1 = data100[18]&0x000000FF;
        reading1 |= (data100[19] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str10 =(float)reading1/100;
        reading1=0;
        printf("str10=%.2f\n",str10);
        reading1 = data100[20]&0x000000FF;
        reading1 |= (data100[21] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str11 =(float)reading1/100;
        reading1=0;
        printf("str11=%.2f\n",str11);
        reading1 = data100[22]&0x000000FF;
        reading1 |= (data100[23] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str12 =(float)reading1/100;
        reading1=0;
        printf("str12=%.2f\n",str12);
        reading1 = data100[24]&0x000000FF;
        reading1 |= (data100[25] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str13 =(float)reading1/100;
        reading1=0;
        printf("str13=%.2f\n",str13);
        reading1 = data100[26]&0x000000FF;
        reading1 |= (data100[27] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str14 =(float)reading1/100;
        reading1=0;
        printf("str14=%.2f\n",str14);
        reading1 = data100[28]&0x000000FF;
        reading1 |= (data100[29] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str15 =(float)reading1/100;
        reading1=0;
        printf("str15=%.2f\n",str15);
        reading1 = data100[30]&0x000000FF;
        reading1 |= (data100[31] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str16 =(float)reading1/100;
        reading1=0;
        printf("str16=%.2f\n",str16);
        reading1 = data100[32]&0x000000FF;
        reading1 |= (data100[33] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str17 =(float)reading1/100;
        reading1=0;
        printf("str17=%.2f\n",str17);
        reading1 = data100[34]&0x000000FF;
        reading1 |= (data100[35] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str18 =(float)reading1/100;
        reading1=0;
        printf("str18=%.2f\n",str18);
        reading1 = data100[36]&0x000000FF;
        reading1 |= (data100[37] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str19 =(float)reading1/100;
        reading1=0;
        printf("str19=%.2f\n",str19);
        reading1 = data100[38]&0x000000FF;
        reading1 |= (data100[39] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str20 =(float)reading1/100;
        reading1=0;
        printf("str20=%.2f\n",str20);
        reading1 = data100[40]&0x000000FF;
        reading1 |= (data100[41] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str21 =(float)reading1/100;
        reading1=0;
        printf("str21=%.2f\n",str21);
        reading1 = data100[42]&0x000000FF;
        reading1 |= (data100[43] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str22 =(float)reading1/100;
        reading1=0;
        printf("str22=%.2f\n",str22);
        reading1 = data100[44]&0x000000FF;
        reading1 |= (data100[45] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str23 =(float)reading1/100;
        reading1=0;
        printf("str23=%.2f\n",str23);
        reading1 = data100[46]&0x000000FF;
        reading1 |= (data100[47] << 8) & 0x0000FF00;
        reading1 = reading1 & 0x0000FFFF;
        str24 =(float)reading1/100;
        reading1=0;
        printf("str24=%.2f\n",str24);

        sprintf(str_status1,"%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",str1,str2,str3,str4,str5,str6,str7,str8,str9,str10,str11,str12,str12,str14,str15,str16,str17,str18,str19,str20,str21,str22,str23,str24);
        // str_status = reading1;
        printf("str_status =%S\n",str_status1);
        memset(data100,0,sizeof(data100));
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
            ESP_LOGI(TAG8, "DATE time = %d-%d-%d   %d:%d:%d",datetime.day,datetime.month,datetime.year,datetime.hours,datetime.mins,datetime.secs);
            interTaskMsg_t msg;
            // printf("-------------1--------------\n");
            // // memset(data, 0, sizeof(data));
            // printf("-------------1--------------\n");
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

