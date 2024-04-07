#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "my_iic.h"
#include "mqtt_publish.h"
#include "portmacro.h"

#define SGP30_ADDRESS 0x58
#define INIT_AIR_QUALITY 0x2003
#define MEASURE_AIR_QUALITY 0x2008

int CO2Data,TVOCData;//定义CO2浓度变量与TVOC浓度变量


static const char*  TAG  = "SGP30";

static esp_err_t i2c_master_sgp30_write(i2c_port_t i2c_num, uint16_t i2c_write)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SGP30_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, i2c_write>>8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, i2c_write&0xff, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_sgp30_read(i2c_port_t i2c_num, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SGP30_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

void sgp30_task(void *arg)
{
    uint8_t sensor_data[6];
    int ret;
    memset(sensor_data, 0, 6);

    ret = i2c_master_sgp30_write(I2C_MASTER_NUM,INIT_AIR_QUALITY);
    if(ret!=ESP_OK){
        ESP_LOGW(TAG,"write init command error");
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
    i2c_master_sgp30_write(I2C_MASTER_NUM,MEASURE_AIR_QUALITY);
    if(ret!=ESP_OK){
        ESP_LOGW(TAG,"write measure command error");
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
    ret = i2c_master_sgp30_read(I2C_MASTER_NUM,sensor_data,6);
    if(ret!=ESP_OK){
        ESP_LOGW(TAG,"init read error");
    }
    CO2Data = (sensor_data[0]<<8)|sensor_data[1];
    TVOCData = (sensor_data[3]<<8)|sensor_data[4];

    while(CO2Data == 400 && TVOCData == 0)
    {
        i2c_master_sgp30_write(I2C_MASTER_NUM,MEASURE_AIR_QUALITY);
        if(ret!=ESP_OK){
            ESP_LOGW(TAG,"init measure error");
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
        ret = i2c_master_sgp30_read(I2C_MASTER_NUM,sensor_data,6);
        if(ret!=ESP_OK){
            ESP_LOGW(TAG,"read error");
        }
        CO2Data = (sensor_data[0]<<8)|sensor_data[1];
        TVOCData = (sensor_data[3]<<8)|sensor_data[4];
        //ESP_LOGI(TAG,"The CO2DATA: %d,TVOCDATA: %d",CO2Data,TVOCData);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }

    while (1) {
        ret = i2c_master_sgp30_write(I2C_MASTER_NUM,MEASURE_AIR_QUALITY);
        if(ret!=ESP_OK){
            ESP_LOGW(TAG,"measure error");
        }
        vTaskDelay(500/portTICK_PERIOD_MS);
        ret = i2c_master_sgp30_read(I2C_MASTER_NUM,sensor_data,6);
        if(ret!=ESP_OK){
            ESP_LOGW(TAG,"read error");
        }
        pSensorData->CO2Data = (sensor_data[0]<<8)|sensor_data[1];
        TVOCData = (sensor_data[3]<<8)|sensor_data[4];
        ESP_LOGI(TAG,"The CO2DATA: %d,TVOCDATA: %d",CO2Data,TVOCData);
        vTaskDelay(3000/portTICK_PERIOD_MS);
    }
    i2c_driver_delete(I2C_MASTER_NUM);
}