#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "mqtt_publish.h"
#include "my_iic.h"
#include "driver/adc.h"

static const char*  TAG  = "COMMON_SENSOR";

//下面是BH1750传感器相关宏定义
#define BH1750_ADDRESS 0x23     /* slave address for BH1750 sensor */

#define CONTINUE_H_MODE1 0X10                                /* mode for continuously high resolution */
#define CONTINUE_H_MODE2 0X11                                /* mode2 for continuously high resolution */
#define CONTINUE_L_MODE 0X13                                 /* mode for continuously low resolution */

//下面是GXHT30传感器相关红定义
#define GXHT30_ADDRESS 0x44 
#define HIGH_OPEN_ONETIME_MODE 0x2C06
#define HIGH_CLOSE_ONETIME_MODE 0X2400


//下面是数据转换函数，将raw data进行转换
int DATA_CONVENTION_TO_CELSIUS(int temperature){
  return (175*temperature/(0xffff)-45);
}

int DATA_CONVENTION_TO_HUMIDITY(int humidity){
  return (100*humidity/(0xffff));
}

static esp_err_t i2c_master_gy30_write(i2c_port_t i2c_num, uint8_t i2c_write)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, i2c_write, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
}

static esp_err_t i2c_master_gy30_read(i2c_port_t i2c_num, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_master_gxht30_write(i2c_port_t i2c_num, uint16_t i2c_write)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, GXHT30_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, i2c_write>>8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, i2c_write&0xff, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_gxht30_read(i2c_port_t i2c_num, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, GXHT30_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


static void moisture_init(){
    // 1. init adc
    adc_config_t adc_config;

    // Depend on menuconfig->Component config->PHY->vdd33_const value
    // When measuring system voltage(ADC_READ_VDD_MODE), vdd33_const must be set to 255.
    adc_config.mode = ADC_READ_TOUT_MODE;
    adc_config.clk_div = 8; // ADC sample collection clock = 80MHz/clk_div = 10MHz
    ESP_ERROR_CHECK(adc_init(&adc_config));
}


void commonSensor_task(void *arg)
{
    uint8_t sensor_data[6];
    int ret;

    moisture_init();
    i2c_master_gy30_write(I2C_MASTER_NUM,CONTINUE_H_MODE1); // start high resolution mode

    while (1) {
        memset(sensor_data, 0, 2);
        ret = i2c_master_gy30_read(I2C_MASTER_NUM, sensor_data, 2);
        if(ret==ESP_OK){
            pSensorData->lightTensity = ((sensor_data[0]<<8)|(sensor_data[1]))/1.2;
            // ESP_LOGI(TAG,"The tensity of the light is %d.",pSensorData->lightTensity);
        }else{
            ESP_LOGW(TAG,"gy30Detect Error.");
        }
        vTaskDelay(1000 / portTICK_RATE_MS);

        memset(sensor_data, 0, 6);
        i2c_master_gxht30_write(I2C_MASTER_NUM,HIGH_CLOSE_ONETIME_MODE);
        vTaskDelay(200 / portTICK_RATE_MS);
        ret = i2c_master_gxht30_read(I2C_MASTER_NUM, sensor_data, 6);
        if(ret==ESP_OK){
            pSensorData->temperature = DATA_CONVENTION_TO_CELSIUS((sensor_data[0]<<8)|sensor_data[1]);
            // ESP_LOGI(TAG,"The temperature is %d.",pSensorData->temperature);
            pSensorData->humidity = DATA_CONVENTION_TO_HUMIDITY((sensor_data[3]<<8)|sensor_data[4]);
            // ESP_LOGI(TAG,"The humidity is %d.",pSensorData->humidity);
        }else{
            ESP_LOGW(TAG,"gxht30 Detect Error.");
        }
        vTaskDelay(1000 / portTICK_RATE_MS);

        if (ESP_OK == adc_read((uint16_t*)&(pSensorData->soilMoisture))) {
            //ESP_LOGI(TAG, "adc read: %d\r\n", pSensorData->soilMoisture);
        }else{
            ESP_LOGW(TAG,"ADC READ ERROR");
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    i2c_driver_delete(I2C_MASTER_NUM);
}


