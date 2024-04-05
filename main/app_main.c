/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "wifi_init.h"
#include "sgp30.h"
#include "my_iic.h"
#include "control.h"
#include "mqtt_event.h"
#include "mqtt_publish.h"
#include "commonSensor.h"


static const char* TAG = "Main";
//这里创建指向传感器数据结构体的指针
SensorData* pSensorData;
//这里创建指向设备状态信息的结构体指针
DeviceStatus* pDeviceStatus;
//这里创建传递控制指令的消息队列
QueueHandle_t controlQueue;

char* stepper1_device_str = "roller_1";
char* relay1_device_str = "waterPump_1";
char* relay2_device_str = "fan_1";
char* majorDeviceId = "esp8266_1";

char* resultTopic = "/executionResult/upload/esp8266_1";
char* statusTopic = "/status/upload/esp8266_1";
char* commandTopic = "/command/send/esp8266_1";
char* sensorDataTopic = "/data/upload/esp8266_1";

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
                        ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 创建消息队列
    controlQueue = xQueueCreate(10, sizeof(ControlCommand));
    // 下面是初始化deviceStatus结构体成员变量
    pDeviceStatus = malloc(sizeof(DeviceStatus));
    memset(pDeviceStatus,0,sizeof(DeviceStatus));
    // 下面是初始化sensorData结构体成员变量
    pSensorData = malloc(sizeof(SensorData));
    memset(pSensorData,0,sizeof(SensorData));
    
    wifi_init_sta();

    mqtt_app_start();
    i2c_master_init();
    
    xTaskCreate(control_task,"controlTask",2024,NULL,10,NULL);
    xTaskCreate(commonSensor_task,"commonSensor_task",2024,NULL,5,NULL);
    xTaskCreate(sgp30_task,"sgp30_task",2024,NULL,4,NULL);
    xTaskCreate(mqtt_publish_data_task,"mqtt_publish_data_task",2024,NULL,3,NULL);
}


