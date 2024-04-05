#include "mqtt_publish.h"
#include <stdio.h>
#include "control.h"
#include "esp_err.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "portmacro.h"
#include "cJSON.h"
#include "mqtt_event.h"

static const char *TAG = "MQTT_PUBLISH_TASK";

void numToString(int value,char* string){
    sprintf(string,"%d",value);
}

esp_err_t getGxht30TemperatureMessage(char* sensorString){
    char sensorValueString[10];
    numToString(pSensorData->temperature, sensorValueString);

    cJSON *sensorJSONMessage = cJSON_CreateObject();
    cJSON_AddStringToObject(sensorJSONMessage, "sensorId", "gxht30_1");
    
    cJSON* sensorDataJSONMessage = cJSON_CreateObject();
    cJSON_AddStringToObject(sensorDataJSONMessage, "type", "temperature");
    cJSON_AddStringToObject(sensorDataJSONMessage, "value", sensorValueString);
    cJSON_AddStringToObject(sensorDataJSONMessage, "unit", "C");
    cJSON_AddItemToObject(sensorJSONMessage, "data", sensorDataJSONMessage);
    
    cJSON_bool success = cJSON_PrintPreallocated(sensorJSONMessage, sensorString, 100, 0);
    if(!success){
        ESP_LOGE(TAG,"JSON PRINT ERROR");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG,"%s",sensorString);
    ESP_LOGI(TAG,"the length of String:%d",(int)strlen(sensorString));
    
    return ESP_OK;
}

esp_err_t getGxht30HumdityMessage(char* sensorString){
    char sensorValueString[10];
    numToString(pSensorData->humidity, sensorValueString);

    cJSON *sensorJSONMessage = cJSON_CreateObject();
    cJSON_AddStringToObject(sensorJSONMessage, "sensorId", "gxht30_1");
    
    cJSON* sensorDataJSONMessage = cJSON_CreateObject();
    cJSON_AddStringToObject(sensorDataJSONMessage, "type", "humidity");
    cJSON_AddStringToObject(sensorDataJSONMessage, "value", sensorValueString);
    cJSON_AddStringToObject(sensorDataJSONMessage, "unit", "%");
    cJSON_AddItemToObject(sensorJSONMessage, "data", sensorDataJSONMessage);
    
    cJSON_bool success = cJSON_PrintPreallocated(sensorJSONMessage, sensorString, 100, 0);
    if(!success){
        ESP_LOGE(TAG,"JSON PRINT ERROR");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG,"%s",sensorString);
    ESP_LOGI(TAG,"the length of String:%d",(int)strlen(sensorString));
    
    return ESP_OK;
}

esp_err_t getSoilSensorMessage(char* sensorString){
    char sensorValueString[10];
    numToString(pSensorData->soilMoisture, sensorValueString);

    cJSON *sensorJSONMessage = cJSON_CreateObject();
    cJSON_AddStringToObject(sensorJSONMessage, "sensorId", "soil_moisture_1");
    
    cJSON* sensorDataJSONMessage = cJSON_CreateObject();
    cJSON_AddStringToObject(sensorDataJSONMessage, "type", "soil_moisture");
    cJSON_AddStringToObject(sensorDataJSONMessage, "value", sensorValueString);
    cJSON_AddStringToObject(sensorDataJSONMessage, "unit", "C");
    cJSON_AddItemToObject(sensorJSONMessage, "data", sensorDataJSONMessage);
    
    cJSON_bool success = cJSON_PrintPreallocated(sensorJSONMessage, sensorString, 100, 0);
    if(!success){
        ESP_LOGE(TAG,"JSON PRINT ERROR");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG,"%s",sensorString);
    ESP_LOGI(TAG,"the length of String:%d",(int)strlen(sensorString));
    
    return ESP_OK;
}

esp_err_t getSgp30CO2DataMessage(char* sensorString){
    char sensorValueString[10];
    numToString(pSensorData->soilMoisture, sensorValueString);

    cJSON *sensorJSONMessage = cJSON_CreateObject();
    cJSON_AddStringToObject(sensorJSONMessage, "sensorId", "sgp30_1");
    
    cJSON* sensorDataJSONMessage = cJSON_CreateObject();
    cJSON_AddStringToObject(sensorDataJSONMessage, "type", "co2");
    cJSON_AddStringToObject(sensorDataJSONMessage, "value", sensorValueString);
    cJSON_AddStringToObject(sensorDataJSONMessage, "unit", "ppm");
    cJSON_AddItemToObject(sensorJSONMessage, "data", sensorDataJSONMessage);
    
    cJSON_bool success = cJSON_PrintPreallocated(sensorJSONMessage, sensorString, 100, 0);
    if(!success){
        ESP_LOGE(TAG,"JSON PRINT ERROR");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG,"%s",sensorString);
    ESP_LOGI(TAG,"the length of String:%d",(int)strlen(sensorString));
    
    return ESP_OK;
}

esp_err_t getGy30LightTensityMessage(char* sensorString){
    char sensorValueString[10];
    numToString(pSensorData->soilMoisture, sensorValueString);

    cJSON *sensorJSONMessage = cJSON_CreateObject();
    cJSON_AddStringToObject(sensorJSONMessage, "sensorId", "gy30_1");
    
    cJSON* sensorDataJSONMessage = cJSON_CreateObject();
    cJSON_AddStringToObject(sensorDataJSONMessage, "type", "lightTesity");
    cJSON_AddStringToObject(sensorDataJSONMessage, "value", sensorValueString);
    cJSON_AddStringToObject(sensorDataJSONMessage, "unit", "lx");
    cJSON_AddItemToObject(sensorJSONMessage, "data", sensorDataJSONMessage);
    
    cJSON_bool success = cJSON_PrintPreallocated(sensorJSONMessage, sensorString, 100, 0);
    if(!success){
        cJSON_Delete(sensorJSONMessage);
        ESP_LOGE(TAG,"JSON PRINT ERROR");
        return ESP_FAIL;
    }
    cJSON_Delete(sensorJSONMessage);
    ESP_LOGI(TAG,"%s",sensorString);
    ESP_LOGI(TAG,"the length of String:%d",(int)strlen(sensorString));
    
    return ESP_OK;
}

esp_err_t getWaterPump1ControlMessage(char* controlString) {
    char controlValueString[10];
    numToString(pDeviceStatus->relay1_isOn,controlValueString);

    cJSON *controlMessage = cJSON_CreateObject();
    cJSON_AddStringToObject(controlMessage, "controlDeviceId", relay1_device_str);
    cJSON_AddStringToObject(controlMessage, "controlDeviceType", ""); // 可选
    cJSON_AddStringToObject(controlMessage, "action", ""); // 可选
    
    cJSON *parameters = cJSON_CreateObject();
    cJSON_AddStringToObject(parameters, "type", "isOn");
    cJSON_AddStringToObject(parameters, "value", controlValueString);
    cJSON_AddStringToObject(parameters, "unit", ""); // 可选
    cJSON_AddItemToObject(controlMessage, "parameters", parameters);

    cJSON_bool success = cJSON_PrintPreallocated(controlMessage, controlString, 200, 0);
    if(!success){
        cJSON_Delete(controlMessage);
        ESP_LOGE(TAG,"JSON PRINT ERROR");
        return ESP_FAIL;
    }
    cJSON_Delete(controlMessage);
    ESP_LOGI(TAG,"%s",controlString);
    ESP_LOGI(TAG,"the length of String:%d",(int)strlen(controlString));
    
    return ESP_OK;
}

esp_err_t getFan1ControlMessage(char* controlString) {
    char controlValueString[10];
    numToString(pDeviceStatus->relay2_isOn,controlValueString);

    cJSON *controlMessage = cJSON_CreateObject();
    cJSON_AddStringToObject(controlMessage, "controlDeviceId", relay2_device_str);
    cJSON_AddStringToObject(controlMessage, "controlDeviceType", ""); // 可选
    cJSON_AddStringToObject(controlMessage, "action", ""); // 可选
    
    cJSON *parameters = cJSON_CreateObject();
    cJSON_AddStringToObject(parameters, "type", "isOn");
    cJSON_AddStringToObject(parameters, "value", controlValueString);
    cJSON_AddStringToObject(parameters, "unit", ""); // 可选
    cJSON_AddItemToObject(controlMessage, "parameters", parameters);

    cJSON_bool success = cJSON_PrintPreallocated(controlMessage, controlString, 200, 0);
    if(!success){
        cJSON_Delete(controlMessage);
        ESP_LOGE(TAG,"JSON PRINT ERROR");
        return ESP_FAIL;
    }
    cJSON_Delete(controlMessage);
    ESP_LOGI(TAG,"%s",controlString);
    ESP_LOGI(TAG,"the length of String:%d",(int)strlen(controlString));
    
    return ESP_OK;
}


esp_err_t getRollerControlMessage(char* controlString) {
    char controlValueString[10];
    numToString(pDeviceStatus->stepper1_absoluteDistance,controlValueString);

    cJSON *controlMessage = cJSON_CreateObject();
    cJSON_AddStringToObject(controlMessage, "controlDeviceId", stepper1_device_str);
    cJSON_AddStringToObject(controlMessage, "controlDeviceType", ""); // 可选
    cJSON_AddStringToObject(controlMessage, "action", "execute"); // 可选
    
    cJSON *parameters = cJSON_CreateObject();
    cJSON_AddStringToObject(parameters, "type", "location");
    cJSON_AddStringToObject(parameters, "value", controlValueString);
    cJSON_AddStringToObject(parameters, "unit", ""); // 可选
    cJSON_AddItemToObject(controlMessage, "parameters", parameters);

    cJSON_bool success = cJSON_PrintPreallocated(controlMessage, controlString, 200, 0);
    if(!success){
        ESP_LOGE(TAG,"JSON PRINT ERROR");
        cJSON_Delete(controlMessage);
        return ESP_FAIL;
    }
    cJSON_Delete(controlMessage);
    ESP_LOGI(TAG,"%s",controlString);
    ESP_LOGI(TAG,"the length of String:%d",(int)strlen(controlString));
    
    return ESP_OK;
}

//这里不知道printPrintPreallocated是否有效
void publishSensorData(){
    char sensorString[100];
    if(getSgp30CO2DataMessage(sensorString)==ESP_FAIL){
        ESP_LOGW(TAG,"getSgp30CO2DataMessage error");
    }
    esp_mqtt_client_publish(client,sensorDataTopic,sensorString,strlen(sensorString),1,0);

    if(getSoilSensorMessage(sensorString)==ESP_FAIL){
        ESP_LOGW(TAG,"getSoilSensorMessage error");
    }
    esp_mqtt_client_publish(client,sensorDataTopic,sensorString,strlen(sensorString),1,0);

    if(getGxht30HumdityMessage(sensorString)==ESP_FAIL){
        ESP_LOGW(TAG,"getGxht30HumdityMessage error");
    }
    esp_mqtt_client_publish(client,sensorDataTopic,sensorString,strlen(sensorString),1,0);

    if(getGxht30TemperatureMessage(sensorString)==ESP_FAIL){
        ESP_LOGW(TAG,"getGxht30TemperatureMessage error");
    }
    esp_mqtt_client_publish(client,sensorDataTopic,sensorString,strlen(sensorString),1,0);

    if(getGy30LightTensityMessage(sensorString)==ESP_FAIL){
        ESP_LOGW(TAG,"getGy30LightTensityMessage error");
    }
    esp_mqtt_client_publish(client,sensorDataTopic,sensorString,strlen(sensorString),1,0);
}


void mqtt_publish_data_task(){
    while(1){
        publishSensorData();
        vTaskDelay(20000/portTICK_PERIOD_MS);
        char controlString[100];
        if(getWaterPump1ControlMessage(controlString)==ESP_FAIL){
            ESP_LOGW(TAG,"getWaterPump1ControlMessage error");
        }
        esp_mqtt_client_publish(client,statusTopic,controlString,strlen(controlString),1,0);

        if(getFan1ControlMessage(controlString)==ESP_FAIL){
            ESP_LOGW(TAG,"getFan1ControlMessage error");
        }
        esp_mqtt_client_publish(client,statusTopic,controlString,strlen(controlString),1,0);

        if(getRollerControlMessage(controlString)==ESP_FAIL){
            ESP_LOGW(TAG,"getRollerControlMessage error");
        }
        esp_mqtt_client_publish(client,statusTopic,controlString,strlen(controlString),1,0);
        vTaskDelay(20000/portTICK_PERIOD_MS);
    }
}




