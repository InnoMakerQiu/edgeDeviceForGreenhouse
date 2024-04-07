#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "portmacro.h"
#include "mqtt_event.h"
#include "cJSON.h"
#include "control.h"

const char* TAG = "MQTT_EVENT";

const char* errorParseMessage = "PARSE JSON ERROR";

#define BROKER_URL "mqtt://frp.vhelpyou.eu.org"

esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri = BROKER_URL,
    .broker.address.port = 1883,
    .credentials.client_id = "esp32c3",
    .credentials.username = "esp32c3_1",
    .credentials.authentication.password = "xiaocaiji"
};

esp_mqtt_client_handle_t client;

void parseSubscribedData(const char* json_str);

//这里设置字符转为整形数字的函数
int stringToInteger(const char* str) {
    int result = 0;
    int sign = 1; // 符号位，默认为正数

    // 检查是否为负数
    if (*str == '-') {
        sign = -1;
        str++; // 跳过负号字符
    }

    // 遍历字符串每个字符，计算对应的数字
    while (*str != '\0') {
        if (*str >= '0' && *str <= '9') {
            result = result * 10 + (*str - '0');
        } else {
            // 如果遇到非数字字符，返回0
            return 0;
        }
        str++; // 移动到下一个字符
    }

    return result * sign; // 返回最终结果，包括符号位
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, commandTopic, 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        parseSubscribedData(event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}


void mqtt_app_start(void)
{
    client = esp_mqtt_client_init(&mqtt_cfg);
    if(client==NULL){
        ESP_LOGE(TAG,"The MQTT Client is NULL.");
    }
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

// 上传指令解析的状态
void upLoadFeedBack(const char* controlDeviceId, const char* status, const char* feedback) {
    // 创建 cJSON 对象
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        // 处理创建 cJSON 对象失败的情况
        ESP_LOGW(TAG,"Failed to create cJSON object.\n");
        return;
    }

    // 添加键值对到 cJSON 对象
    cJSON_AddStringToObject(root, "controlDeviceId", controlDeviceId);
    cJSON_AddStringToObject(root, "status", status);
    if (feedback == NULL) {
        cJSON_AddStringToObject(root, "feedback", "");
    } else {
        cJSON_AddStringToObject(root, "feedback", feedback);
    }

    // 序列化 cJSON 对象为字符串
    char *jsonString = cJSON_PrintUnformatted(root);
    if (jsonString == NULL) {
        // 处理序列化 cJSON 对象失败的情况
        ESP_LOGW(TAG,"Failed to serialize cJSON object.\n");
        cJSON_Delete(root); // 释放 cJSON 对象
        return;
    }

    // 输出序列化后的 JSON 字符串
    ESP_LOGI(TAG,"%s\n", jsonString);

    // 发布 MQTT 消息
    esp_mqtt_client_publish(client,resultTopic, jsonString,strlen(jsonString)\
        ,1,0);

    // 释放内存
    free(jsonString);
    cJSON_Delete(root);
}


void parseSubscribedData(const char* json_str){
    // 解析 JSON 字符串
    cJSON *root = cJSON_Parse(json_str);
    ControlCommand controlCommand;
    int value;
    
    if (root == NULL) {
        ESP_LOGW(TAG,"%s",errorParseMessage);
        upLoadFeedBack(NULL, "FAIL", errorParseMessage);
        return;
    }

    // 查找 "command" 字段
    cJSON *command = cJSON_GetObjectItemCaseSensitive(root, "command");
    if (command == NULL) {
        ESP_LOGW(TAG,"Invalid JSON: 'command' field not found or not an object.\n");
        cJSON_Delete(root);
        upLoadFeedBack(NULL, "FAIL", errorParseMessage);
        return;
    }

    // 查找“controlDeviceId”字段
    cJSON *controlDeviceId = cJSON_GetObjectItemCaseSensitive(command, "controlDeviceId");
    if (controlDeviceId == NULL) {
        ESP_LOGW(TAG,"Invalid JSON: 'controlDeviceId' field not found or not an object.\n");
        cJSON_Delete(root);
        upLoadFeedBack(NULL, "FAIL", errorParseMessage);
        return;
    }
        //获取“controlDeviceId”字段
    const char *controlDeviceId_str = controlDeviceId->valuestring;
    
    // 查找 "parameters" 字段
    cJSON *parameters = cJSON_GetObjectItemCaseSensitive(command, "parameters");
    if (parameters == NULL) {
        ESP_LOGW(TAG,"Invalid JSON: 'parameters' field not found or not an object.\n");
        cJSON_Delete(root);
        upLoadFeedBack(controlDeviceId_str, "FAIL", errorParseMessage);
        return;
    }

    // 查找 "value" 字段
    cJSON *valueJSON = cJSON_GetObjectItemCaseSensitive(parameters, "value");
    if (valueJSON == NULL) {
        ESP_LOGW(TAG,"Invalid JSON: 'value' field not found or not a string.\n");
        cJSON_Delete(root);
        upLoadFeedBack(controlDeviceId_str, "FAIL", errorParseMessage);
        return;
    }

    // 查找 "type" 字段
    cJSON *type = cJSON_GetObjectItemCaseSensitive(parameters, "type");
    if (type == NULL) {
        ESP_LOGW(TAG,"Invalid JSON: 'type' field not found or not a string.\n");
        cJSON_Delete(root);
        upLoadFeedBack(controlDeviceId_str, "FAIL", errorParseMessage);
        return;
    }

    // 获取 "value" 字段的值
    const char *value_str = valueJSON->valuestring;
    // 获取"type_str"字段的值
    const char *type_str = type->valuestring;

    ESP_LOGI(TAG,"controlDeviceId is %s",controlDeviceId_str);
    ESP_LOGI(TAG,"Type: %s\n", type_str);
    ESP_LOGI(TAG,"Value: %s\n", value_str);
    
    value = stringToInteger(value_str);
    controlCommand.controlDeviceID = 0;
    controlCommand.commandValue = value;

    //这里根据不同设备id，进行不同的controlCommand赋值操作
    if(!strcmp(controlDeviceId_str,stepper1_device_str)){
        controlCommand.controlDeviceID = STEPPER1_DEVICE;
        if(strcmp(type_str,"rise")){
            controlCommand.commandValue = 0-value;
        }
    }
    else if(!strcmp(controlDeviceId_str,relay1_device_str)){
        controlCommand.controlDeviceID = RELAY1_DEVICE;
    }
    else if(!strcmp(controlDeviceId_str,relay2_device_str)){
        controlCommand.controlDeviceID = RELAY2_DEVICE;
    }

    //如果没有找到制定设备也返回错误
    if(controlCommand.controlDeviceID==0){
        upLoadFeedBack(controlDeviceId_str, "FAIL", errorParseMessage);
        return;
    }
    //这里向指令队列传递控制指令
    xQueueSend(controlQueue,&controlCommand,portMAX_DELAY);
    upLoadFeedBack(controlDeviceId_str, "OK", NULL);

    // 释放 cJSON 对象
    cJSON_Delete(root);
    return;
}



