#ifndef CONTROL_H
#define CONTROL_H
#include "FreeRTOS.h"
#include "queue.h"

// 定义消息队列句柄
extern QueueHandle_t controlQueue;

extern void control_task(void *args);

#define STEPPER1_DEVICE 1
#define RELAY1_DEVICE 2
#define RELAY2_DEVICE 3

// 结构体定义设备状态数据
typedef struct DeviceStatus {
    int stepper1_absoluteDistance;
    int relay1_isOn;
    int relay2_isOn;
}DeviceStatus;

// 定义控制指令结构体
typedef struct {
    int controlDeviceID; // 控制设备的标识
    int commandValue; // 控制指令的值
} ControlCommand;


extern char* stepper1_device_str;
extern char* relay1_device_str;
extern char* relay2_device_str;
extern char* majorDeviceId;


extern DeviceStatus* pDeviceStatus;

#endif