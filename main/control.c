#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pwm.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_event.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_event.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "control.h"

#define TAG "CONTROL_TASK"

// PWM period 1000us(1Khz), same as depth
#define PWM_PERIOD    (2000)
#define PWM_0_OUT_IO_NUM GPIO_NUM_14
#define PWM_0_DIR_IO_NUM GPIO_NUM_2
//下面是设置继电器的控制引脚
#define  RELAY1_IO  GPIO_NUM_0
#define  RELAY2_IO  GPIO_NUM_13

// pwm pin number
const uint32_t pin_num[1] = {
    PWM_0_OUT_IO_NUM
};

// duties table, real_duty = duties[x]/PERIOD
uint32_t duties[1] = {
    1000
};

//这个任务是用于驱动步进电机，实现距离控制
static void stepper1_task(int moveDistance);


void relay_init(){
    /* 定义gpio配置结构体 */
    gpio_config_t io_conf;
    /* 初始化gpio配置结构体 */
    io_conf.pin_bit_mask = (1ULL << RELAY2_IO); // 选择gpio
    io_conf.intr_type = GPIO_INTR_DISABLE; // 禁用中断
    io_conf.mode = GPIO_MODE_OUTPUT; // 设置为输出模式
    io_conf.pull_down_en = 0; // 不用下拉模式
    io_conf.pull_up_en = 0; // 不用上拉模式
    /* 用给定的设置配置GPIO */
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << RELAY2_IO); // 选择gpio
    /* 用给定的设置配置GPIO */
    gpio_config(&io_conf);
    /* 初始化gpio配置结构体 */
    io_conf.pin_bit_mask = (1ULL << PWM_0_DIR_IO_NUM); // 选择gpio
    gpio_config(&io_conf);
}

void control_task(void *args){
    
    relay_init();
        // 初始化 PWM
    pwm_init(PWM_PERIOD,duties,1, pin_num);
    pwm_set_phase(0, 0);
    pwm_start();
    pwm_stop(0);

    ControlCommand receivedCommand;

    while(1) {
        // 从消息队列中接收控制指令
        xQueueReceive(controlQueue, &receivedCommand, portMAX_DELAY);
        
        // 根据设备标识执行相应的控制操作
        switch(receivedCommand.controlDeviceID) {
            case STEPPER1_DEVICE:
                // 执行设备1的控制操作
                ESP_LOGI(TAG,"Stepper device 1 with value %d\n", receivedCommand.commandValue);
                stepper1_task(receivedCommand.commandValue);
                break;
            case RELAY1_DEVICE:
                // 执行设备2的控制操作
                ESP_LOGI(TAG,"Relay device 1 with value %d\n", receivedCommand.commandValue);
                gpio_set_level(RELAY1_IO, receivedCommand.commandValue);
                pDeviceStatus->relay1_isOn = receivedCommand.commandValue;
                break;
            // 可以根据需要添加其他设备的控制操作
            case RELAY2_DEVICE:
                ESP_LOGI(TAG,"Relay device 2 with value %d\n", receivedCommand.commandValue);
                gpio_set_level(RELAY2_IO, receivedCommand.commandValue);
                pDeviceStatus->relay2_isOn = receivedCommand.commandValue;
        }
    }
}

static void stepper1_task(int moveDistance){
    int desAbsoluteDistance = pDeviceStatus->stepper1_absoluteDistance+moveDistance;
    while(pDeviceStatus->stepper1_absoluteDistance<desAbsoluteDistance){
        gpio_set_level(PWM_0_DIR_IO_NUM, 1);
        pDeviceStatus->stepper1_absoluteDistance++;
        pwm_start();
        vTaskDelay(100/portTICK_PERIOD_MS);
        pwm_stop(0);
    }
    while(pDeviceStatus->stepper1_absoluteDistance>desAbsoluteDistance){
        pDeviceStatus->stepper1_absoluteDistance--;
        gpio_set_level(PWM_0_DIR_IO_NUM, 0);
        pwm_start();
        vTaskDelay(100/portTICK_PERIOD_MS);
        pwm_stop(0);
    }
}



