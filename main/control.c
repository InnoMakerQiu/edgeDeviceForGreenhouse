#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
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
#include "driver/ledc.h"
#include "portmacro.h"


#define TAG "CONTROL_TASK"


//下面是设置继电器的控制引脚
#define  RELAY1_IO  GPIO_NUM_12
#define  RELAY2_IO  GPIO_NUM_13



//set the stepper direction
#define PWM_0_DIR_IO_NUM GPIO_NUM_0
//Set ledc parameters below
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (1) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz



//这个任务是用于驱动步进电机，实现距离控制
static void stepper1_task(int moveDistance);


void relay_init(){
    /* 定义gpio配置结构体 */
    gpio_config_t io_conf;
    /* 初始化gpio配置结构体 */
    io_conf.pin_bit_mask = (1ULL << RELAY1_IO); // 选择gpio
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



/* Warning:
 * For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2, ESP32P4 targets,
 * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
 * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
 */

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void ledc1_on(){
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

void ledc1_off(){
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}


void control_task(void *args){
    
    relay_init();
    // 初始化 PWM
    ledc_init();

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
        ledc1_on();
        vTaskDelay(100/portTICK_PERIOD_MS);
        ledc1_off();
    }
    while(pDeviceStatus->stepper1_absoluteDistance>desAbsoluteDistance){
        pDeviceStatus->stepper1_absoluteDistance--;
        gpio_set_level(PWM_0_DIR_IO_NUM, 0);
        ledc1_on();
        vTaskDelay(100/portTICK_PERIOD_MS);
        ledc1_off();
    }
}



