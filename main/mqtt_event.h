#ifndef MQTT_TCP_H
#define MQTT_TCP_H
#include "mqtt_client.h"
void mqtt_app_start();
extern esp_mqtt_client_handle_t client;
extern char* resultTopic;
extern char* statusTopic;
extern char* commandTopic;
extern char* sensorDataTopic;

#endif