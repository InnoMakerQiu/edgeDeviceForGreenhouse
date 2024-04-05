#ifndef MQTT_PUBLISH_H
#define MQTT_PUBLISH_H

void mqtt_publish_data_task();
typedef struct SensorData{
    int temperature;
    int humidity;
    int soilMoisture;
    int CO2Data;
    int lightTensity;
}SensorData;

extern SensorData* pSensorData;


#endif