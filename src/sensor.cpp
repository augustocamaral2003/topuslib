#include "sensor.h"

void sensor::printSensorDetails(void) {
    sensor_t sensor;
    getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.print(F("Sensor:       "));
    Serial.println(sensor.name);
    Serial.print(F("Driver Ver:   "));
    Serial.println(sensor.version);
    Serial.print(F("Unique ID:    "));
    Serial.println(sensor.sensor_id);
    Serial.print(F("Min Value:    "));
    Serial.println(sensor.min_value);
    Serial.print(F("Max Value:    "));
    Serial.println(sensor.max_value);
    Serial.print(F("Resolution:   "));
    Serial.println(sensor.resolution);
    Serial.println(F("------------------------------------\n"));
}
