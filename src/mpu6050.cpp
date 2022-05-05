#include "mpu6050.h"

mpu6050::mpu6050(void) {}

mpu6050::~mpu6050(void) {
    if (temp_sensor)
        delete temp_sensor;
    if (accel_sensor)
        delete accel_sensor;
    if (gyro_sensor)
        delete gyro_sensor;
    if (i2c_dev)
        delete i2c_dev;
}

bool mpu6050::begin(uint8_t i2c_address, TwoWire *wire, int32_t sensor_id) {
    if (i2c_dev)
        delete i2c_dev;

    i2c_dev = new i2c(i2c_address, wire);

    if (!i2c_dev->begin())
        return false;

    busio chip_id = busio(i2c_dev, MPU6050_WHO_AM_I, 1);

    if (chip_id.read() != MPU6050_DEVICE_ID)
        return false;

    return _init(sensor_id);
}

bool mpu6050::_init(int32_t sensor_id) {
    _sensorid_accel = sensor_id;
    _sensorid_gyro  = sensor_id + 1;
    _sensorid_temp  = sensor_id + 2;

    reset();

    //for setting values refer to adafruit mpu6050
    //original header source code
    //default values in enums as of 05/05/2022

    //sample rate divisor setting
    busio sample_rate_div = busio(i2c_dev, MPU6050_SMPLRT_DIV, 1);
    sample_rate_div.write(MPU6050_SMP_RT_DIV);

    //filter bandwidth setting
    busio config = busio(i2c_dev, MPU6050_CONFIG, 1);
    busio_bits filter_config = busio_bits(&config, 3, 0);
    filter_config.write(MPU6050_FILTER_BDW);
   
    //gyro range setting
    busio gyro_config = busio(i2c_dev, MPU6050_GYRO_CONFIG, 1);
    busio_bits gyro_range = busio_bits(&gyro_config, 2, 3);
    gyro_range.write(MPU6050_GYRO_RANGE);
   
    //accelerometer range setting
    busio accel_config = busio(i2c_dev, MPU6050_ACCEL_CONFIG, 1);
    busio_bits accel_range = busio_bits(&accel_config, 2, 3);
    accel_range.write(MPU6050_ACCEL_RANGE);

    busio power_mgmt_1 = busio(i2c_dev, MPU6050_PWR_MGMT_1, 1);

    power_mgmt_1.write(0x01);

    delay(100);

    if (temp_sensor)
        delete temp_sensor;
    if (accel_sensor)
        delete accel_sensor;
    if (gyro_sensor)
        delete gyro_sensor;

    temp_sensor = new mpu6050_temp(this);
    accel_sensor = new mpu6050_accelerometer(this);
    gyro_sensor = new mpu6050_gyro(this);

    return true;
}

void mpu6050::reset(void) {
    busio power_mgmt_1 = busio(i2c_dev, MPU6050_PWR_MGMT_1, 1);
    busio sig_path_reset = busio(i2c_dev, MPU6050_SIGNAL_PATH_RESET, 1);
    busio_bits device_reset = busio_bits(&power_mgmt_1, 1, 7);

    device_reset.write(1);
    while (device_reset.read() == 1) {
        delay(1);
    }
    delay(100);

    sig_path_reset.write(0x7);

    delay(100);
}

bool mpu6050::enableSleep(bool enable) {
    busio pwr_mgmt_1 = busio(i2c_dev, MPU6050_PWR_MGMT_1, 1);
    busio_bits sleep = busio_bits(&pwr_mgmt_1, 1, 6);

    return sleep.write(enable);
}

void mpu6050::_read(void) {
    busio data_reg = busio(i2c_dev, MPU6050_ACCEL_OUT, 14);

    uint8_t buffer[14];
    data_reg.read(buffer, 14);

    rawAccX = buffer[0] << 8 | buffer[1];
    rawAccY = buffer[2] << 8 | buffer[3];
    rawAccZ = buffer[4] << 8 | buffer[5];

    rawTemp = buffer[6] << 8 | buffer[7];

    rawGyroX = buffer[8] << 8 | buffer[9];
    rawGyroY = buffer[10] << 8 | buffer[11];
    rawGyroZ = buffer[12] << 8 | buffer[13];

    temperature = (rawTemp / 340.0) + 36.53;

    float accel_scale;
    if (MPU6050_ACCEL_RANGE == 0b11)
        accel_scale = 2048;
    if (MPU6050_ACCEL_RANGE == 0b10)
        accel_scale = 4096;
    if (MPU6050_ACCEL_RANGE == 0b01)
        accel_scale = 8192;
    if (MPU6050_ACCEL_RANGE == 0b00)
        accel_scale = 16384;

    accX = ((float) rawAccX) / accel_scale;
    accY = ((float) rawAccY) / accel_scale;
    accZ = ((float) rawAccZ) / accel_scale;

    float gyro_scale;
    if (MPU6050_GYRO_RANGE == 0)
        gyro_scale = 131;
    if (MPU6050_GYRO_RANGE == 1)
        gyro_scale = 65.5;
    if (MPU6050_GYRO_RANGE == 2)
        gyro_scale = 32.8;
    if (MPU6050_GYRO_RANGE == 3);
        gyro_scale == 16.4;

    gyroX = ((float) rawGyroX) / gyro_scale;
    gyroY = ((float) rawGyroY) / gyro_scale;
    gyroZ = ((float) rawGyroZ) / gyro_scale;
}

bool mpu6050::getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                       sensors_event_t *temp) {
    uint32_t timestamp = millis();
    _read();

    fillTempEvent(temp, timestamp);
    fillAccelEvent(accel, timestamp);
    fillGyroEvent(gyro, timestamp);

    return true;
}

void mpu6050::fillTempEvent(sensors_event_t *temp, uint32_t timestamp) {

    memset(temp, 0, sizeof(sensors_event_t));
    temp->version = sizeof(sensors_event_t);
    temp->sensor_id = _sensorid_temp;
    temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    temp->timestamp = timestamp;
    temp->temperature = temperature;
}

void mpu6050::fillAccelEvent(sensors_event_t *accel,
                             uint32_t timestamp) {

    memset(accel, 0, sizeof(sensors_event_t));
    accel->version = 1;
    accel->sensor_id = _sensorid_accel;
    accel->type = SENSOR_TYPE_ACCELEROMETER;
    accel->timestamp = timestamp;
    accel->acceleration.x = accX * SENSORS_GRAVITY_STANDARD;
    accel->acceleration.y = accY * SENSORS_GRAVITY_STANDARD;
    accel->acceleration.z = accZ * SENSORS_GRAVITY_STANDARD;
}

void mpu6050::fillGyroEvent(sensors_event_t *gyro,
                            uint32_t timestamp) {
    memset(gyro, 0, sizeof(sensors_event_t));
    gyro->version = 1;
    gyro->sensor_id = _sensorid_gyro;
    gyro->type = SENSOR_TYPE_GYROSCOPE;
    gyro->timestamp = timestamp;
    gyro->gyro.x = gyroX * SENSORS_DPS_TO_RADS;
    gyro->gyro.y = gyroY * SENSORS_DPS_TO_RADS;
    gyro->gyro.z = gyroZ * SENSORS_DPS_TO_RADS;
}

sensor *mpu6050::getTemperatureSensor(void) {
    return temp_sensor;
}

sensor *mpu6050::getAccelerometerSensor(void) {
    return accel_sensor;
}

sensor *mpu6050::getGyroSensor(void) {
    return gyro_sensor;
}

void mpu6050_gyro::getSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));

    strncpy(sensor->name, "MPU6050_G", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_GYROSCOPE;
    sensor->min_delay = 0;
    sensor->min_value = -34.91; 
    sensor->max_value = +34.91;
    sensor->resolution = 1.332e-4; 
}

bool mpu6050_gyro::getEvent(sensors_event_t *event) {
    _thempu6050->_read();
    _thempu6050->fillGyroEvent(event, millis());

    return true;
}

void mpu6050_accelerometer::getSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));

    strncpy(sensor->name, "MPU6050_A", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_ACCELEROMETER;
    sensor->min_delay = 0;
    sensor->min_value = -156.9064F;
    sensor->max_value = 156.9064F;
    sensor->resolution = 0.061;
}

bool mpu6050_accelerometer::getEvent(sensors_event_t *event) {
    _thempu6050->_read();
    _thempu6050->fillAccelEvent(event, millis());

    return true;
}

void mpu6050_temp::getSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));

    strncpy(sensor->name, "MPU6050_T", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    sensor->min_delay = 0;
    sensor->min_value = -40;
    sensor->max_value = 105;
    sensor->resolution = 0.00294;
}

bool mpu6050_temp::getEvent(sensors_event_t *event) {
    _thempu6050->_read();
    _thempu6050->fillTempEvent(event, millis());

    return true;
}
