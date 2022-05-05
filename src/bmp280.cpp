#include "bmp280.h"

bmp280::bmp280(TwoWire *theWire) {
    _wire = theWire;
    temp_sensor = new bmp280_temp(this);
    pressure_sensor = new bmp280_pressure(this);
}

bmp280::~bmp280(void) {
    if (i2c_dev)
        delete i2c_dev;
    if (temp_sensor)
        delete temp_sensor;
    if (pressure_sensor)
        delete pressure_sensor;
}

bool bmp280::begin(uint8_t addr, uint8_t chipid) {
    if (i2c_dev)
        delete i2c_dev;
    i2c_dev = new i2c(addr, _wire);

    if (!i2c_dev->begin())
        return false;

    _sensorID = read8(BMP280_REGISTER_CHIPID);
    if (_sensorID != chipid)
        return false;

    _bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
    _bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
    _bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

    _bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
    _bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
    _bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
    _bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
    _bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
    _bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
    _bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
    _bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
    _bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);

    _measReg.mode = MODE_NORMAL;
    _measReg.osrs_t = SAMPLING_X16;
    _measReg.osrs_p = SAMPLING_X16;

    _configReg.filter = FILTER_OFF;
    _configReg.t_sb = STANDBY_MS_1;

    write8(BMP280_REGISTER_CONFIG, _configReg.get());
    write8(BMP280_REGISTER_CONTROL, _measReg.get());

    delay(100);
    return true;
}

void bmp280::write8(uint8_t reg, uint8_t value) {
    uint8_t buffer[2];

    buffer[1] = value;
    buffer[0] = reg;
    i2c_dev->write(buffer, 2);
}

uint8_t bmp280::read8(uint8_t reg) {
    uint8_t buffer[1];

    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 1);

    return buffer[0];
}

uint16_t bmp280::read16(uint8_t reg) {
    uint8_t buffer[2];

    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 2);

    return uint16_t(buffer[0]) << 8 | uint16_t(buffer[1]);
}

uint16_t bmp280::read16_LE(uint8_t reg) {
    uint16_t temp = read16(reg);
    return (temp >> 8) | (temp << 8);
}

int16_t bmp280::readS16(uint8_t reg) {
    return (int16_t) read16(reg);
}

int16_t bmp280::readS16_LE(uint8_t reg) {
    return (int16_t) read16_LE(reg);
}

uint32_t bmp280::read24(uint8_t reg) {
    uint8_t buffer[3];

    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 3);

    return uint32_t(buffer[0]) << 16 | uint32_t(buffer[1]) << 8 |
           uint32_t(buffer[2]);
}

float bmp280::readTemperature() {
    int32_t var1, var2;
    if (!_sensorID)
        return NAN;

    int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
    adc_T >>= 4;

    var1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) *
           ((int32_t)_bmp280_calib.dig_T2)) >>
           11;

    var2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1)) *
           ((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >>
           12) * ((int32_t)_bmp280_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    float T = (t_fine * 5 + 128) >> 8;
    return T / 100;
}

float bmp280::readPressure() {
    int64_t var1, var2, p;
    if (!_sensorID)
        return NAN;

    readTemperature();

    int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)_bmp280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
           ((var1 * (int64_t)_bmp280_calib.dig_P2) << 12);
    var1 =
       (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

    if (var1 == 0)
        return 0;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
    return (float)p / 256;
}

float bmp280::readAltitude(float seaLevelhPa) {
    float altitude;
    float pressure = readPressure();

    pressure /= 100;

    altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

    return altitude;
}

void bmp280::reset(void) {
    write8(BMP280_REGISTER_SOFTRESET, MODE_SOFT_RESET_CODE);
}

uint8_t bmp280::sensorID(void) { return _sensorID; }

uint8_t bmp280::getStatus(void) {
    return read8(BMP280_REGISTER_STATUS);
}

sensor *bmp280::getTemperatureSensor(void) {
    return temp_sensor;
}

sensor *bmp280::getPressureSensor(void) {
    return pressure_sensor;
}

void bmp280_temp::getSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));

    strncpy(sensor->name, "BMP280", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    sensor->min_delay = 0;
    sensor->min_value = -40.0;
    sensor->max_value = +85.0;
    sensor->resolution = 0.01;
}

bool bmp280_temp::getEvent(sensors_event_t *event) {
    memset(event, 0, sizeof(sensors_event_t));

    event->version = sizeof(sensors_event_t);
    event->sensor_id = _sensorID;
    event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    event->timestamp = millis();
    event->temperature = _thebmp280->readTemperature();
    return true;
}

void bmp280_pressure::getSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));

    strncpy(sensor->name, "BMP280", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_PRESSURE;
    sensor->min_delay = 0;
    sensor->min_value = 300.0;
    sensor->max_value = 1100.0;
    sensor->resolution = 0.012;
}

bool bmp280_pressure::getEvent(sensors_event_t *event) {
    memset(event, 0, sizeof(sensors_event_t));

    event->version = sizeof(sensors_event_t);
    event->sensor_id = _sensorID;
    event->type = SENSOR_TYPE_PRESSURE;
    event->timestamp = millis();
    event->pressure = _thebmp280->readPressure() / 100;
    return true;
}
