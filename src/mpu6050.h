#ifndef mpu6050_h
#define mpu6050_h

#include <Arduino.h>
#include "busio.h"
#include "i2c.h"
#include "sensor.h"
#include <Wire.h>

#define MPU6050_I2CADDR_DEFAULT     0x68 
#define MPU6050_DEVICE_ID           0x68 
#define MPU6050_SELF_TEST_X         0x0D 
#define MPU6050_SELF_TEST_Y         0x0E 
#define MPU6050_SELF_TEST_Z         0x0F 
#define MPU6050_SELF_TEST_A         0x10 
#define MPU6050_SMPLRT_DIV          0x19 
#define MPU6050_CONFIG              0x1A
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_INT_PIN_CONFIG      0x37
#define MPU6050_INT_ENABLE          0x38
#define MPU6050_INT_STATUS          0x3A
#define MPU6050_WHO_AM_I            0x75
#define MPU6050_SIGNAL_PATH_RESET   0x68
#define MPU6050_USER_CTRL           0x6A
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_PWR_MGMT_2          0x6C
#define MPU6050_TEMP_H              0x41
#define MPU6050_TEMP_L              0x42
#define MPU6050_ACCEL_OUT           0x3B
#define MPU6050_MOT_THR             0x1F
#define MPU6050_MOT_DUR             0x20

#define MPU6050_SMP_RT_DIV  0
#define MPU6050_FILTER_BDW  0
#define MPU6050_GYRO_RANGE  1
#define MPU6050_ACCEL_RANGE 0b11

class mpu6050;

class mpu6050_temp : public sensor {
    public:
        mpu6050_temp(mpu6050 *parent) { _thempu6050 = parent; }
        bool getEvent(sensors_event_t *);
        void getSensor(sensor_t *);

    private:
        int _sensorID = 0x650;
        mpu6050 *_thempu6050 = NULL;
};

class mpu6050_accelerometer : public sensor {
    public:
        mpu6050_accelerometer(mpu6050 *parent) { _thempu6050 = parent; }
        bool getEvent(sensors_event_t *);
        void getSensor(sensor_t *);

    private:
        int _sensorID = 0x651;
        mpu6050 *_thempu6050 = NULL;
};

class mpu6050_gyro : public sensor {
    public:
        mpu6050_gyro(mpu6050 *parent) { _thempu6050 = parent; }
        bool getEvent(sensors_event_t *);
        void getSensor(sensor_t *);

    private:
        int _sensorID = 0x650;
        mpu6050 *_thempu6050 = NULL;
};

class mpu6050 {
    public:
        mpu6050();
        virtual ~mpu6050();

        bool begin(uint8_t i2c_addr = MPU6050_I2CADDR_DEFAULT,
                   TwoWire *wire = &Wire, int32_t sensorID = 0);

        bool getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                      sensors_event_t *temp);

        bool enableSleep(bool enable);

        void reset(void);

        sensor *getTemperatureSensor(void);
        sensor *getAccelerometerSensor(void);
        sensor *getGyroSensor(void);

    protected:
        float temperature,
              accX, accY, accZ,
              gyroX, gyroY, gyroZ;

        i2c *i2c_dev = NULL;

        mpu6050_temp *temp_sensor = NULL;
        mpu6050_accelerometer *accel_sensor = NULL;
        mpu6050_gyro *gyro_sensor = NULL;

        uint16_t _sensorid_accel,
                 _sensorid_gyro,
                 _sensorid_temp;

        void _read(void);
        virtual bool _init(int32_t sensor_id);

    private:
        friend class mpu6050_temp;
        friend class mpu6050_accelerometer;
        friend class mpu6050_gyro;

        int16_t rawAccX, rawAccY, rawAccZ,
                rawTemp,
                rawGyroX, rawGyroY, rawGyroZ;

        void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
        void fillAccelEvent(sensors_event_t *accel, uint32_t timestamp);
        void fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp);
};

#endif // mpu6050_h
