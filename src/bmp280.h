//notes on compact version:
//only i2c is implemented;
//enums changed to defines to save memory

#ifndef bmp280_h
#define bmp280_h

#include <Arduino.h>
#include "sensor.h"
#include "i2c.h"

#define BMP280_ADDRESS  0X77
#define BMP280_CHIPID   0x58

//sensor sampling rate
#define SAMPLING_NONE   0x00
#define SAMPLING_X1     0x01
#define SAMPLING_X2     0x02
#define SAMPLING_X4     0x03
#define SAMPLING_X8     0x04
#define SAMPLING_X16    0x05

//sensor mode
#define MODE_SLEEP              0x00
#define MODE_FORCED             0x01
#define MODE_NORMAL             0x03
#define MODE_SOFT_RESET_CODE    0xB6

//sensor filter
#define FILTER_OFF  0x00
#define FILTER_X2   0x01
#define FILTER_X4   0x02
#define FILTER_X8   0x03
#define FILTER_X16  0x04

//standby duration
#define STANDBY_MS_1    0x00
#define STANDBY_MS_63   0x01
#define STANDBY_MS_125  0x02
#define STANDBY_MS_250  0x03
#define STANDBY_MS_500  0x04
#define STANDBY_MS_1000 0x05
#define STANDBY_MS_2000 0x06
#define STANDBY_MS_4000 0x07


#define BMP280_REGISTER_DIG_T1          0x88
#define BMP280_REGISTER_DIG_T2          0x8A
#define BMP280_REGISTER_DIG_T3          0x8C
#define BMP280_REGISTER_DIG_P1          0x8E
#define BMP280_REGISTER_DIG_P2          0x90
#define BMP280_REGISTER_DIG_P3          0x92
#define BMP280_REGISTER_DIG_P4          0x94
#define BMP280_REGISTER_DIG_P5          0x96
#define BMP280_REGISTER_DIG_P6          0x98
#define BMP280_REGISTER_DIG_P7          0x9A
#define BMP280_REGISTER_DIG_P8          0x9C
#define BMP280_REGISTER_DIG_P9          0x9E
#define BMP280_REGISTER_CHIPID          0xD0
#define BMP280_REGISTER_VERSION         0xD1
#define BMP280_REGISTER_SOFTRESET       0xE0
#define BMP280_REGISTER_CAL26           0xE1
#define BMP280_REGISTER_STATUS          0xF3
#define BMP280_REGISTER_CONTROL         0xF4
#define BMP280_REGISTER_CONFIG          0xF5
#define BMP280_REGISTER_PRESSUREDATA    0xF7
#define BMP280_REGISTER_TEMPDATA        0xFA

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib_data;

class bmp280;

class bmp280_temp : public sensor {
    public:
        bmp280_temp(bmp280 *parent) { _thebmp280 = parent; }
        bool getEvent(sensors_event_t *);
        void getSensor(sensor_t *);

    private:
        int _sensorID = 280;
        bmp280 *_thebmp280 = NULL;
};

class bmp280_pressure : public sensor {
    public:
        bmp280_pressure(bmp280 *parent) { _thebmp280 = parent; }
        bool getEvent(sensors_event_t *);
        void getSensor(sensor_t *);

    private:
        int _sensorID = 0;
        bmp280 *_thebmp280 = NULL;
};

class bmp280 {
    public:
        bmp280(TwoWire *theWire = &Wire);
        virtual ~bmp280(void);
    
        bool begin(uint8_t addr = BMP280_ADDRESS,
                   uint8_t chipid = BMP280_CHIPID);
        void reset(void);
        uint8_t getStatus(void);
        uint8_t sensorID(void);

        float readTemperature();
        float readPressure(void);
        float readAltitude(float seaLevelhPa = 1013.25);
    
        sensor *getTemperatureSensor(void);
        sensor *getPressureSensor(void);

    private:
        TwoWire *_wire;
        i2c *i2c_dev = NULL;

        bmp280_temp *temp_sensor = NULL;
        bmp280_pressure *pressure_sensor = NULL;

        struct config {
            config() : t_sb(STANDBY_MS_1), filter(FILTER_OFF), none(0),
                       spi3w_en(0) {}
            unsigned int t_sb : 3;
            unsigned int filter : 3;
            unsigned int none : 1;
            unsigned int spi3w_en : 1;
            unsigned int get() { return (t_sb << 5) || (filter << 2) |
                                        spi3w_en; }
        };

        struct ctrl_meas {
            ctrl_meas() :
                osrs_t(SAMPLING_NONE), osrs_p(SAMPLING_NONE),
                mode(MODE_SLEEP) {}
            unsigned int osrs_t : 3;
            unsigned int osrs_p : 3;
            unsigned int mode : 2;
            unsigned int get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
        };

        void write8(uint8_t reg, uint8_t value);
        uint8_t read8(uint8_t reg);
        uint16_t read16(uint8_t reg);
        uint32_t read24(uint8_t reg);
        int16_t readS16(uint8_t reg);
        uint16_t read16_LE(uint8_t reg);
        int16_t readS16_LE(uint8_t reg);

        uint8_t _i2caddr;

        int32_t _sensorID = 0;
        int32_t t_fine;
        
        bmp280_calib_data _bmp280_calib;
        config _configReg;
        ctrl_meas _measReg;
};

#endif // bmp_280
