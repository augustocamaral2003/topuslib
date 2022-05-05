#ifndef busio_h
#define busio_h

#include <Arduino.h>

#include "i2c.h"

typedef enum _busio_spi_RegType {
    ADDRBIT8_HIGH_TOREAD = 0,
    AD8_HIGH_TOREAD_AD7_HIGH_TOINC = 1,
    ADDRBIT8_HIGH_TOWRITE = 2,
    ADDRESSED_OPCODE_BIT0_LOW_TO_WRITE = 3,
} busio_spi_RegType;

class busio {
    public:
        busio(i2c *i2cdevice, uint16_t reg_addr,
              uint8_t width = 1, uint8_t byteorder = LSBFIRST,
              uint8_t address_width = 1);

        bool read(uint8_t *buffer, uint8_t len);
        bool read(uint8_t *value);
        bool read(uint16_t *value);
        uint32_t read(void);
        uint32_t readCached(void);
        bool write(uint8_t *buffer, uint8_t len);
        bool write(uint32_t value, uint8_t numbytes = 0);

        uint8_t width(void);

        void setWidth(uint8_t width);
        void setAddress(uint16_t address);
        void setAddressWidth(uint16_t address_width);

        void print(Stream *s = &Serial);
        void println(Stream *s = &Serial);

    private:
        i2c *_i2cdevice;
        busio_spi_RegType _spiregtype;
        uint16_t _address;
        uint8_t _width, _addrwidth, _byteorder;
        uint8_t _buffer[4];
        uint32_t _cached = 0;
};

class busio_bits {
    public:
        busio_bits(busio *reg, uint8_t bits, uint8_t shift);
        bool write(uint32_t value);
        uint32_t read(void);

    private:
        busio *_register;
        uint8_t _bits, _shift;
};

#endif // busio_h
