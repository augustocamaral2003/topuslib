#ifndef busio_h
#define busio_h

#include <Arduino.h>

#if !defined(SPI_INTERFACES_COUNT) ||                                          \
    (defined(SPI_INTERFACES_COUNT) && (SPI_INTERFACES_COUNT > 0))

#include "i2c/i2c.h"
#include <spidevice/spidevice.h>

typedef enum _busio_spi_RegType {
  ADDRBIT8_HIGH_TOREAD = 0,
  AD8_HIGH_TOREAD_AD7_HIGH_TOINC = 1,
  ADDRBIT8_HIGH_TOWRITE = 2,
  ADDRESSED_OPCODE_BIT0_LOW_TO_WRITE = 3,
} busio_spi_RegType;

/*!
 * @brief The class which defines a device register (a location to read/write
 * data from)
 */
class busio {
    public:
        busio(i2c *i2cdevice, uint16_t reg_addr,
              uint8_t width = 1, uint8_t byteorder = LSBFIRST,
              uint8_t address_width = 1);

        busio(spidevice *spidevice, uint16_t reg_addr,
              busio_spi_RegType type, uint8_t width = 1,
              uint8_t byteorder = LSBFIRST,
              uint8_t address_width = 1);

        busio(i2c *i2cdevice, spidevice *spidevice,
              busio_spi_RegType type, uint16_t reg_addr,
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
        spidevice *_spidevice;
        busio_spi_RegType _spiregtype;
        uint16_t _address;
        uint8_t _width, _addrwidth, _byteorder;
        uint8_t _buffer[4]; // we won't support anything larger than uint32 for
                      // non-buffered read
        uint32_t _cached = 0;
};
