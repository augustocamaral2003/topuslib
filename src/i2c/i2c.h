#ifndef i2c_h
#define i2c_h

#include <Arduino.h>
#include <Wire.h>

class i2c {
    public:
        i2c(uint8_t adrr, TwoWire *theWire = &Wire);
        uint8_t address(void);
        bool begin(bool addr_detect = true);
        void end(void);
        bool detected(void);

        bool read(uint8_t *buffer, size_t len, bool stop = true);
        bool write(const uint8_t *buffer, size_t len, bool stop = true,
                   const uint8_t *prefix_buffer = NULL, size_t prefix_len = 0);

        size_t maxBufferSize() { return _maxBufferSize; }

    private:
        uint8_t _addr;
        TwoWire *_wire;
        bool _begun;
        size_t _maxBufferSize;
        bool _read(uint8_t *buffer, size_t len, bool stop);
};

#endif // i2c.h
