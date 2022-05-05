#include "i2c.h"

i2c::i2c(uint8_t addr, TwoWire *theWire) {
    _addr = addr;
    _wire = theWire;
    _begun = false;
    _maxBufferSize = 32;
}

bool i2c::begin(bool addr_detect) {
    _wire->begin();
    _begun = true;

    if (addr_detect) {
        return detected();
    }

    return true;
}

void i2c::end(void) {
    _wire->end();
    _begun = false;
}

bool i2c::detected(void) {
    if (!_begun && !begin())
        return false;

    _wire->beginTransmission(_addr);
    if (_wire->endTransmission() == 0)
        return true;

    return false;
}

bool i2c::write(const uint8_t *buffer, size_t len, bool stop,
                const uint8_t *prefix_buffer,
                size_t prefix_len) {
    if ((len + prefix_len) > maxBufferSize())
        return false;

    _wire->beginTransmission(_addr);

    if ((prefix_len != 0) && (prefix_buffer != NULL))
        if (_wire->write(prefix_buffer, prefix_len) != prefix_len)
            return false;

    if (_wire->write(buffer, len) != len)
        return false;

    if (_wire->endTransmission(stop) == 0)
        return true;
    else
        return false;
}

bool i2c::read(uint8_t *buffer, size_t len, bool stop) {
    size_t pos = 0;
    while (pos < len) {
        size_t read_len = 
            ((len - pos) > maxBufferSize() ? maxBufferSize() : (len - pos));
        bool read_stop = (pos < (len - read_len)) ? false : stop;
        if (!_read(buffer + pos, read_len, read_stop))
            return false;
        pos += read_len;
    }
    return true;
}

bool i2c::_read(uint8_t *buffer, size_t len, bool stop) {
    size_t recv = _wire->requestFrom((uint8_t) _addr, (uint8_t) len,
                                     (uint8_t) stop);

    if (recv != len)
        return false;

    for (uint16_t i = 0; i < len; i++)
        buffer[i] = _wire->read();
        
    return true;
}

uint8_t i2c::address(void) { return _addr; }
