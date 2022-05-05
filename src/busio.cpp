#include "busio.h"

busio::busio(i2c *i2cdevice, uint16_t reg_addr, uint8_t width,
             uint8_t byteorder, uint8_t address_width) {
    _i2cdevice = i2cdevice;
    _addrwidth = address_width;
    _address = reg_addr;
    _byteorder = byteorder;
    _width = width;
}

bool busio::write(uint8_t *buffer, uint8_t len) {

    uint8_t addrbuffer[2] = {(uint8_t)(_address & 0xFF),
                             (uint8_t)(_address >> 8)};

    return _i2cdevice->write(buffer, len, true, addrbuffer, _addrwidth);
}

bool busio::write(uint32_t value, uint8_t numbytes) {
    if (numbytes == 0) {
        numbytes = _width;
    }
    if (numbytes > 4) {
        return false;
    }

  // store a copy
    _cached = value;

    for (int i = 0; i < numbytes; i++) {
        if (_byteorder == LSBFIRST) {
            _buffer[i] = value & 0xFF;
        } else {
            _buffer[numbytes - i - 1] = value & 0xFF;
        }
        value >>= 8;
    }
    return write(_buffer, numbytes);
}

uint32_t busio::read(void) {
    if (!read(_buffer, _width)) {
        return -1;
    }

    uint32_t value = 0;

    for (int i = 0; i < _width; i++) {
        value <<= 8;
        if (_byteorder == LSBFIRST) {
            value |= _buffer[_width - i - 1];
        } else {
            value |= _buffer[i];
        }
    }

    return value;
}

uint32_t busio::readCached(void) { return _cached; }

bool busio::read(uint8_t *buffer, uint8_t len) {
    uint8_t addrbuffer[2] = {(uint8_t)(_address & 0xFF),
                             (uint8_t)(_address >> 8)};

    return _i2cdevice->write_then_read(addrbuffer,_addrwidth, buffer, len);
}

bool busio::read(uint16_t *value) {
    if (!read(_buffer, 2)) {
        return false;
    }

    if (_byteorder == LSBFIRST) {
        *value = _buffer[1];
        *value <<= 8;
        *value |= _buffer[0];
    } else {
        *value = _buffer[0];
        *value <<= 8;
        *value |= _buffer[1];
    }
    return true;
}

bool busio::read(uint8_t *value) {
    if (!read(_buffer, 1)) {
        return false;
    }

    *value = _buffer[0];
    return true;
}

void busio::print(Stream *s) {
    uint32_t val = read();
    s->print("0x");
    s->print(val, HEX);
}

void busio::println(Stream *s) {
    print(s);
    s->println();
}

uint8_t busio::width(void) { return _width; }

void busio::setWidth(uint8_t width) { _width = width; }

void busio::setAddress(uint16_t address) { _address = address; }

void busio::setAddressWidth(uint16_t address_width) {
  _addrwidth = address_width;
}

busio_bits::busio_bits(busio *reg, uint8_t bits, uint8_t shift) {
    _register = reg;
    _bits = bits;
    _shift = shift;
}

uint32_t busio_bits::read(void) {
    uint32_t val = _register->read();
    val >>= _shift;
    return val & ((1 << (_bits)) - 1);
}

bool busio_bits::write(uint32_t data) {
    uint32_t val = _register->read();

    uint32_t mask = (1 << (_bits)) - 1;
    data &= mask;

    mask <<= _shift;
    val &= ~mask;
    val |= data << _shift;

    return _register->write(val, _register->width());
}
