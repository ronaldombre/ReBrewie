/*
 * HX711 Arduino driver, adapted from bogde/HX711.
 * Original project: https://github.com/bogde/HX711
 * MIT License, Copyright (c) 2018 Bogdan Necula.
 */
#include "HX711.h"

HX711::HX711()
    : _pd_sck(0), _dout(0), _gain(1), _offset(0), _scale(1.0f),
      _port_h(false), _dout_mask(0), _sck_mask(0) {}

void HX711::begin(byte dout, byte pd_sck, byte gain) {
    _port_h = false;
    _dout = dout;
    _pd_sck = pd_sck;
    pinMode(_pd_sck, OUTPUT);
    digitalWrite(_pd_sck, LOW);
    pinMode(_dout, INPUT_PULLUP);
    set_gain(gain);
}

void HX711::begin_port_h(byte dout_bit, byte pd_sck_bit, byte gain) {
    _port_h = true;
    _dout_mask = (byte)(1U << dout_bit);
    _sck_mask = (byte)(1U << pd_sck_bit);
    configure_port_h();
    set_gain(gain);
}

inline void HX711::configure_port_h() {
    DDRH &= (byte)~_dout_mask;
    PORTH |= _dout_mask;       // input pull-up on DOUT
    DDRH |= _sck_mask;
    PORTH &= (byte)~_sck_mask; // SCK must idle low
}

inline bool HX711::read_dout() {
    return _port_h ? ((PINH & _dout_mask) != 0) : (digitalRead(_dout) != LOW);
}

inline void HX711::write_sck(bool high) {
    if (_port_h) {
        if (high) {
            PORTH |= _sck_mask;
        } else {
            PORTH &= (byte)~_sck_mask;
        }
    } else {
        digitalWrite(_pd_sck, high ? HIGH : LOW);
    }
}

bool HX711::is_ready() {
    return !read_dout();
}

void HX711::wait_ready(unsigned long delay_ms) {
    while (!is_ready()) {
        delay(delay_ms);
    }
}

bool HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
    unsigned long started = millis();
    while (millis() - started < timeout) {
        if (is_ready()) {
            return true;
        }
        delay(delay_ms);
    }
    return false;
}

void HX711::set_gain(byte gain) {
    switch (gain) {
    case 128: _gain = 1; break; // channel A, gain 128
    case 64:  _gain = 3; break; // channel A, gain 64
    case 32:  _gain = 2; break; // channel B, gain 32
    default:  _gain = 1; break;
    }
}

byte HX711::shift_byte() {
    byte value = 0;
    for (byte bit = 0; bit < 8; ++bit) {
        write_sck(true);
        delayMicroseconds(1);
        value = (byte)(value << 1);
        if (read_dout()) {
            value |= 1;
        }
        write_sck(false);
        delayMicroseconds(1);
    }
    return value;
}

long HX711::read() {
    wait_ready();

    noInterrupts();
    byte data[3];
    data[2] = shift_byte();
    data[1] = shift_byte();
    data[0] = shift_byte();

    // Select channel and gain for the next conversion.
    for (byte i = 0; i < _gain; ++i) {
        write_sck(true);
        delayMicroseconds(1);
        write_sck(false);
        delayMicroseconds(1);
    }
    interrupts();

    byte filler = (data[2] & 0x80) ? 0xFF : 0x00;
    uint32_t value = ((uint32_t)filler << 24) |
                     ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[1] << 8) |
                     data[0];
    return (long)value;
}

long HX711::read_average(byte times) {
    if (times == 0) {
        return 0;
    }
    int64_t sum = 0;
    for (byte i = 0; i < times; ++i) {
        sum += read();
        delay(0);
    }
    return (long)(sum / times);
}

double HX711::get_value(byte times) {
    return (double)read_average(times) - _offset;
}

float HX711::get_units(byte times) {
    return (float)(get_value(times) / _scale);
}

void HX711::tare(byte times) {
    set_offset(read_average(times));
}

void HX711::set_scale(float scale) { _scale = scale; }
float HX711::get_scale() { return _scale; }
void HX711::set_offset(long offset) { _offset = offset; }
long HX711::get_offset() { return _offset; }

void HX711::power_down() {
    write_sck(false);
    write_sck(true);
}

void HX711::power_up() {
    write_sck(false);
}
