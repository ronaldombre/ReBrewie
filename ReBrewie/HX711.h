/*
 * HX711 Arduino driver, adapted from bogde/HX711.
 * Original project: https://github.com/bogde/HX711
 * MIT License, Copyright (c) 2018 Bogdan Necula.
 *
 * The B20 uses the ATmega2560 port-H pins directly, so this adaptation adds
 * begin_port_h() for DOUT=PH2 and SCK=PH7 while retaining the small public
 * API used by the original Arduino library.
 */
#ifndef BREWIE_HX711_H
#define BREWIE_HX711_H

#include <Arduino.h>

class HX711 {
public:
    HX711();

    void begin(byte dout, byte pd_sck, byte gain = 128);
    void begin_port_h(byte dout_bit, byte pd_sck_bit, byte gain = 128);

    bool is_ready();
    void wait_ready(unsigned long delay_ms = 0);
    bool wait_ready_timeout(unsigned long timeout = 1000,
                            unsigned long delay_ms = 0);
    void set_gain(byte gain = 128);
    long read();
    long read_average(byte times = 10);
    double get_value(byte times = 1);
    float get_units(byte times = 1);
    void tare(byte times = 10);
    void set_scale(float scale = 1.0f);
    float get_scale();
    void set_offset(long offset = 0);
    long get_offset();
    void power_down();
    void power_up();

private:
    byte _pd_sck;
    byte _dout;
    byte _gain;
    long _offset;
    float _scale;
    bool _port_h;
    byte _dout_mask;
    byte _sck_mask;

    inline bool read_dout();
    inline void write_sck(bool high);
    inline void configure_port_h();
    byte shift_byte();
};

#endif
