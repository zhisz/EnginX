#ifndef __BUZZER_MUSIC_H
#define __BUZZER_MUSIC_H

#include "buzzer.h"

static const uint8_t m_two_tigers[] = {
    DO, 50, RE, 50, MI, 50, DO, 50,
    DO, 50, RE, 50, MI, 50, DO, 50,
    MI, 50, FA, 50, SO, 100,
    MI, 50, FA, 50, SO, 100,
    SO, 25, LA, 25, SO, 25, FA, 25, MI, 50, DO, 50,
    SO, 25, LA, 25, SO, 25, FA, 25, MI, 50, DO, 50,
    MI, 50, SO_, 50, RE, 100,
    MI, 50, SO_, 50, RE, 100,
    ST, 250,
    0xff};

static const uint8_t m_qfl[] = {
    /*XI_, 30, DO, 30, RE, 30, MI, 90, SO, 30, MI, 30, RE, 30, MI, 30, DO, 60, XI_, 30, DO, 30, SO_, 60,
    XI_, 30, DO, 30, RE, 30, MI, 90, SO, 30, MI, 30, RE, 30, MI, 30, DO, 30, RE, 30, XI_, 30, DO, 30, LA_, 60,
    XI_, 30, DO, 30, RE, 30, MI, 90, SO, 30, MI, 30, RE, 120, XI_, 60, SO_, 60,
    LA_, 250, ST, 110, */
    RE, 90, DO, 30, RE, 90, DO, 30, RE, 60, MI, 60, SO, 60, MI, 60,
    RE, 90, DO, 30, RE, 90, DO, 30, RE, 30, MI, 30, RE, 30, DO, 30, LA_, 120,
    RE, 90, DO, 30, RE, 90, DO, 30, RE, 60, MI, 60, SO, 60, MI, 60,
    RE, 90, DO, 30, RE, 60, DO, 60, RE, 240,
    RE, 90, DO, 30, RE, 90, DO, 30, RE, 60, MI, 60, SO, 60, MI, 60,
    RE, 90, DO, 30, RE, 60, DO, 60, LA_, 120, MI, 30, RE, 30, DO, 30, RE, 30,
    DO, 120, MI, 30, RE, 30, DO, 30, RE, 30, DO, 90, SO_, 30, MI, 30, RE, 30, DO, 30, RE, 30,
    DO, 240, DO, 60, RE, 60, MI, 60, DO, 60,
    LA, 60, SO, 30, LA, 120, DO, 30, XI, 60, LA, 30, XI, 150,
    XI, 60, LA, 30, XI, 30, ST, 60, MI, 60, _DO, 30, _RE, 30, _DO, 30, XI, 30, LA, 60, SO, 60,
    LA, 60, SO, 30, LA, 30, ST, 30, SO, 30, LA, 30, SO, 30, LA, 60, SO, 30, RE, 30, ST, 60, SO, 60,
    MI, 120, ST, 120, DO, 60, RE, 60, MI, 60, DO, 60,
    LA, 60, SO, 30, LA, 30, ST, 90, DO, 30, XI, 60, LA, 30, XI, 30, ST, 120,
    XI, 60, LA, 30, XI, 30, ST, 60, MI, 60, _DO, 30, _RE, 30, _DO, 30, XI, 30, LA, 60, SO, 60,
    LA, 60, _MI, 30, _MI, 30, ST, 60, SO, 60, LA, 60, _MI, 30, _MI, 30, ST, 60, SO, 60,
    LA, 120, ST, 240, _DO, 60, _RE, 60,
    _MI, 60, _LA, 30, _SO, 30, ST, 60, _LA, 30, _SO, 30, ST, 60, _LA, 30, _SO, 30, ST, 60, _RE, 30, _MI, 30,
    ST, 60, _LA, 30, _SO, 30, ST, 60, _LA, 30, _SO, 30, ST, 60, _LA, 30, _SO, 30, ST, 30, _MI, 90,
    _RE, 60, _DO, 30, LA, 30, ST, 60, _DO, 30, _DO, 30, _RE, 30, _DO, 30, LA, 60, ST, 30, _DO, 90,
    _MI, 120, ST, 30, _FA, 30, _MI, 30, _RE, 30, _MI, 30, _RE, 90, _DO, 60, _RE, 60,
    _MI, 60, _LA, 30, _SO, 30, ST, 60, _LA, 30, _SO, 30, ST, 60, _LA, 30, _SO, 30, ST, 60, _RE, 60,
    _MI, 60, _LA, 30, _SO, 30, ST, 60, _LA, 30, _SO, 30, ST, 60, _LA, 30, _SO, 30, ST, 30, _MI, 90,
    _RE, 60, _DO, 30, _LA, 30, ST, 30, _MI, 90, _RE, 60, _DO, 30, _LA, 30, ST, 30, _DO, 90,
    _DO, 120, ST, 240, _LA, 30, _MI, 90,
    _RE, 60, _DO, 30, _LA, 30, ST, 30, _MI, 90, _RE, 60, _DO, 30, _LA, 30, ST, 30, _DO, 90,
    _DO, 240, ST, 240,

    ST, 250,
    0xff};

static const uint8_t m_ryssdnr[] = {
    SO_, 80, LA_, 80,
    DO, 80, DO, 80, DO, 80, RE, 40, MI, 200, ST, 80, MI, 40, SO, 40,
    LA, 80, LA, 80, XI, 50, LA, 50, SO, 50, MI, 160, ST, 80, MI, 80,
    RE, 160, RE, 40, DO, 40, LA_, 80, SO_, 80, LA_, 80, ST, 80, SO_, 80,
    DO, 160, DO, 80, DO, 80, RE, 160, ST, 80, SO_, 40, LA_, 40,
    DO, 80, DO, 80, DO, 80, RE, 40, MI, 200, ST, 80, MI, 40, SO, 40,
    LA, 80, LA, 80, XI, 50, LA, 50, SO, 50, MI, 160, ST, 80, MI, 80,
    RE, 160, RE, 40, DO, 40, LA_, 40, SO_, 40, LA_, 240, DO, 80,
    XI_, 160, LA_, 80, SO_, 80, LA_, 160, DO, 80, RE, 80,
    MI, 80, SO, 80, MI, 40, RE, 40, DO, 40, RE, 40, MI, 80, LA, 80, MI, 40, RE, 40, DO, 40, RE, 40,
    MI, 80, XI, 80, XI, 80, MI, 40, SO, 40, LA, 160, LA, 80, SO, 80,
    LA, 80, SO, 80, LA, 40, SO, 40, MI, 40, RE, 40, MI, 80, SO, 80, MI, 80, MI, 80,
    RE, 160, DO, 80, RE, 80, MI, 160, DO, 80, RE, 80,
    MI, 80, SO, 80, MI, 40, RE, 40, DO, 40, RE, 40, MI, 80, LA, 80, MI, 40, RE, 40, DO, 40, RE, 40,
    MI, 80, XI, 80, XI, 80, MI, 40, SO, 40, LA, 160, LA, 80, SO, 40, LA, 40,
    ST, 240, SO, 80, LA, 80, _DO, 160, LA, 80,
    MI, 80, SO, 240, ST, 240, SO_, 40, LA_, 40,
    DO, 80, DO, 80, DO, 80, RE, 40, MI, 200, ST, 80, MI, 40, SO, 40,
    LA, 80, LA, 80, XI, 50, LA, 50, SO, 50, MI, 160, ST, 80, MI, 80,
    RE, 160, RE, 40, DO, 40, LA_, 40, SO_, 40, LA_, 240, DO, 80,
    XI_, 160, LA_, 80, SO_, 40, LA_, 200, ST, 160,

    SO_, 80, LA_, 80,
    DO, 80, DO, 80, DO, 80, RE, 40, MI, 200, ST, 80, MI, 40, SO, 40,
    LA, 80, LA, 80, XI, 50, LA, 50, SO, 50, MI, 160, ST, 80, RE, 40, MI, 40,
    RE, 160, RE, 40, DO, 40, LA_, 80, SO_, 80, LA_, 80, ST, 80, SO_, 80,
    DO, 160, DO, 80, DO, 80, RE, 160, ST, 80, SO_, 40, LA_, 40,
    DO, 80, DO, 80, DO, 80, RE, 40, MI, 200, ST, 80, MI, 40, SO, 40,
    LA, 80, LA, 80, XI, 50, LA, 50, SO, 50, MI, 160, ST, 80, MI, 80,
    RE, 160, RE, 40, DO, 40, LA_, 40, SO_, 40, LA_, 240, DO, 80,
    XI_, 160, LA_, 80, SO_, 80, LA_, 160, DO, 80, RE, 80,
    MI, 80, SO, 80, MI, 40, RE, 40, DO, 40, RE, 40, MI, 80, LA, 80, MI, 40, RE, 40, DO, 40, RE, 40,
    MI, 80, XI, 80, XI, 80, MI, 40, SO, 40, LA, 160, LA, 80, SO, 80,
    LA, 80, SO, 80, LA, 40, SO, 40, MI, 40, RE, 40, MI, 80, SO, 80, MI, 80, MI, 80,
    RE, 160, DO, 80, RE, 80, MI, 160, DO, 80, RE, 80,
    MI, 80, SO, 80, MI, 40, RE, 40, DO, 40, RE, 40, MI, 80, LA, 80, MI, 40, RE, 40, DO, 40, RE, 40,
    MI, 80, XI, 80, XI, 80, MI, 40, SO, 40, LA, 160, LA, 80, SO, 80,
    LA, 80, LA, 160, _DO, 80, _DO, 80, _DO, 80, LA, 80, MI, 80,
    SO, 160, ST, 160, ST, 240, SO_, 40, LA_, 40,

    DO, 80, DO, 80, DO, 80, RE, 40, MI, 200, ST, 80, MI, 40, SO, 40,
    LA, 80, LA, 80, XI, 50, LA, 50, SO, 50, MI, 160, ST, 80, MI, 80,
    RE, 160, RE, 40, DO, 40, LA_, 40, SO_, 40, LA_, 240, DO, 80,
    XI_, 160, LA_, 80, SO_, 80, LA_, 160, DO, 80, RE, 80,
    MI, 80, SO, 80, MI, 40, RE, 40, DO, 40, RE, 40, MI, 80, LA, 80, MI, 40, RE, 40, DO, 40, RE, 40,
    MI, 80, XI, 80, XI, 80, MI, 40, SO, 40, LA, 160, LA, 80, SO, 80,
    LA, 80, SO, 80, LA, 40, SO, 40, MI, 40, RE, 40, MI, 80, SO, 80, MI, 80, MI, 80,
    RE, 160, DO, 80, RE, 80, MI, 160, DO, 80, RE, 80,
    MI, 80, SO, 80, MI, 40, RE, 40, DO, 40, RE, 40, MI, 80, LA, 80, MI, 40, RE, 40, DO, 40, RE, 40,
    MI, 80, XI, 80, XI, 80, MI, 40, SO, 40, LA, 160, LA, 80, SO, 40, LA, 40,
    ST, 240, SO, 80, LA, 80, _DO, 160, LA, 80,
    MI, 80, SO, 240, ST, 240, SO_, 40, LA_, 40,

    DO, 80, DO, 80, DO, 80, RE, 40, MI, 200, ST, 80, MI, 40, SO, 40,
    LA, 80, LA, 80, XI, 50, LA, 50, SO, 50, MI, 160, ST, 80, MI, 80,
    RE, 160, RE, 40, DO, 40, LA_, 40, SO_, 40, LA_, 240, DO, 80,
    XI_, 160, LA_, 80, SO_, 40, LA_, 200, ST, 160,

    ST, 250, 0xff};

static const uint8_t m_bad_apple[] = {
    DO, 20, RE, 20, MI, 20, FA, 20, SO, 20, LA, 20, XI, 40,
    XI, 20, LA, 20, SO, 20, FA, 20, MI, 20, RE, 20, DO, 60,
    0xFF};

const Music_PWM_t pwm_musics[] = {
    {(uint8_t *)m_two_tigers, "TWO_TIGGERS"},
    {(uint8_t *)m_qfl, "QFL"},
    {(uint8_t *)m_ryssdnr, "RYSSDNR"},
    {(uint8_t *)m_bad_apple, "BAD_APPLE"},

    {NULL, NULL}  // 结束标致
};

#endif