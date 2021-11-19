#ifndef PICO_COPTER_HPP
#define PICO_COPTER_HPP

#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"
#include "sensor.hpp"
#include "ekf.hpp"
#include "pwm.hpp"
#include "radio.hpp"
#include "control.hpp"
#include <math.h>

#define LED_PIN 25
#define MAINLOOP loop_400Hz

//グローバル変数
extern uint8_t Arm_flag;
extern semaphore_t sem;

#endif

