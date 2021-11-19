#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <stdio.h>
#include "pico_copter.hpp"
#include "pico/stdlib.h"
#include "sensor.hpp"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include <Eigen/Dense>
#include "ekf.hpp"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Matrix;
using Eigen::PartialPivLU;
using namespace Eigen;


//グローバル関数の宣言
void loop_400Hz(void);
void control_init();
void rate_control(void);
void angle_control(void);
void gyro_calibration(void);
void variable_init(void);
void log_output(void);

//グローバル変数
extern uint8_t LockMode;
extern volatile uint8_t Logoutputflag;

class PID
{
  private:
    float m_kp;
    float m_ti;
    float m_td;
    float m_integral;
    float m_filter_time_constant;
    float m_err;
    float m_filter_output;
    float m_h;
  public:
    PID();
    void set_parameter(
        float kp, 
        float ti, 
        float td,
        float filter_time_constant, 
        float h);
    void reset(void);
    float filter(float x);
    float update(float err);
};

#endif
