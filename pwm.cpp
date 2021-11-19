/*
 * pwm.cpp
*/
#include "pwm.hpp"

//グローバル変数
uint8_t ESC_calib=0;

//ファイル内グローバル変数
uint Slice_num_rear=1;
uint Slice_num_front=2;

void pwm_init()
{

    // PWMの設定
    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(2, GPIO_FUNC_PWM);
    gpio_set_function(3, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);
    gpio_set_function(5, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    // Set period T
    // T=(wrap+1)*clkdiv/sysclock
    // T=(24999+1)*100/125e6=25000e2/125e6=200e-4=0.02s(=50Hz)
    pwm_set_wrap(Slice_num_front, 3124);
    pwm_set_wrap(Slice_num_rear,  3124);
    pwm_set_clkdiv(Slice_num_front, 100.0);
    pwm_set_clkdiv(Slice_num_rear, 100.0);
    pwm_clear_irq(Slice_num_front);
    pwm_set_irq_enabled(Slice_num_front, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, MAINLOOP);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    if(ESC_calib)
    {
      // Set channel A Duty
      // DutyA=clkdiv*PWM_CHAN_A/sysclock
      pwm_set_chan_level(Slice_num_front, PWM_CHAN_A, DUTYMAX);
      pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_A, DUTYMAX);
      // Set initial B Duty
      // DutyB=clkdiv*PWM_CHAN_B/sysclock
      pwm_set_chan_level(Slice_num_front, PWM_CHAN_B, DUTYMAX);
      pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_B, DUTYMAX);
    } 
    else
    {
      // Set channel A Duty
      // DutyA=clkdiv*PWM_CHAN_A/sysclock
      pwm_set_chan_level(Slice_num_front, PWM_CHAN_A, DUTYMIN);
      pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_A, DUTYMIN);
      // Set initial B Duty
      // DutyB=clkdiv*PWM_CHAN_B/sysclock
      pwm_set_chan_level(Slice_num_front, PWM_CHAN_B, DUTYMIN);
      pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_B, DUTYMIN);
    }

    // Set the PWM running
    pwm_set_enabled(Slice_num_front, true);
    pwm_set_enabled(Slice_num_rear, true);
    sleep_ms(2000);
    pwm_set_chan_level(Slice_num_front, PWM_CHAN_A, DUTYMIN);
    pwm_set_chan_level(Slice_num_front, PWM_CHAN_B, DUTYMIN);
    pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_A, DUTYMIN);
    pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_B, DUTYMIN);
}


void set_duty_fr(float duty)
{
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    pwm_set_chan_level(Slice_num_front, PWM_CHAN_B, duty);
}

void set_duty_fl(float duty)
{
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    pwm_set_chan_level(Slice_num_front, PWM_CHAN_A, duty);
}

void set_duty_rr(float duty)
{
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    pwm_set_chan_level(Slice_num_rear, PWM_CHAN_B, duty);
}

void set_duty_rl(float duty)
{
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    pwm_set_chan_level(Slice_num_rear, PWM_CHAN_A, duty);
}
