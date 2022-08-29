/*
 * pwm.cpp
*/
#include "pwm.hpp"

//グローバル変数
uint8_t ESC_calib=0;

//ファイル内グローバル変数
const uint Slice_num_rear=1;
const uint Slice_num_front=2;
const uint Slice_num_servo = 3;

void pwm_init()
{
    // PWMの設定
    // Tell GPIO 2-6 they are allocated to the PWM for Motor & Servo Control
    gpio_set_function(2, GPIO_FUNC_PWM);//Rear  Left  (RL) Motor PWM
    gpio_set_function(3, GPIO_FUNC_PWM);//Rear  Right (RR) Motor PWM
    gpio_set_function(4, GPIO_FUNC_PWM);//Front Left  (FL) Motor PWM
    gpio_set_function(5, GPIO_FUNC_PWM);//Front Right (FR) Motro PWM
    gpio_set_function(6, GPIO_FUNC_PWM);//Servo PWM



    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    // Set period T
    // T=(wrap+1)*clkdiv/sysclock
    // T=(24999+1)*100/125e6=25000e2/125e6=200e-4=0.02s(=50Hz)
    pwm_set_wrap(Slice_num_front, 3124);
    pwm_set_wrap(Slice_num_rear,  3124);
    pwm_set_wrap(Slice_num_servo, 3124);

    pwm_set_clkdiv(Slice_num_front, 100.0);
    pwm_set_clkdiv(Slice_num_rear, 100.0);
    pwm_set_clkdiv(Slice_num_servo, 100.0);

    pwm_clear_irq(Slice_num_front);
    pwm_set_irq_enabled(Slice_num_front, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, MAINLOOP);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    if(ESC_calib==1)
    {
      // ESC calibration
      // Set Duty
      // DutyA=clkdiv*PWM_CHAN_A/sysclock
      pwm_set_chan_level(Slice_num_front, PWM_CHAN_A, DUTYMAX);
      pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_A, DUTYMAX);
      pwm_set_chan_level(Slice_num_front, PWM_CHAN_B, DUTYMAX);
      pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_B, DUTYMAX);
    } 
    else
    {
      pwm_set_chan_level(Slice_num_front, PWM_CHAN_A, DUTYMIN);
      pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_A, DUTYMIN);
      pwm_set_chan_level(Slice_num_front, PWM_CHAN_B, DUTYMIN);
      pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_B, DUTYMIN);
    }

    // Set the PWM running
    //sleep_ms(500);
    pwm_set_enabled(Slice_num_front, true);
    pwm_set_enabled(Slice_num_rear,  true);
    pwm_set_enabled(Slice_num_servo, true);

    sleep_ms(3000);
    
    pwm_set_chan_level(Slice_num_front, PWM_CHAN_A, DUTYMIN);
    pwm_set_chan_level(Slice_num_front, PWM_CHAN_B, DUTYMIN);
    pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_A, DUTYMIN);
    pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_B, DUTYMIN);
    pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_B, DUTYMIN);
    pwm_set_chan_level(Slice_num_servo, PWM_CHAN_A, 625);

    sleep_ms(1000);
}


void set_duty_fr(float duty)
{
    if (duty>0.97)duty=0.97;  
    if (duty<0.01)duty=0.01;
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    pwm_set_chan_level(Slice_num_front, PWM_CHAN_B, duty);
    //printf("%4.0f ", duty);
}

void set_duty_fl(float duty)
{
    if (duty>0.97)duty=0.97;  
    if (duty<0.01)duty=0.01;
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    pwm_set_chan_level(Slice_num_front, PWM_CHAN_A, duty);
    //printf("%4.0f ", duty);
}

void set_duty_rr(float duty)
{
    if (duty>0.97)duty=0.97;  
    if (duty<0.01)duty=0.01;
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    pwm_set_chan_level(Slice_num_rear, PWM_CHAN_B, duty);
    //printf("%4.0f ", duty);
}

void set_duty_rl(float duty)
{
    if (duty>0.97)duty=0.97;  
    if (duty<0.01)duty=0.01;
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    pwm_set_chan_level(Slice_num_rear, PWM_CHAN_A, duty);
    //printf("%4.0f ", duty);
}
