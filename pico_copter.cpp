// comment
#include "pico_copter.hpp"

//グローバル変数
uint8_t Arm_flag=0;
semaphore_t sem;

int main(void)
{
  int start_wait=5;
  
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  
  //Initialize stdio for Pico
  stdio_init_all();
  
  //Initialize LSM9DS1
  imu_mag_init();
  
  //Initialize Radio
  radio_init();
  
  //Initialize Variavle
  variable_init();
  
  //Initilize Control
  control_init();
  
  //Initialize PWM
  //Start 400Hz Interval
  ESC_calib=0;
  pwm_init();

  while(start_wait)
  {
    start_wait--;
    printf("#Please wait %d[s]\r",start_wait); 
    sleep_ms(1000);
  }
  printf("\n");
 
  //マルチコア関連の設定
  sem_init(&sem, 0, 1);
  multicore_launch_core1(angle_control);  

  Arm_flag=1;
  
  while(1)
  {
    //printf("Arm_flag:%d LockMode:%d\n",Arm_flag, LockMode);
    tight_loop_contents();
    while (Logoutputflag==1){
      log_output();
    }
  }

  return 0;
}
