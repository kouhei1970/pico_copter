#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "sensor.hpp"
#include "ekf.hpp"
#include <math.h>

double elapsed_time=0.0;
float dt=0.01;
float ax,ay,az;
float wp,wq,wr;
float mx,my,mz,mx0,my0,mz0,mx1,my1,mz1,q0b,q1b,q2b,q3b;
float mag_norm;
uint32_t s_time=0,e_time=0,s_time2=0,e_time2=0,d_time,d_time2=0;

Matrix<float, 7 ,1> xp = MatrixXf::Zero(7,1);
Matrix<float, 7 ,1> xe = MatrixXf::Zero(7,1);
Matrix<float, 6 ,1> z = MatrixXf::Zero(6,1);
Matrix<float, 3, 1> omega_m = MatrixXf::Zero(3, 1);
Matrix<float, 3, 1> domega;
Matrix<float, 7, 7> P;
Matrix<float, 3, 3> Q;// = MatrixXf::Identity(3, 3)*0.1;
Matrix<float, 6, 6> R;// = MatrixXf::Identity(6, 6)*0.0001;
Matrix<float, 7 ,3> G;
Matrix<float, 3 ,1> beta;

void variableInit(void)
{
  //Variable Initalize
  xe << 1.00, 0.0, 0.0, 0.0,-6.1e-4,-6.1e-4, 6.1e-4;
  xp =xe;
/*
  Q <<  8.86e-6  , 0.0    , 0.0,
        0.0    , 9.9e-6   , 0.0,
        0.0,     0.0    ,   5.38e-6;
*/
  Q <<  1.0e0 , 0.0    , 0.0,
        0.0   , 1.0e0  , 0.0,
        0.0   , 0.0    , 1.0e0;
/*
  R <<  8.17e-6, 0.0    , 0.0    , 0.0, 0.0, 0.0,
        0.0    , 6.25e-6, 0.0    , 0.0, 0.0, 0.0,
        0.0    , 0.0    , 1.19e-5, 0.0, 0.0, 0.0,
        0.0    , 0.0    , 0.0    , 2.03e-4, 0.0    , 0.0,
        0.0    , 0.0    , 0.0    , 0.0    , 2.08e-4, 0.0,
        0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 3.29e-4;
*/

  R <<  1e-4   , 0.0    , 0.0    , 0.0   , 0.0   , 0.0,
        0.0    , 1e-4   , 0.0    , 0.0   , 0.0   , 0.0,
        0.0    , 0.0    , 1e-4   , 0.0   , 0.0   , 0.0,
        0.0    , 0.0    , 0.0    , 1.0e-4, 0.0   , 0.0,
        0.0    , 0.0    , 0.0    , 0.0   , 1.0e-4, 0.0,
        0.0    , 0.0    , 0.0    , 0.0   , 0.0   , 1.0e-4;
          
  G <<  0.0,0.0,0.0, 
        0.0,0.0,0.0, 
        0.0,0.0,0.0, 
        0.0,0.0,0.0, 
        1.0,0.0,0.0, 
        0.0,1.0,0.0, 
        0.0,0.0,1.0;
  beta << 0.0, 0.0, 0.0;
  P <<  1e0,0,0,0,0,0,0,  
        0,1e0,0,0,0,0,0,
        0,0,1e0,0,0,0,0,  
        0,0,0,1e0,0,0,0, 
        0,0,0,0,1e0,0,0,  
        0,0,0,0,0,1e0,0,  
        0,0,0,0,0,0,1e0;
}

void printPQR(void)
{
  volatile int m=0;
  volatile int n=0;
  //Print P
  printf("#P\n");
  for (m=0;m<7;m++)
  {
    printf("# ");
    for (n=0;n<7;n++)
    {
      printf("%12.4e ",P(m,n));
    }
    printf("\n");
  }
  //Print Q
  printf("#Q\n");
  for (m=0;m<3;m++)
  {
    printf("# ");
    for (n=0;n<3;n++)
    {
      printf("%12.4e ",Q(m,n));
    }
    printf("\n");
  }
  //Print R
  printf("#R\n");
  for (m=0;m<6;m++)
  {
    printf("# ");
    for (n=0;n<6;n++)
    {
      printf("%12.4e ",R(m,n));
    }
    printf("\n");
  }
}

void outputData(void)
{
  printf("%9.3f,"
         "%13.8f,%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%6lu,%6lu,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f"
         //"%13.8f"
         "\n"
            ,elapsed_time//1
            ,xe(0,0), xe(1,0), xe(2,0), xe(3,0)//2~5 
            ,xe(4,0), xe(5,0),xe(6,0)//6~8
            ,d_time, d_time2//10,11
            ,ax, ay, az//11~13
            ,wp, wq, wr//14~16
            ,mx, my, mz//17~19
            //,mag_norm
        ); //20
}

bool callback(repeating_timer_t *rt)
{
  static int flag=0;

  
  s_time=time_us_32();//Measure function execution time
  
  //Output Data
  s_time2=time_us_32();//Measure Output time  
  outputData();
  e_time2=time_us_32();//Measure Output time
  d_time2=e_time2-s_time2;  
  
  //Raad 9DOF sensor
  imu_mag_data_read();
  ax =-acceleration_mg[0]*GRAV*0.001;
  ay =-acceleration_mg[1]*GRAV*0.001;
  az = acceleration_mg[2]*GRAV*0.001;
  wp = angular_rate_mdps[0]*M_PI*5.55555555e-6;//5.5.....e-6=1/180/1000
  wq = angular_rate_mdps[1]*M_PI*5.55555555e-6;
  wr =-angular_rate_mdps[2]*M_PI*5.55555555e-6;
  mx0 =-magnetic_field_mgauss[0];
  my0 = magnetic_field_mgauss[1];
  mz0 =-magnetic_field_mgauss[2];

/*地磁気校正データ
  地磁気の回転行列
  [[ 0.7476018   0.49847825  0.43887467]
   [-0.58069715  0.81129925  0.06770773]
   [-0.32230786 -0.3054717   0.89599369]]
  中心座標
  -112.81022304592778 65.47692484225598 -174.3009521339447
  拡大係数
  0.0031687151914840095 0.0035509799555226906 0.003019055826773856
*/

  mx1 = 0.0031687151914840095*( 0.74760180*mx0 +0.49847825*my0 +0.43887467*mz0 +112.81022304592778);
  my1 = 0.0035509799555226906*(-0.58069715*mx0 +0.81129925*my0 +0.06770773*mz0 - 65.47692484225598 );
  mz1 = 0.003019055826773856*(-0.32230786*mx0 -0.30547170*my0 +0.89599369*mz0 +174.3009521339447);
  mx = 0.74760180*mx1 -0.58069715*my1 -0.32230786*mz1;
  my = 0.49847825*mx1 +0.81129925*my1 -0.30547170*mz1;
  mz = 0.43887467*mx1 +0.06770773*my1 +0.89599369*mz1; 
//  mag_norm=sqrt(mx*mx +my*my +mz*mz);

  
  //Kalman Filter
  omega_m << wp, wq, wr;
  z << ax, ay, az, mx, my, mz;
  ekf(xp, xe, P, z, omega_m, Q, R, G*dt, beta, dt);

  elapsed_time = elapsed_time + 0.01;
  
  e_time=time_us_32();//Measure function execution time
  d_time=e_time-s_time;
  return ( true );
}

int main(void)
{
  static repeating_timer_t timer;
  int start_wait=10;
  //Initialize stdio for Pico
  stdio_init_all();
  //Initialize LSM9DS1
  imu_mag_init();
  //Initialize Variavle
  variableInit();

  while(start_wait)
  {
    start_wait--;
    printf("#Please wait %d[s]\r",start_wait); 
    sleep_ms(1000);
  }
  printf("\n");
  printPQR();
 
  /* インターバルタイマ設定 */
  add_repeating_timer_us( -10000, &callback, NULL, &timer );
  
  while(1);

  return 0;
}
