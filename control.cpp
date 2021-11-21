#include "control.hpp"

float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0;
float Pbias=0.0,Qbias=0.0,Rbias=0.0;
float Phi_bias=0.0,Theta_bias=0.0,Psi_bias=0.0;  
float Phi,Theta,Psi;
float Phi_ref=0.0,Theta_ref=0.0,Psi_ref=0.0;
float Elevator_center=0.0, Aileron_center=0.0, Rudder_center=0.0;
float Pref=0.0,Qref=0.0,Rref=0.0;
float Dt=0.0025;
float  Elapsed_time=0.0;
uint32_t S_time=0,E_time=0,D_time=0,S_time2=0,E_time2=0,D_time2=0;
Matrix<float, 7 ,1> Xp = MatrixXf::Zero(7,1);
Matrix<float, 7 ,1> Xe = MatrixXf::Zero(7,1);
Matrix<float, 6 ,1> Z = MatrixXf::Zero(6,1);
Matrix<float, 3, 1> Omega_m = MatrixXf::Zero(3, 1);
Matrix<float, 3, 1> Oomega;
Matrix<float, 7, 7> P;
Matrix<float, 3, 3> Q;// = MatrixXf::Identity(3, 3)*0.1;
Matrix<float, 6, 6> R;// = MatrixXf::Identity(6, 6)*0.0001;
Matrix<float, 7 ,3> G;
Matrix<float, 3 ,1> Beta;
uint8_t AngleControlCounter=0;
uint16_t RateControlCounter=0;
uint16_t BiasCounter=0;
uint16_t LogdataCounter=0;
uint8_t Logflag=0;
volatile uint8_t Logoutputflag=0;
float Log_time=0.0;
uint16_t LedBlinkCounter=0;
const uint8_t DATANUM=10;
const uint32_t LOGDATANUM=48000;
float Logdata[LOGDATANUM]={0.0};
uint8_t LockMode=0;

PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;

void loop_400Hz(void);
void rate_control(void);
void sensor_read(void);
void angle_control(void);
void output_data(void);
void output_sensor_raw_data(void);
void kalman_filter(void);
void logging(void);
uint8_t lock_com(void);
uint8_t logdata_out_com(void);

//Main loop
//This function is called from PWM Intrupt on 400Hz.
void loop_400Hz(void)
{
  static uint8_t led=1;
  S_time=time_us_32();
  
  //割り込みフラグリセット
  pwm_clear_irq(2);


  if (Arm_flag==0)
  {
      Elevator_center = 0.0;
      Aileron_center = 0.0;
      Rudder_center = 0.0;
      Pbias = 0.0;
      Qbias = 0.0;
      Rbias = 0.0;
      Phi_bias = 0.0;
      Theta_bias = 0.0;
      Psi_bias = 0.0;
      return;
  }
  else if (Arm_flag==1)
  {
    //Gyro Bias Estimate
    if (BiasCounter<2000)
    {
      AngleControlCounter++;
      //Sensor Read
      sensor_read();
      if(AngleControlCounter==4)
      {
        AngleControlCounter=0;
        sem_release(&sem);
      }
      Elevator_center += Chdata[1];
      Aileron_center+= Chdata[3];
      Rudder_center += Chdata[0];
      Pbias+=Wp;
      Qbias+=Wq;
      Rbias+=Wr;
      Phi_bias+=Phi;
      Theta_bias+=Theta;
      Psi_bias+=Psi;
      BiasCounter++;
      return;
    }
    else
    {
      Arm_flag = 3;
      Pbias=Pbias/BiasCounter;
      Qbias=Qbias/BiasCounter;
      Rbias=Rbias/BiasCounter;
      Xe(4,0)=Pbias;
      Xe(5,0)=Qbias;
      Xe(6,0)=Rbias;
      Xp(4,0)=Pbias;
      Xp(5,0)=Qbias;
      Xp(6,0)=Rbias;
      Phi_bias=Phi_bias/BiasCounter;
      Theta_bias=Theta_bias/BiasCounter;
      Psi_bias=Psi_bias/BiasCounter;
      Elevator_center = Elevator_center/BiasCounter;
      Aileron_center = Aileron_center/BiasCounter;
      Rudder_center = Rudder_center/BiasCounter;
      return;
    }
  }
  else if( Arm_flag==2)
  {
    if(LockMode==2)
    {
      if(lock_com()==1)
      {
        LockMode=3;//Disenable Flight
        led=0;
        gpio_put(LED_PIN,led);
        return;
      }
      //Goto Flight
    }
    else if(LockMode==3)
    {
      if(lock_com()==0){
        LockMode=0;
        Arm_flag=3;
      }
      return;
    }
    //LED Blink
    gpio_put(LED_PIN, led);
    if(Logflag==1&&LedBlinkCounter<100){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      led=!led;
    }
   
    //Rate Control (400Hz)
    rate_control();
   
    if(AngleControlCounter==4)
    {
      AngleControlCounter=0;
      //Angle Control (100Hz)
      sem_release(&sem);
    }
    AngleControlCounter++;
  }
  else if(Arm_flag==3)
  {
    if(LedBlinkCounter<10){
      gpio_put(LED_PIN, 1);
      LedBlinkCounter++;
    }
    else if(LedBlinkCounter<100)
    {
      gpio_put(LED_PIN, 0);
      LedBlinkCounter++;
    }
    else LedBlinkCounter=0;
    
    if(LockMode==0)
    {
      if( lock_com()==1)
      {
        LockMode=1;
        return;
      }
      //Wait  output log
    }
    else if(LockMode==1)
    {
      if(lock_com()==0)
      {
        LockMode=2;//Enable Flight
        Arm_flag=2;
      }
      return;
    }

    if(logdata_out_com()==1)
    {
      Arm_flag=4;
      return;
    }
  }
  else if(Arm_flag==4)
  {
    Logoutputflag=1;
    //LED Blink
    gpio_put(LED_PIN, led);
    if(LedBlinkCounter<400){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      led=!led;
    }
  }
  E_time=time_us_32();
  D_time=E_time-S_time;
}

void control_init(void)
{
  //Rate control
  p_pid.set_parameter(0.9, 1.0e1, 0.01, 0.01, 0.0025);
  q_pid.set_parameter(0.9, 1.0e1, 0.01, 0.01, 0.0025);
  r_pid.set_parameter(1.0, 1.0e1, 0.001, 0.01, 0.0025);
  //Angle control
  phi_pid.set_parameter  (0.3, 1.0e2, 0.0,  0.01, 0.01);
  theta_pid.set_parameter(0.3, 1.0e2, 0.0,  0.01, 0.01);
  psi_pid.set_parameter  (0.3, 1.0e2, 0.0,  0.01, 0.01);
}

uint8_t lock_com(void)
{
  static uint8_t chatta=0,state=0;
  if( Chdata[2]<CH3MIN+50 
   && Chdata[0]>CH1MAX-50
   && Chdata[3]<CH4MIN+50 
   && Chdata[1]>CH2MAX-50)
  { 
    chatta++;
    if(chatta>50){
      chatta=50;
      state=1;
    }
  }
  else 
  {
    chatta=0;
    state=0;
  }

  return state;

}

uint8_t logdata_out_com(void)
{
  static uint8_t chatta=0,state=0;
  if( Chdata[4]<(CH5MAX+CH5MIN)*0.5 
   && Chdata[2]<CH3MIN+50 
   && Chdata[0]<CH1MIN+50
   && Chdata[3]>CH4MAX-50 
   && Chdata[1]>CH2MAX-50)
  {
    chatta++;
    if(chatta>50){
      chatta=50;
      state=1;
    }
  }
  else 
  {
    chatta=0;
    state=0;
  }

  return state;
}

void rate_control(void)
{
  float p_rate, q_rate, r_rate;
  float p_ref, q_ref, r_ref, t_ref;
  float p_err, q_err, r_err;
  float p_com, q_com, r_com;
  float fr_duty, fl_duty, rr_duty, rl_duty;

  //Read Sensor Value
  sensor_read();

  //Get Bias
  //Pbias = Xe(4, 0);
  //Qbias = Xe(5, 0);
  //Rbias = Xe(6, 0);

  //Control angle velocity
  p_rate = Wp - Pbias;
  q_rate = Wq - Qbias;
  r_rate = Wr - Rbias;

  //Get reference
  p_ref = Pref;
  q_ref = Qref;
  r_ref = Rref;
  t_ref=(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);

  //Error
  p_err = p_ref - p_rate;
  q_err = q_ref - q_rate;
  r_err = r_ref - r_rate;

  //PID
  p_com = p_pid.update(p_err);
  q_com = q_pid.update(q_err);
  r_com = r_pid.update(r_err);

  //Motor Control
  fr_duty = t_ref +(-p_com +q_com -r_com)*0.25;
  fl_duty = t_ref +( p_com +q_com +r_com)*0.25;
  rr_duty = t_ref +(-p_com -q_com +r_com)*0.25;
  rl_duty = t_ref +( p_com -q_com -r_com)*0.25;

  //Duty set
  if(Chdata[2]>CH3MIN*1.07)
  {
    //set_duty_fr(fr_duty);
    //set_duty_fl(fl_duty);
    //set_duty_rr(rr_duty);
    //set_duty_rl(rl_duty);
    set_duty_fr(0.0);
    set_duty_fl(0.0);
    set_duty_rr(0.0);
    set_duty_rl(0.0);
  }
  else
  {
    set_duty_fr(0.0);
    set_duty_fl(0.0);
    set_duty_rr(0.0);
    set_duty_rl(0.0);
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();
    Pref=0.0;
    Qref=0.0;
    Rref=0.0;
  }


  //printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n", 
  //    Elapsed_time, fr_duty, fl_duty, rr_duty, rl_duty, p_rate, q_rate, r_rate);
  //printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n", 
  //    Elapsed_time, p_com, q_com, r_com, p_ref, q_ref, r_ref);
  //printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n", 
  //    Elapsed_time, Phi, Theta, Psi, Phi_bias, Theta_bias, Psi_bias);
  //Elapsed_time = Elapsed_time + 0.0025;
  //Logging
  logging();
}

void angle_control(void)
{
  float phi_err,theta_err,psi_err;
  float q0,q1,q2,q3;
  float e23,e33,e13,e11,e12;
  while(1)
  {
    sem_acquire_blocking(&sem);
    sem_reset(&sem, 0);
    S_time2=time_us_32();
    kalman_filter();
    q0 = Xe(0,0);
    q1 = Xe(1,0);
    q2 = Xe(2,0);
    q3 = Xe(3,0);
    e11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    e12 = 2*(q1*q2 + q0*q3);
    e13 = 2*(q1*q3 - q0*q2);
    e23 = 2*(q2*q3 + q0*q1);
    e33 = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    Phi = atan2(e23, e33);
    Theta = atan2(-e13, sqrt(e23*e23+e33*e33));
    Psi = atan2(e12,e11);

    //Get angle ref 
    Phi_ref=0.5*M_PI*(float)(Chdata[3] - Aileron_center )*2/(CH3MAX-CH3MIN);
    Theta_ref=0.5*M_PI*(float)(Chdata[1] - Elevator_center)*2/(CH2MAX-CH2MIN);
    Psi_ref=0.5*M_PI*(float)(Chdata[0] - Rudder_center  )*2/(CH1MAX-CH1MIN);

    //Error
    phi_err = Phi_ref - (Phi-Phi_bias);
    theta_err = Theta_ref - (Theta-Theta_bias);
    psi_err = Psi_ref - (Psi-Psi_bias);
    
    //PID Control
    if(Chdata[2]>CH3MIN*1.07)
    {
      Pref = phi_pid.update(phi_err);
      Qref = theta_pid.update(theta_err);
      Rref = psi_pid.update(psi_err);
    }
    else
    {
      Pref=0.0;
      Qref=0.0;
      Rref=0.0;
      phi_pid.reset();
      theta_pid.reset();
      psi_pid.reset();
    }

    E_time2=time_us_32();
    D_time2=E_time2-S_time2;
    if(Arm_flag==2)
    {
      Elapsed_time=Elapsed_time+0.01;
      output_data();
    }
    else
    {
      Elapsed_time=0.0;
    }


    //Pref=0.0;
    //Qref=0.0;
    //Rref=0.0;
  }
}

void logging(void)
{  
  //Logging
  if(Chdata[4]>(CH5MAX+CH5MIN)*0.5)
  { 
    if(Logflag==0)
    {
      Logflag=1;
      LogdataCounter=0;
    }
    if(LogdataCounter<LOGDATANUM)
    {
      Logdata[LogdataCounter++]=Xe(0,0);
      Logdata[LogdataCounter++]=Xe(1,0);
      Logdata[LogdataCounter++]=Xe(2,0);
      Logdata[LogdataCounter++]=Xe(3,0);
      Logdata[LogdataCounter++]=Wp;
      Logdata[LogdataCounter++]=Wq;
      Logdata[LogdataCounter++]=Wr;
      Logdata[LogdataCounter++]=Ax;
      Logdata[LogdataCounter++]=Ay;
      Logdata[LogdataCounter++]=Az;
      /*
      Logdata[LogdataCounter++]=Pref;
      Logdata[LogdataCounter++]=Qref;
      Logdata[LogdataCounter++]=Rref;
      Logdata[LogdataCounter++]=Phi-Phi_bias;
      Logdata[LogdataCounter++]=Theta-Theta_bias;
      Logdata[LogdataCounter++]=Psi-Psi_bias;
      Logdata[LogdataCounter++]=Phi_ref;
      Logdata[LogdataCounter++]=Theta_ref;
      Logdata[LogdataCounter++]=Psi_ref;
      */
    }
    else Logflag=2;
  }
  else
  { 
    if(Logflag>0)
    {
      Logflag=0;
      LogdataCounter=0;
    }
  }
}

void log_output(void)
{
  if(LogdataCounter<LOGDATANUM)
  {
    //LockMode=0;
    printf("%10.5f ", Log_time);
    Log_time=Log_time + 0.0025;
    for (uint8_t i=0;i<DATANUM;i++)
    {
      printf("%10.5f",Logdata[LogdataCounter+i]);
    }
    printf("\n");
    LogdataCounter=LogdataCounter + DATANUM;
  }
  else 
  {
    Arm_flag=3;
    Logoutputflag=0;
    LockMode=0;
    Log_time=0.0;
    LogdataCounter=0;
  }
  
  //output_sensor_raw_data();
  //Elapsed_time=Elapsed_time+Dt;
}


void gyroCalibration(void)
{
  float wp,wq,wr;
  float sump,sumq,sumr;
  uint16_t N=400;
  for(uint16_t i=0;i<N;i++)
  {
    sensor_read();
    sump=sump+Wp;
    sumq=sumq+Wq;
    sumr=sumr+Wr;
  }
  Pbias=sump/N;
  Qbias=sumq/N;
  Rbias=sumr/N;
}

void sensor_read(void)
{
  float mx1,my1,mz1;

  imu_mag_data_read();
  Ax =-acceleration_mg[0]*GRAV*0.001;
  Ay =-acceleration_mg[1]*GRAV*0.001;
  Az = acceleration_mg[2]*GRAV*0.001;
  Wp = angular_rate_mdps[0]*M_PI*5.55555555e-6;//5.5.....e-6=1/180/1000
  Wq = angular_rate_mdps[1]*M_PI*5.55555555e-6;
  Wr =-angular_rate_mdps[2]*M_PI*5.55555555e-6;
  Mx0 =-magnetic_field_mgauss[0];
  My0 = magnetic_field_mgauss[1];
  Mz0 =-magnetic_field_mgauss[2];

/*地磁気校正データ
  地磁気の回転行列
  [[ 0.7476018   0.49847825  0.43887467]
   [-0.58069715  0.81129925  0.06770773]
   [-0.32230786 -0.3054717   0.89599369]]
  中心座標
  -112.81022304592778 65.47692484225598 -174.3009521339447
  拡大係数
  0.0031687151914840095 0.0035509799555226906 0.003019055826773856
  mx1 = 0.0031687151914840095*( 0.74760180*Mx0 +0.49847825*My0 +0.43887467*Mz0 +112.81022304592778);
  my1 = 0.0035509799555226906*(-0.58069715*Mx0 +0.81129925*My0 +0.06770773*Mz0 - 65.47692484225598 );
  mz1 = 0.003019055826773856*(-0.32230786*Mx0 -0.30547170*My0 +0.89599369*Mz0 +174.3009521339447);
  Mx = 0.74760180*mx1 -0.58069715*my1 -0.32230786*mz1;
  My = 0.49847825*mx1 +0.81129925*my1 -0.30547170*mz1;
  Mz = 0.43887467*mx1 +0.06770773*my1 +0.89599369*mz1; 

回転行列
[[ 0.65330968  0.75327755 -0.07589064]
 [-0.75666134  0.65302622 -0.03194321]
 [ 0.02549647  0.07829232  0.99660436]]
中心座標
122.37559195017053 149.0184454603531 -138.99116060635413
W
-2.432054387460946
拡大係数
0.003077277151877191 0.0031893151610213463 0.0033832794976645804
*/
//回転行列
const float rot[9]={0.65330968, 0.75327755, -0.07589064,
             -0.75666134, 0.65302622, -0.03194321,
              0.02549647,  0.07829232,  0.99660436};
//中心座標
const float center[3]={122.37559195017053, 149.0184454603531, -138.99116060635413};
//拡大係数
const float zoom[3]={0.003077277151877191, 0.0031893151610213463, 0.0033832794976645804};

//回転・平行移動・拡大
  mx1 = zoom[0]*( rot[0]*Mx0 +rot[1]*My0 +rot[2]*Mz0 -center[0]);
  my1 = zoom[1]*( rot[3]*Mx0 +rot[4]*My0 +rot[5]*Mz0 -center[1]);
  mz1 = zoom[2]*( rot[6]*Mx0 +rot[7]*My0 +rot[8]*Mz0 -center[2]);
//逆回転
  Mx = rot[0]*mx1 +rot[3]*my1 +rot[6]*mz1;
  My = rot[1]*mx1 +rot[4]*my1 +rot[7]*mz1;
  Mz = rot[2]*mx1 +rot[5]*my1 +rot[8]*mz1; 
//  mag_norm=sqrt(mx*mx +my*my +mz*mz);
}

void variable_init(void)
{
  //Variable Initalize
  Xe << 1.00, 0.0, 0.0, 0.0,0.0,0.0, 0.0;
  Xp =Xe;
/*
  Q <<  8.86e-6  , 0.0    , 0.0,
        0.0    , 9.9e-6   , 0.0,
        0.0,     0.0    ,   5.38e-6;
*/
  Q <<  1.0e-1 , 0.0    , 0.0,
        0.0   , 1.0e-1  , 0.0,
        0.0   , 0.0    , 1.0e-10;
/*
  R <<  8.17e-6, 0.0    , 0.0    , 0.0, 0.0, 0.0,
        0.0    , 6.25e-6, 0.0    , 0.0, 0.0, 0.0,
        0.0    , 0.0    , 1.19e-5, 0.0, 0.0, 0.0,
        0.0    , 0.0    , 0.0    , 2.03e-4, 0.0    , 0.0,
        0.0    , 0.0    , 0.0    , 0.0    , 2.08e-4, 0.0,
        0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 3.29e-4;
*/

  R <<  10.0e-5   , 0.0     , 0.0    , 0.0   , 0.0   , 0.0,
        0.0    , 7.5e-5    , 0.0    , 0.0   , 0.0   , 0.0,
        0.0    , 0.0    , 11.7e-5    , 0.0   , 0.0   , 0.0,
        0.0    , 0.0    , 0.0    , 18.4e-3, 0.0   , 0.0,
        0.0    , 0.0    , 0.0    , 0.0   , 21.5e-3, 0.0,
        0.0    , 0.0    , 0.0    , 0.0   , 0.0   , 45.5e-3;
          
  G <<  0.0,0.0,0.0, 
        0.0,0.0,0.0, 
        0.0,0.0,0.0, 
        0.0,0.0,0.0, 
        1.0,0.0,0.0, 
        0.0,1.0,0.0, 
        0.0,0.0,1.0;
  Beta << 0.0, 0.0, 0.0;
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

void output_data(void)
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
            ,Elapsed_time//1
            ,Xe(0,0), Xe(1,0), Xe(2,0), Xe(3,0)//2~5 
            ,Xe(4,0), Xe(5,0), Xe(6,0)//6~8
            //,Phi-Phi_bias, Theta-Theta_bias, Psi-Psi_bias//6~8
            ,D_time, D_time2//10,11
            ,Ax, Ay, Az//11~13
            ,Wp, Wq, Wr//14~16
            ,Mx, My, Mz//17~19
            //,mag_norm
        ); //20
}
void output_sensor_raw_data(void)
{
  printf("%9.3f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f"
         "\n"
            ,Elapsed_time//1
            ,Ax, Ay, Az//2~4
            ,Wp, Wq, Wr//5~7
            ,Mx0, My0, Mz0//8~10
        ); //20
}

void kalman_filter(void)
{
  //Kalman Filter
  float dt=0.01;
  Omega_m << Wp, Wq, Wr;
  Z << Ax, Ay, Az, Mx, My, Mz;
  ekf(Xp, Xe, P, Z, Omega_m, Q, R, G*dt, Beta, dt);
}


PID::PID()
{
  m_kp=1.0e-8;
  m_ti=1.0e8;
  m_td=0.0;
  m_integral=0.0;
  m_filter_time_constant=0.01;
  m_filter_output=0.0;
  m_err=0.0;
  m_h=0.01;
}

void PID::set_parameter(
    float kp, 
    float ti, 
    float td,
    float filter_time_constant, 
    float h)
{
  m_kp=kp;
  m_ti=ti;
  m_td=td;
  m_filter_time_constant=filter_time_constant;
  m_h=h;
}

void PID::reset(void)
{
  m_integral=0.0;
  m_filter_output=0.0;
  m_err=0.0;
}

float PID::filter(float x)
{
  return x;
}

float PID::update(float err)
{
  float d;
  m_integral = m_integral + err;
  if(m_integral> 100.0)m_integral = 100.0;
  if(m_integral<-100.0)m_integral =-100.0;
  m_filter_output = filter((err-m_err)/m_h);
  m_err = err;
  return m_kp*(err + m_h * m_integral/m_ti + m_td * m_filter_output); 
}
