#ifndef EKF_H
#define EKF_H

#include <stdio.h>
#include <iostream>
#include <random>
#include "pico/stdlib.h"
#include <Eigen/Dense>
#include <unistd.h>
#include <math.h>

#define GRAV (9.80665)
#define MN (-0.04333)
#define MD  (0.99906)
#define PI (M_PI)

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Matrix;
using Eigen::PartialPivLU;
using namespace Eigen;

#if 0
//Runge-Kutta Method 
uint8_t rk4(uint8_t (*func)(float t, 
                            Matrix<float, 7, 1> x, 
                            Matrix<float, 3, 1> omega, 
                            Matrix<float, 3, 1> beta, 
                            Matrix<float, 7, 1> &k),
            float t, 
            float h, 
            Matrix<float, 7 ,1> &x, 
            Matrix<float, 3, 1> omega,
            Matrix<float, 3, 1> beta);

//Continuous Time State Equation for simulation
uint8_t xdot( float t, 
              Matrix<float, 7, 1> x, 
              Matrix<float, 3, 1> omega , 
              Matrix<float, 3, 1> beta, 
              Matrix<float, 7, 1> &k);


//Discrite Time State Equation
uint8_t state_equation( Matrix<float, 7, 1> &xe, 
                        Matrix<float, 3, 1> omega_m, 
                        Matrix<float, 3, 1> beta, 
                        float dt,
                        Matrix<float, 7, 1> &xp);


//Observation Equation
uint8_t observation_equation(Matrix<float, 7, 1>x, Matrix<float, 6, 1>&z, float g, float mn, float md);


//Make Jacobian matrix F
uint8_t F_jacobian( Matrix<float, 7, 7>&F, 
                    Matrix<float, 7, 1> x, 
                    Matrix<float, 3, 1> omega, 
                    Matrix<float, 3, 1> beta, 
                    float dt);


//Make Jacobian matrix H
uint8_t H_jacobian( Matrix<float, 6, 7> &H, 
                    Matrix<float, 7, 1> x, 
                    float g, 
                    float mn, 
                    float md);

#endif
//Extended Kalman Filter
uint8_t ekf( Matrix<float, 7, 1> &xe,
             Matrix<float, 7, 1> &xp,
             Matrix<float, 7, 7> &P,
             Matrix<float, 6, 1> z,
             Matrix<float, 3, 1> omega,
             Matrix<float, 3, 3> Q, 
             Matrix<float, 6, 6> R, 
             Matrix<float, 7, 3> G,
             Matrix<float, 3, 1> beta,
             float dt);

#endif
