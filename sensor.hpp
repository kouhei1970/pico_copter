#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "lsm9ds1_reg.h"

//extern int16_t data_raw_acceleration[3];
//extern int16_t data_raw_angular_rate[3];
//extern int16_t data_raw_magnetic_field[3];
extern float acceleration_mg[3];
extern float angular_rate_mdps[3];
extern float magnetic_field_mgauss[3];
//extern lsm9ds1_id_t whoamI;
//extern lsm9ds1_status_t reg;
//extern uint8_t rst;
//extern uint8_t tx_buffer_imu[1000];
//extern uint8_t tx_buffer_mag[1000];
//extern stmdev_ctx_t Imu_h;
//extern stmdev_ctx_t Mag_h;

#define PIN_CSAG  13
#define PIN_MISO  8
#define PIN_CSM   9
#define PIN_SCK   10
#define PIN_MOSI  11

//sensbus_t Ins_bus={spi0, PIN_CSAG};
//sensbus_t Mag_bus={spi0, PIN_CSM};


void imu_mag_init(void);
void imu_mag_data_read(void);


#endif
