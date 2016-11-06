// The MIT License (MIT)
// Copyright (c) 2015, Guilherme C. Camargo, Diego P. Domingos
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

// Part of the code of this source file has been based on MPU-6050 Short
// Example Sketch By Arduino User JohnChi August 17, 2014 Public Domain

#include<Wire.h>
#include<math.h>

/* Angle Calculation Variables */
int16_t acx,acy,acz,temp,gyx,gyy,gyz;
double gravity_norm;
double accel_x, accel_y, accel_z;
double gyro_x, gyro_y, gyro_z;
double accel_roll, accel_pitch, accel_yaw;
double gyro_pitch_rate, gyro_roll_rate, gyro_yaw_rate;
double kalman_roll, kalman_pitch, kalman_yaw, integral_yaw;
double DT;

static unsigned long start_time;

#define MPU_I2C_ADDR                            0x68 // Default I2C address of the MPU-6050
#define MPU6050_CONFIG_REG_ADDR                 0x1A
#define GYRO_CONFIG_REG_ADDR            0x1B
#define MPU6050_ACCEL_CONFIG_REG_ADDR           0x1C
#define MPU6050_PWR_MGMT_1_REG_ADDR             0x6B
#define MPU6050_GYRO_CONFIG_FS_SEL_0    0x00
#define MPU6050_GYRO_CONFIG_FS_SEL_1    0x08
#define MPU6050_GYRO_CONFIG_FS_SEL_2    0x10
#define MPU6050_GYRO_CONFIG_FS_SEL_3    0x18
#define MPU6050_ACCEL_CONFIG_AFS_SEL_0    0x00
#define MPU6050_ACCEL_CONFIG_AFS_SEL_1    0x08
#define MPU6050_ACCEL_CONFIG_AFS_SEL_2    0x10
#define MPU6050_ACCEL_CONFIG_AFS_SEL_3    0x18
#define MPU6050_DLPF_256HZ                0x00
#define MPU6050_DLPF_188HZ                0x01
#define MPU6050_DLPF_98HZ                 0x02
#define MPU6050_DLPF_42HZ                 0x03
#define MPU6050_DLPF_20HZ                 0x04
#define MPU6050_DLPF_10HZ                 0x05
#define MPU6050_DLPF_5HZ                  0x06


#define MPU6050_ACCEL_CONFIG_SET MPU6050_ACCEL_CONFIG_AFS_SEL_1
#define MPU6050_GYRO_CONFIG_SET MPU6050_GYRO_CONFIG_FS_SEL_1

#define DELAY_TO_PRINT 10000
#define CALIBRATION_TIME 100

#define PRINT_RAW false
#define PRINT_ANG_RATES false
#define PRINT_ANGLES true
#define DEC_PLACES 1

double mpu6050_accel_factor(char SET){
  switch(SET){
    case MPU6050_ACCEL_CONFIG_AFS_SEL_0:
      return 16.384;
    case MPU6050_ACCEL_CONFIG_AFS_SEL_1:
      return 8.192;
    case MPU6050_ACCEL_CONFIG_AFS_SEL_2:
      return 4.096;
    case MPU6050_ACCEL_CONFIG_AFS_SEL_3:
      return 2.048;
    default:
      return -1;
  }
}

double mpu6050_gyro_factor(char SET){
  switch(SET){
    case MPU6050_GYRO_CONFIG_FS_SEL_0:
      return 131.0;
    case MPU6050_GYRO_CONFIG_FS_SEL_1:
      return 65.5;
    case MPU6050_GYRO_CONFIG_FS_SEL_2:
      return 32.8;
    case MPU6050_GYRO_CONFIG_FS_SEL_3:
      return 16.4;
    default:
      return -1;
  }
}

void setup(){
  
   start_time = micros();
   
  /* Start a transmission to the I2C device MPU */
  Wire.begin();
  
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(MPU6050_CONFIG_REG_ADDR);                // Config Low Pass Filter
  Wire.write(MPU6050_DLPF_5HZ);             // Set to 256Hz (Maximum Bandwidth)
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(GYRO_CONFIG_REG_ADDR);           // Config Gyro Full Scale Range
  Wire.write(MPU6050_GYRO_CONFIG_SET);         // Set to +-500deg/s
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(MPU6050_ACCEL_CONFIG_REG_ADDR);          // Config Accel Full Scale Range
  Wire.write(MPU6050_ACCEL_CONFIG_SET);               // Set to +-4g
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(MPU6050_PWR_MGMT_1_REG_ADDR);              // PWR_MGMT_1 register
  Wire.write(0);                                        // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  /* End a transmission to the I2C device MPU */
  
  /* Start a serial connection to print the output angles (debug) */
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  } 
}

void loop(){
  static int k = 0;
  static double gyx_drift = 0.0;
  static double gyy_drift = 0.0;
  static double gyz_drift = 0.0;
  
  /* Start a transmission to the I2C device MPU */
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x3B);                // Tell MPU that the next commands will refer to 0x3B
  Wire.endTransmission(false);     // Write, but keep the connection active
  Wire.requestFrom(MPU_I2C_ADDR,14,true);   // request a total of 14 registers beginning from 0x3B
  acx=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  acy=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acz=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyx=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyy=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyz=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  /* Calibration Steps - Average drift*/
  if(k < CALIBRATION_TIME){
    if(k > 0){
      gyx_drift = ((gyx_drift*k) + gyx)/(double)(k+1);
      gyy_drift = ((gyy_drift*k) + gyy)/(double)(k+1);
      gyz_drift = ((gyz_drift*k) + gyz)/(double)(k+1);
    }else{
      gyx_drift = gyx;
      gyy_drift = gyy;
      gyz_drift = gyz;
    }
    k++;
    if(k == CALIBRATION_TIME){
      Serial.print("Calibration:"); Serial.print(acx);
      Serial.print(" gyx_drift = "); Serial.print(gyx_drift);
      Serial.print(" gyy_drift = "); Serial.print(gyy_drift);
      Serial.print(" gyz_drift = "); Serial.println(gyz_drift);
    }
    return;
  }
  
  /* After Calibration */
  
  /* Acceleration measured by the accelerometer */
  accel_x = acx/(double)mpu6050_accel_factor(MPU6050_ACCEL_CONFIG_SET);
  accel_y = acy/(double)mpu6050_accel_factor(MPU6050_ACCEL_CONFIG_SET);
  accel_z = acz/(double)mpu6050_accel_factor(MPU6050_ACCEL_CONFIG_SET);
  
  /* Angles from acceleration */
  accel_roll = atan2(accel_z, sqrt(pow(accel_x,2)+pow(accel_y,2)));
  accel_pitch = atan2(accel_y, sqrt(pow(accel_x,2)+pow(accel_z,2)));
  
  /* Angular rate from Gyroscope */
  gyro_pitch_rate = -((double)gyz-gyz_drift)/(double)mpu6050_gyro_factor(MPU6050_GYRO_CONFIG_SET)*PI/180;
  gyro_roll_rate = ((double)gyy-gyy_drift)/(double)mpu6050_gyro_factor(MPU6050_GYRO_CONFIG_SET)*PI/180;
  gyro_yaw_rate = ((double)gyx-gyx_drift)/(double)mpu6050_gyro_factor(MPU6050_GYRO_CONFIG_SET);
  
  /* Calculate Pitch, Roll with Kalman Filter, using gyroscope and Accel */
  kalman_roll = kalmanCalculateRoll(accel_roll, gyro_roll_rate);
  kalman_pitch = kalmanCalculatePitch(accel_pitch, gyro_pitch_rate);
  
  /* Calculate Yaw by integration of gyroscope */
  integral_yaw = CalculateYaw(gyro_yaw_rate, 0.0);
    
  /* Waits to print the next ones */
  if((micros() - start_time) > DELAY_TO_PRINT){
    start_time = micros();
    Serial.print(DT);
    
    if(PRINT_RAW) {
      /* Print values to the Arduino's serial port */
      Serial.print("AcX = "); Serial.print(acx);
      Serial.print(" | AcY = "); Serial.print(acy);
      Serial.print(" | AcZ = "); Serial.print(acz);
      Serial.print(" | Tmp = "); Serial.print(temp/340.00+36.53);  //equation for temperature in degrees C from datasheet
      Serial.print(" | GyX = "); Serial.print(gyx);
      Serial.print(" | GyY = "); Serial.print(gyy);
      Serial.print(" | GyZ = "); Serial.println(gyz);
    }
    
    if(PRINT_ANG_RATES) {
      /* Print value of angular rates */
      Serial.print("Pitch_Rate = "); Serial.print(gyro_pitch_rate*180/PI);
      Serial.print(" | Roll_Rate = "); Serial.print(gyro_roll_rate*180/PI);
      Serial.print(" | Yaw_Rate = "); Serial.println(gyro_yaw_rate*180/PI);
    }
    
    if(PRINT_ANGLES) {
      /* Print value of angles */
      Serial.print("Accel_Pitch = "); Serial.print(accel_pitch*180/PI, DEC_PLACES);
      Serial.print(" | Kalman_Pitch = "); Serial.print(kalman_pitch*180/PI, DEC_PLACES);
      Serial.print(" | Accel_Roll = "); Serial.print(accel_roll*180/PI, DEC_PLACES);
      Serial.print(" | Kalman_Roll = "); Serial.print(accel_roll*180/PI, DEC_PLACES);
      Serial.print(" | Integral Yaw = "); Serial.println(integral_yaw, DEC_PLACES);
    }
    
  }
}

double CalculateYaw(double newRate, double startAngle){
   static unsigned long last_calculation_time = micros();
   static double x_angle = startAngle;
  
   double dt = (double)(micros() - last_calculation_time)/1000000.0;
   last_calculation_time = micros();
   x_angle += newRate*dt;
}
   

double kalmanCalculateRoll(double newAngle, double newRate){
  
  /* Kalman Filter Variables */
  static unsigned long last_calculation_time = micros();
  static double P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
  static double x_angle = 0.0;
  static double x_bias = 0;
  const double Q_angle  =  0.01; //0.001
  const double Q_gyro   =  0.0003;  //0.003
  const double R_angle  =  0.01;  //0.03
  double  y, S;
  double K_0, K_1;
  
  double dt = (double)(micros() - last_calculation_time)/1000000;
  
  DT = dt;
  last_calculation_time = micros();
  x_angle += dt * (newRate - x_bias);
  P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 +=  - dt * P_11;
  P_10 +=  - dt * P_11;
  P_11 +=  + Q_gyro * dt;
  
  y = newAngle - x_angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;
  
  x_angle +=  K_0 * y;
  x_bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;
  
  return x_angle;
}

double kalmanCalculatePitch(double newAngle, double newRate){
  
  /* Kalman Filter Variables */
  static unsigned long last_calculation_time = micros();
  static double P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
  static double x_angle = 0.0;
  static double x_bias = 0;
  const double Q_angle  =  0.01; //0.001
  const double Q_gyro   =  0.0003;  //0.003
  const double R_angle  =  0.01;  //0.03
  double  y, S;
  double K_0, K_1;
  
  double dt = (double)(micros() - last_calculation_time)/1000000;
  last_calculation_time = micros();
  x_angle += dt * (newRate - x_bias);
  P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 +=  - dt * P_11;
  P_10 +=  - dt * P_11;
  P_11 +=  + Q_gyro * dt;
  
  y = newAngle - x_angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;
  
  x_angle +=  K_0 * y;
  x_bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;
  
  return x_angle;
}
