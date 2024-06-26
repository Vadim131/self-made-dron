/*
 * imu.h
 *
 *  Created on: 30 окт. 2023 г.
 *      Author: archuser
 */

#ifndef IMU_H_
#define IMU_H_

void IMU_Get_Gyro_XYZ(float* X, float* Y, float* Z);
bool IMU_MPU6050_Test(void);
void IMU_Get_Accel_XYZ(float* X, float* Y, float* Z);

void IMU_Gyro_get_meas_error();
void IMU_Accel_get_meas_error();

void MPU650_Init();

void
IMU_get_angles(float d_time, float gX, float gY, float gZ, float aX, float aY, float aZ, IMU_ANGLES* Angles);

#define PI			(float)3.141596
#define RAD_TO_DEG	(float)57.29577
#define DEG_TO_RAD	(float)0.0174533


/* Gyroscope sensitivity */
#define L3GD20_Sensitivity_250dps     (float)0.00875
#define L3GD20_Sensitivity_500dps     (float)0.0175f
#define L3GD20_Sensitivity_2000dps    (float)0.7f

/* MPU-6050 sensitivity */
#define MPU650_ACCELEROMETER_Sensitivity_2G  			(float)16384  /* ±2  g */
#define MPU650_ACCELEROMETER_Sensitivity_4G  			(float)8192   /* ±4  g */
#define	MPU650_ACCELEROMETER_Sensitivity_8G  			(float)4096 	/* ±8  g */
#define	MPU650_ACCELEROMETER_Sensitivity_16G          	(float)2048 	/* ±16 g */


static struct quaternion {
    float q1;
    float q2;
    float q3;
    float q4;
};

#ifndef GYRO_MEAN_ERROR
    #define GYRO_MEAN_ERROR PI * (5.0f / 180.0f) // 5 deg/s gyroscope measurement error (in rad/s)  *from paper*
#endif

#ifndef BETA
    #define BETA sqrt(3.0f/4.0f) * GYRO_MEAN_ERROR
#endif

// IMU consists of a Gyroscope plus Accelerometer sensor fusion
void imu_filter(float ax, float ay, float az, float gx, float gy, float gz, float dt);
void eulerAngles(struct quaternion q, float* roll, float* pitch, float* yaw);



#endif /* IMU_H_ */
