/*
 * imu.c
 *
 *  Created on: 30 окт. 2023 г.
 *      Author: archuser
 */

#include <math.h>
#include "main.h"
#include "imu.h"
#include "mpu-650.h"

/* X, Y, Z */
static float Xgyro_error, Ygyro_error, Zgyro_error;
static float Xaccel_error, Yaccel_error, Zaccel_error;

/* Board should be in static position*/
void IMU_Gyro_get_meas_error(){
	float X, Y, Z;
	for (uint16_t i = 0; i < 200; ++i){
		IMU_Get_Gyro_XYZ(&X, &Y, &Z);
		Xgyro_error += X;
		Ygyro_error += Y;
		Zgyro_error += Z;
		//SysTick_Delay(10);
	}
	Xgyro_error /= (float)200;
	Ygyro_error /= (float)200;
	Zgyro_error /= (float)200;

}

/* This function used to calibrate accelerometer */
void IMU_Accel_get_meas_error(){
	float X, Y, Z;
	for (uint16_t i = 0; i < 200; ++i){
		uint8_t x_l, x_h, y_l, y_h, z_l, z_h;

		MPU650_Read(MPU650_READ_ADDRESS, MPU650_ACCEL_XOUT_L, &x_l, 1);
		MPU650_Read(MPU650_READ_ADDRESS, MPU650_ACCEL_XOUT_H, &x_h, 1);
		MPU650_Read(MPU650_READ_ADDRESS, MPU650_ACCEL_YOUT_L, &y_l, 1);
		MPU650_Read(MPU650_READ_ADDRESS, MPU650_ACCEL_YOUT_H, &y_h, 1);
		MPU650_Read(MPU650_READ_ADDRESS, MPU650_ACCEL_ZOUT_L, &z_l, 1);
		MPU650_Read(MPU650_READ_ADDRESS, MPU650_ACCEL_ZOUT_H, &z_h, 1);

		X = (float)((int16_t)(x_h << 8 | x_l));
		Y = (float)((int16_t)(y_h << 8 | y_l));
		Z = (float)((int16_t)(z_h << 8 | z_l));

		Xaccel_error += (X / MPU650_ACCELEROMETER_Sensitivity_2G);
		Yaccel_error += (Y / MPU650_ACCELEROMETER_Sensitivity_2G);
		Zaccel_error += (Z / MPU650_ACCELEROMETER_Sensitivity_2G);

	}
	Xaccel_error /= (float)200;
	Yaccel_error /= (float)200;
	Zaccel_error /= (float)200;

}


void IMU_Gyro_Disable(){
	// TODO
}



bool IMU_MPU6050_Test(void){
	uint8_t test;
	MPU650_Read(MPU650_READ_ADDRESS, MPU650_WHO_AM_I, &test, 1);
	if (test == 0x68){
		return 0;
	}
	return 1;

}

void IMU_Get_Accel_XYZ(float* X, float* Y, float* Z){
	uint8_t x_l, x_h, y_l, y_h, z_l, z_h;

	MPU650_Read(MPU650_READ_ADDRESS, MPU650_ACCEL_XOUT_L, &x_l, 1);
	MPU650_Read(MPU650_READ_ADDRESS, MPU650_ACCEL_XOUT_H, &x_h, 1);
	MPU650_Read(MPU650_READ_ADDRESS, MPU650_ACCEL_YOUT_L, &y_l, 1);
	MPU650_Read(MPU650_READ_ADDRESS, MPU650_ACCEL_YOUT_H, &y_h, 1);
	MPU650_Read(MPU650_READ_ADDRESS, MPU650_ACCEL_ZOUT_L, &z_l, 1);
	MPU650_Read(MPU650_READ_ADDRESS, MPU650_ACCEL_ZOUT_H, &z_h, 1);

	/* Little-Endian */
	*X = (int16_t)(x_h << 8 | x_l) - Xaccel_error;
	*Y = (int16_t)(y_h << 8 | y_l) - Yaccel_error;
	*Z = (int16_t)(z_h << 8 | z_l) - Zaccel_error - (float)2000; /* Z correction */


	*X = (float)(*X) / MPU650_ACCELEROMETER_Sensitivity_2G;
	*Y = (float)(*Y) / MPU650_ACCELEROMETER_Sensitivity_2G;
	*Z = (float)(*Z) / MPU650_ACCELEROMETER_Sensitivity_2G;

}

void IMU_Get_Gyro_XYZ(float* X, float* Y, float* Z){
	uint8_t x_l, x_h, y_l, y_h, z_l, z_h;

	MPU650_Read(MPU650_READ_ADDRESS, MPU650_GYRO_XOUT_L, &x_l, 1);
	MPU650_Read(MPU650_READ_ADDRESS, MPU650_GYRO_XOUT_H, &x_h, 1);
	MPU650_Read(MPU650_READ_ADDRESS, MPU650_GYRO_YOUT_L, &y_l, 1);
	MPU650_Read(MPU650_READ_ADDRESS, MPU650_GYRO_YOUT_H, &y_h, 1);
	MPU650_Read(MPU650_READ_ADDRESS, MPU650_GYRO_ZOUT_L, &z_l, 1);
	MPU650_Read(MPU650_READ_ADDRESS, MPU650_GYRO_ZOUT_H, &z_h, 1);

	/* Little-Endian */
	*X = (int16_t)(x_h << 8 | x_l) - Xgyro_error;
	*Y = (int16_t)(y_h << 8 | y_l) - Ygyro_error;
	*Z = (int16_t)(z_h << 8 | z_l) - Zgyro_error;


	*X = (float)(*X) / MPU650_GYRO_250dps;
	*Y = (float)(*Y) / MPU650_GYRO_250dps;
	*Z = (float)(*Z) / MPU650_GYRO_250dps;
}


/*
 * Count angles with Madgwick filter
 */

float clamp(float trigan){
	if (trigan > 1.0f){
		return 1.0f;
	}
	else if (trigan < -1.0f){
		return -1.0f;
	}
	else{
		return trigan;
	}

}

void IMU_get_angles(float d_time, float gX, float gY, float gZ, float aX, float aY, float aZ, IMU_ANGLES* Angles){
	/* a(t) = (1-K) * (a(t-1) + gx*dt) + K * acc */

	Angles->Roll = (1-0.15) * (Angles->Roll + gX*d_time) + 0.15*(90 - RAD_TO_DEG * acosf(clamp(aX*DEG_TO_RAD)));

}
