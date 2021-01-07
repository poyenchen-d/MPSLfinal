/*
 * mpu6050.c
 *
 *  Created on: 2021�~1��6��
 *      Author: user
 */
#include <math.h>
#include <stdint.h>

#include "stm32l4xx_hal.h"
#include "MPU6050/mpu6050.h"
#include "MPU6050/inv_mpu.h"
#include "MPU6050/inv_mpu_dmp_motion_driver.h"

float accelScalingFactor, gyroScalingFactor, gyroRateFactor;
static signed char gyro_orientation[9] = { -1, 0, 0, 0, -1, 0, 0, 0, 1 };
float exInt=0;
float eyInt=0;
float ezInt=0;

HAL_StatusTypeDef IICwriteBits(uint8_t slave_addr, uint8_t reg_addr,
    uint8_t bitStart, uint8_t length, uint8_t data) {

  uint8_t tmp, dataShift;
  HAL_StatusTypeDef status = i2c_read(slave_addr, reg_addr, 1, &tmp);
  if (status == HAL_OK) {
    uint8_t mask = (((1 << length) - 1) << (bitStart - length + 1));
    dataShift = data << (bitStart - length + 1);
    tmp &= mask;
    tmp |= dataShift;
    return i2c_write(slave_addr, reg_addr, 1, &tmp);
  } else {
    return status;
  }
}

static unsigned short inv_row_2_scale(const signed char *row) {
	unsigned short b;

	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7;      // error
	return b;
}

static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx) {
	unsigned short scalar;
	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;

	return scalar;
}

static void run_self_test(void) {
	int result;
	long gyro[3], accel[3];

	result = mpu_run_self_test(gyro, accel);
	//if (result == 0x7) {
		/* Test passed. We can trust the gyro data here, so let's push it down
		 * to the DMP.
		 */
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long) (gyro[0] * sens);
		gyro[1] = (long) (gyro[1] * sens);
		gyro[2] = (long) (gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		//log_i("setting bias succesfully ......\r\n");
	//}
}

void MPU6050_setClockSource(uint8_t source) {
	IICwriteBits(0x68, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

int MPU6050_Init()
{
	if (mpu_init()) {
		return -1;
	}
    if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
    	return -1;
    }
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);

    if (mpu_set_gyro_fsr(250)) {
    	return -1;
    }
    gyroScalingFactor = (250.0f/32768.0f);
    gyroRateFactor = 131.0;
    if (mpu_set_accel_fsr(8)) {
    	return -1;
    }
    accelScalingFactor = (8000.0f/32768.0f);
    if (mpu_set_lpf(188)) {
    	return -1;
    }
    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
    	return -1;
    }
    if (mpu_set_sample_rate(DEFAULT_MPU_HZ)) {
    	return -1;
    }
    if (dmp_load_motion_driver_firmware()) {
    	return -1;
    }
    if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {
    	return -1;
    }
    if (dmp_enable_feature(
 	    DMP_FEATURE_6X_LP_QUAT |
	    DMP_FEATURE_TAP |
	    DMP_FEATURE_ANDROID_ORIENT |
	    DMP_FEATURE_SEND_RAW_ACCEL |
	    DMP_FEATURE_SEND_CAL_GYRO |
	    DMP_FEATURE_GYRO_CAL
    )) {
    	return -1;
    }
    if (dmp_set_fifo_rate(DEFAULT_MPU_HZ)) {
    	return -1;
    }
    run_self_test();
    if (mpu_set_dmp_state(1)) {
    	return -1;
    }
    mpu_reg_dump();

    return 0;
}

uint8_t MPU6050_isConnected(void) {
	uint8_t dev_id = 0;
	mpu_read_reg(MPU6050_RA_WHO_AM_I, &dev_id);
	if (dev_id == 0x68)  //0b01101000;
		return 1;
	else
		return 0;
}

int MPU6050_GetQuaternion(struct Quaternion *q)
{
	short gyro[3], accel[3], sensors;
	unsigned long sensor_timestamp;
    unsigned char more;
	long quat[4];

	if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more)) {
		return -1;
	}
	if (sensors & INV_WXYZ_QUAT) {
	    q->w = quat[0] / q30;
	    q->x = quat[1] / q30;
	    q->y = quat[2] / q30;
	    q->z = quat[3] / q30;
	}
	return 0;
}

void MPU6050_ToAccelScaled(struct ScaledData *scaled, const int16_t raw[3])
{
	scaled->x = ((raw[0] + ACCEL_X_CALIB) * accelScalingFactor);
	scaled->y = ((raw[1] + ACCEL_Y_CALIB) * accelScalingFactor);
	scaled->z = ((raw[2] + ACCEL_Z_CALIB) * accelScalingFactor);
}

void MPU6050_ToGyroScaled(struct ScaledData *scaled, const int16_t raw[3])
{
	scaled->x = ((raw[0] + GYRO_X_CALIB) * gyroScalingFactor);
	scaled->y = ((raw[1] + GYRO_Y_CALIB) * gyroScalingFactor);
	scaled->z = ((raw[2] + GYRO_Z_CALIB) * gyroScalingFactor);
}

void MPU6050_ToQuaternion(struct Quaternion *q, const struct ScaledData *accel, const struct ScaledData *gyro, float sec)
{
	const static float FACTOR = 0.001;

	float w_q = q->w;
	float x_q = q->x;
	float y_q = q->y;
	float z_q = q->z;
	float halfT = sec / 2.0;

	// normalise the measurements
	float norm = sqrt(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
	float ax = accel->x / norm;
	float ay = accel->y / norm;
	float az = accel->z / norm;

	// estimated direction of gravity
	float vx = 2.0 * ((x_q * z_q) - (w_q * y_q));
	float vy = 2.0 * (w_q * x_q + y_q * z_q);
	float vz = 1 - 2.0 * (x_q * x_q - y_q * y_q);

	  // error is sum of cross product between reference direction of field and direction measured by sensor
	float ex = (ay * vz - az * vy);
	float ey = (az * vx - ax * vz);
	float ez = (ax * vy - ay * vx);

	  // integral error scaled integral gain
	float delta_x = gyro->x * halfT + ex * FACTOR;
	float delta_y = gyro->y * halfT + ey * FACTOR;
	float delta_z = gyro->z * halfT + ez * FACTOR;

	q->w = w_q         - x_q*delta_x - y_q*delta_y - z_q*delta_z;
	q->x = w_q*delta_x + x_q         + y_q*delta_z - z_q*delta_y;
	q->y = w_q*delta_y - x_q*delta_z + y_q         + z_q*delta_x;
	q->z = w_q*delta_z + x_q*delta_y - y_q*delta_x + z_q;

	// normalise quaternion
	norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
	q->w = q->w / norm;
	q->x = q->x / norm;
	q->y = q->y / norm;
	q->z = q->z / norm;
}

void MPU6050_ToEuler(struct Euler *p, const struct Quaternion *q)
{
	//1* X-Axis calibrate
	p->x = atan2(2*((q->w*q->x) + (q->y*q->z)), (1 - 2*(q->x*q->x + q->y*q->y)));
	//2* Y-Axis calibrate
	p->y = asin(2*((q->w*q->y) - (q->x*q->z)));
	//3* Z-Axis calibrate
	p->z = atan2(2*((q->w*q->z) + (q->x*q->y)), (1 - 2*(q->z*q->z + q->y*q->y)));
}

void MPU6050_ToGravity(struct Gravity *g, const struct Quaternion *q)
{
	g->x = 2 * (q->x * q->z - q->w * q->y);
	g->y = 2 * (q->w * q->x + q->y * q->z);
	g->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
}

void MPU6050_ToYPR(struct YPR *p, const struct Quaternion *q, const struct Gravity *g)
{
	p->yaw = atan2(2 * (q->x*q->y - q->w*q->z), 2 * (q->w*q->w + q->x*q->x) - 1);
	p->pitch = atan(g->x / sqrt(g->y * g->y + g->z * g->z));
	p->roll = atan(g->y / sqrt(g->x * g->x + g->z * g->z));
}
