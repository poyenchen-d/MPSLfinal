/*
 * mpu6050.c
 *
 *  Created on: 2021¦~1¤ë6¤é
 *      Author: user
 */
#include <math.h>
#include <stdint.h>
#include "MPU6050/mpu6050.h"
#include "MPU6050/inv_mpu.h"
#include "MPU6050/inv_mpu_dmp_motion_driver.h"

static signed char gyro_orientation[9] = { -1, 0, 0, 0, -1, 0, 0, 0, 1 };

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
	if (result == 0x7) {
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
	}
}

void MPU6050_Init()
{
	mpu_init();
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_gyro_fsr(500);
    mpu_set_accel_fsr(4);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    if (dmp_load_motion_driver_firmware()) {
    	return;
    }
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    dmp_enable_feature(
 	    DMP_FEATURE_6X_LP_QUAT |
	    DMP_FEATURE_TAP |
	    DMP_FEATURE_ANDROID_ORIENT |
	    DMP_FEATURE_SEND_RAW_ACCEL |
	    DMP_FEATURE_SEND_CAL_GYRO |
	    DMP_FEATURE_GYRO_CAL
    );
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    run_self_test();
    mpu_reg_dump();
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
