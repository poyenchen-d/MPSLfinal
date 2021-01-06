/*
 * mpu6050.h
 *
 *  Created on: 2021¦~1¤ë6¤é
 *      Author: user
 */

#ifndef INC_MPU6050_MPU6050_H_
#define INC_MPU6050_MPU6050_H_

#define MPU6050_RA_WHO_AM_I         0x75

#define DEFAULT_MPU_HZ  (200)
#define q30  1073741824.0f

struct Quaternion {
	float w;
	float x;
	float y;
	float z;
};

struct Euler {
	float x;
	float y;
	float z;
};

struct Gravity {
	float x;
	float y;
	float z;
};

struct YPR {
	float yaw;
	float pitch;
	float roll;
};

void MPU6050_Init();

int MPU6050_GetQuaternion(struct Quaternion *q);

void MPU6050_ToEuler(struct Euler *p, const struct Quaternion *q);

void MPU6050_ToGravity(struct Gravity *g, const struct Quaternion *q);

void MPU6050_ToYPR(struct YPR *p, const struct Quaternion *q, const struct Gravity *g);

#endif /* INC_MPU6050_MPU6050_H_ */
