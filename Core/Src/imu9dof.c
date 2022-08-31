#include <stdlib.h>

#include "imu9dof.h"

#define PI 3.1416f

#include <math.h>
// System constants
#define deltat 0.01f // sampling period in seconds (shown as 1 ms)
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
// Global system variables
float a_x, a_y, a_z; // accelerometer measurements
float w_x, w_y, w_z; // gyroscope measurements in rad/s
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions



struct EulerAngles {
	float roll, pitch, yaw;
};


struct EulerAngles ToEulerAngles (float q[4])
		{
			struct EulerAngles angles;
			float sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
			float cosr_cosp = 1-(2 * (q[1] * q[1] + q[2] * q[2]));
			angles.roll = atan2(sinr_cosp, cosr_cosp);
			float sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
			if (abs (sinp)>= 1) angles.pitch = copysign (M_PI / 2, sinp);
			else angles.pitch = asin (sinp);
			float siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
			float cosy_cosp = 1-2 * (q[2] * q[2] + q[3] * q[3]);
			angles.yaw = atan2 (siny_cosp, cosy_cosp);
			return angles;
		}

void imu9dof(float *acc_in, float *gyro_in, float* mag_in, float* out, float dt)
{
	//beta = 0.0f;
	float bufquat[4];
	if(dt == 0.2)
	{
		//сброс иму
		SEq_1 = 1.0f;
		SEq_2 = 0.0f;
		SEq_3 = 0.0f;
		SEq_4 = 0.0f;
		return;
	}
	if (abs(gyro_in[0])<0.8f)
		gyro_in[0]=0;

		if (abs(gyro_in[1])<0.8f)
			gyro_in[1]=0;

		if (abs(gyro_in[2])<0.8f)
			gyro_in[2]=0;
	filterUpdate(gyro_in[0] / 180 * PI,gyro_in[1] / 180 * PI,gyro_in[2] / 180 * PI,acc_in[0],acc_in[1],acc_in[2], bufquat);
	//MadgwickAHRSupdate(gyro_in[0],gyro_in[1],gyro_in[2],acc_in[0],acc_in[1],acc_in[2],mag_in[0],mag_in[1],mag_in[2], bufquat);
	struct EulerAngles buffer = ToEulerAngles (bufquat);
//	out[0] = buffer.pitch * 180 / PI;
//	out[1] = buffer.roll * 180 / PI;
	out[2] = buffer.yaw * 180 / PI*2.1;
	if(out[2]<0)
	{
		out[2] += 360;
	}
	out[0] = kr(acc_in[0],acc_in[1],acc_in[2]);
	out[1] = dif(acc_in[0],acc_in[1],acc_in[2]); //чисто по акселемерометрам
}


float kr (float var1,float var2,float var3)
{
	float temporary;
	temporary = atan((float)var1/sqrt((float)var2*(float)var2+(float)var3*(float)var3));   // [-pi/2..pi/2]
	return temporary*57.2958;
}

float dif (float var1,float var2,float var3)
{
	float temporary;
	temporary = (atan((float)var2/sqrt((float)var1*(float)var1+(float)var3*(float)var3)));   // [-pi/2..pi/2]
	return temporary*57.2958;
}

void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float *buf)
{
    // Local system variables
    float norm; // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
    float f_1, f_2, f_3; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    // Axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * SEq_1;
    float halfSEq_2 = 0.5f * SEq_2;
    float halfSEq_3 = 0.5f * SEq_3;
    float halfSEq_4 = 0.5f * SEq_4;
    float twoSEq_1 = 2.0f * SEq_1;
    float twoSEq_2 = 2.0f * SEq_2;
    float twoSEq_3 = 2.0f * SEq_3;
    // Normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;
    // Compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    // Compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
    // Normalise the gradient
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 /= norm;
    SEqHatDot_2 /= norm;
    SEqHatDot_3 /= norm;
    SEqHatDot_4 /= norm;
    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
    // Compute then integrate the estimated quaternion derrivative
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
    // Normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;

    	buf[0] = SEq_1;
		buf[1] = SEq_2;
		buf[2] = SEq_3;
		buf[3] = SEq_4;
}
