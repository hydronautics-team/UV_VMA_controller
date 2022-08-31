void imu9dof(float *acc_in, float *gyro_in, float* mag_i, float* out, float dt);
//void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, long double *buf);
void filterUpdate(float gx, float gy, float gz, float ax, float ay, float az,float *buf);
float kr (float var1,float var2,float var3);
float dif (float var1,float var2,float var3);
