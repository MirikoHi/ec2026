//
// Created by gengcheese on 2026/7/11.
//

#ifndef DART_G_MAHONY_H
#define DART_G_MAHONY_H
#include <stdint.h>
void MahonyAHRSinit();
void MahonyAHRSupdate(float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float dt_acc,float dt_gyro);
void MahonyGetQuaternion(float *q0, float *q1, float *q2, float *q3);
void MahonyGetEuler(float *roll, float *pitch, float *yaw);
void MahonyGetRotationMatrix(float R[3][3]);
#endif //DART_G_MAHONY_H