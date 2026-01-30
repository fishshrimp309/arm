#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "tim.h"
#include "math.h"

#define angle_zero 500
#define angle_rate 11.11111f
// 硬件参数：请根据你的实物测量并修改（单位：mm）
#define L1 60.0f  // 底座底面到j2的高度
#define L2 50.0f  // j2-j3长度
#define L3 50.0f  // j3-j4长度
#define L4 80.0f  // J4到夹爪末端长度
#define M_PI 3.14159265358979323846f

typedef struct {
    float j[7];//1,2,3,4,5,6角度
} IK_Result_t;

IK_Result_t IK_Solve_Geometry(float x, float y, float z);
void servo_init(void);
uint16_t servo_pwm_calculate(float angle);
void servo_set_angle(float angle_1, float angle_2, float angle_3, float angle_4, float angle_5, float angle_6);
void servo_xyz(float x, float y, float z, IK_Mode mode);
IK_Result_t IK_Solve_Core(float x, float y, float z, float pitch_deg);
IK_Result_t IK_Get_Target_Angle(float x, float y, float z,  IK_Mode mode);
float Constrain_Angle(float angle);

#endif
