#include "servo.h"

IK_Result_t result;

void servo_init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    result.j[1] = 90.0f;
    result.j[2] = 90.0f;
    result.j[3] = 90.0f;
    result.j[4] = 90.0f;
    result.j[5] = 90.0f;
    result.j[6] = 150.0f;
    servo_set_angle(result.j[1], result.j[2], result.j[3], result.j[4], result.j[5], result.j[6]);
}

uint16_t servo_pwm_calculate(float angle)
{
    return (uint16_t)(angle_zero + angle_rate * angle);
}

void servo_set_angle(float angle_1, float angle_2, float angle_3, float angle_4, float angle_5, float angle_6)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo_pwm_calculate(angle_1));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, servo_pwm_calculate(angle_2));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, servo_pwm_calculate(angle_3));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, servo_pwm_calculate(angle_4));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_pwm_calculate(angle_5));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_pwm_calculate(angle_6));
}

void servo_xyz(float x, float y, float z, IK_Mode mode) {
    result = IK_Get_Target_Angle(x, y, z, mode);
    servo_set_angle(result.j[1], result.j[2], result.j[3], result.j[4], result.j[5], result.j[6]);
}

IK_Result_t IK_Solve_Core(float x, float y, float z, float pitch_deg) {
    result.j[1] = atan2(y, x) * (180.0f / M_PI) + 90.0f;

    float pitch_rad = pitch_deg * (M_PI / 180.0f);
    float r_target = sqrt(x*x + y*y);
    float r_wrist = r_target - L4 * cos(pitch_rad);
    float z_wrist = (z - L1) - L4 * sin(pitch_rad);
    float vector_len_sq = r_wrist * r_wrist + z_wrist * z_wrist;

    float vector_len = sqrt(vector_len_sq);
    if (vector_len > (L2 + L3)) {
        vector_len = L2 + L3; 
    }

    float cos_angle_j3 = (L2*L2 + L3*L3 - vector_len_sq) / (2 * L2 * L3);
    if (cos_angle_j3 > 1.0f) cos_angle_j3 = 1.0f;
    if (cos_angle_j3 < -1.0f) cos_angle_j3 = -1.0f;
    float j3_inner_angle = acos(cos_angle_j3);
    float j3_math = j3_inner_angle * (180.0f / M_PI);
    result.j[3] = 180.0f - j3_math + 90.0f;

    float j2_base_angle = atan2(z_wrist, r_wrist);
    float cos_angle_j2_offset = (L2*L2 + vector_len_sq - L3*L3) / (2 * L2 * vector_len);
    if (cos_angle_j2_offset > 1.0f) cos_angle_j2_offset = 1.0f;
    float j2_offset_angle = acos(cos_angle_j2_offset);
    float j2_math = (j2_base_angle + j2_offset_angle) * (180.0f / M_PI);
    result.j[2] = j2_math; 

    float j2_real = result.j[2] - 90.0f;
    float j3_real = result.j[3] - 90.0f;
    // result.j[4] = pitch_deg - (j2_real + j3_real) + 90.0f;
    result.j[4] = pitch_deg - (j2_real + j3_real);

    for(int k=1; k<=6; k++) {
        result.j[k] = Constrain_Angle(result.j[k]);
    }

    return result;
}

IK_Result_t IK_Get_Target_Angle(float x, float y, float z,  IK_Mode mode) {
    float target_pitch = 0.0f;
    if (mode == MODE_AUTO_REACH) {
        float r = sqrt(x*x + y*y);
        float h = z - L1;
        target_pitch = atan2(h, r) * (180.0f / M_PI);
    }
     else if (mode == MODE_GRAB_FLAT) {
        target_pitch = 0.0f;
    }
     else if (mode == MODE_GRAB_DOWN) {
        target_pitch = -90.0f; 
    }
 
    return IK_Solve_Core(x, y, z, target_pitch);
}

float Constrain_Angle(float angle) {
    if (angle < 0.0f) return 0.0f;
    if (angle > 180.0f) return 180.0f;
    return angle;
}
