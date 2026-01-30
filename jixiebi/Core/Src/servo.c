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
    result.j[6] = 40.0f;
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

    // --- 2. 计算“腕关节”坐标 (J4的轴心) ---
    // 我们不能直接解算末端，要先扣除掉 L4（夹爪）带来的偏移
    float pitch_rad = pitch_deg * (M_PI / 180.0f);
    float r_target = sqrt(x*x + y*y);
    
    // 扣除 L4 在水平和垂直方向的分量
    // r_wrist: J1轴心到J4轴心的水平距离
    // z_wrist: J2轴心到J4轴心的垂直高度 (注意 z 要减去底座 L1)
    float r_wrist = r_target - L4 * cos(pitch_rad);
    float z_wrist = (z - L1) - L4 * sin(pitch_rad);

    // --- 3. 三角形余弦定理求解 J2, J3 ---
    // 计算 J2 到 J4 的直线距离 (也就是三角形的第三边)
    float vector_len_sq = r_wrist * r_wrist + z_wrist * z_wrist;
    float vector_len = sqrt(vector_len_sq);

    // 检查：如果目标太远，手够不着，直接限制在最大长度
    if (vector_len > (L2 + L3)) {
        vector_len = L2 + L3; // 强行拉回
        // 此时通常意味着手臂伸直
    }

    // 利用余弦定理求 J3 (中间那个关节的弯曲角)
    // c^2 = a^2 + b^2 - 2ab*cos(C)  =>  cos(C) = (a^2 + b^2 - c^2) / 2ab
    float cos_angle_j3 = (L2*L2 + L3*L3 - vector_len_sq) / (2 * L2 * L3);
    
    // 再次限幅，防止计算 acos 出错
    if (cos_angle_j3 > 1.0f) cos_angle_j3 = 1.0f;
    if (cos_angle_j3 < -1.0f) cos_angle_j3 = -1.0f;

    // 计算出 J3 相对直线的夹角 (弧度)
    // 注意：这里的几何模型通常算出来的是三角形内角
    float j3_inner_angle = acos(cos_angle_j3);
    
    // --- 4. 转换 J2, J3 为舵机角度 ---
    
    // 计算大臂(J2)的基础仰角 (atan2)
    float j2_base_angle = atan2(z_wrist, r_wrist);
    
    // 计算大臂由三角形引起的额外抬升角
    float cos_angle_j2_offset = (L2*L2 + vector_len_sq - L3*L3) / (2 * L2 * vector_len);
    if (cos_angle_j2_offset > 1.0f) cos_angle_j2_offset = 1.0f;
    float j2_offset_angle = acos(cos_angle_j2_offset);

    // [关键] 映射到 90度中心的舵机
    // J2 (大臂): 数学角度 90度是垂直向上。如果你的舵机装配也是90度垂直，则直接转换
    float j2_math = (j2_base_angle + j2_offset_angle) * (180.0f / M_PI);
    result.j[2] = j2_math; 
    // *调试提示*: 如果J2方向反了，改为: result.j[2] = 180.0f - j2_math;

    // J3 (小臂): 数学上算出的是相对于大臂的折叠角
    // 如果 J3 是直的(和J2成一条线)，舵机应该是 90度? 还是 180度?
    // 通常 SG90 堆叠结构，90度是直角弯曲。我们需要根据你的实际安装微调。
    // 这里假设：90度时，小臂和大臂垂直。0度时折叠，180度伸直。
    // j3_inner_angle 是三角形内角。伸直时 vector_len最大，cos=-1, angle=180.
    float j3_math = j3_inner_angle * (180.0f / M_PI);
    
    // [根据你的照片修正]
    // 你的照片里 J3 看起来是 90 度时也是直立的。
    // 逆运动学算出的 j3 是"关节角"。
    // 这里最容易反，建议先用这个公式，如果反了就用 (180 - result.j[3])
    result.j[3] = 180.0f - j3_math + 90.0f; // 这里的修正项可能需要你实测微调
    // 修正逻辑：如果算出来需要伸直，j3_math接近180。舵机应该去90度？不，伸直通常是舵机的一端。
    // 让我们可以这样：J3 输出 90 代表垂直于大臂。
    
    // --- 5. 求解 J4 (手腕) ---
    // 我们的目标是：J2_angle + J3_angle + J4_angle = Target_Pitch
    // 所以 J4 = Pitch - J2 - J3
    // 这里必须全部用“相对于水平面的绝对角度”或者全部用“相对上一级角度”
    
    // 简单几何法：J4舵机角度 = 期望Pitch - (当前大臂角度 + 小臂角度 - 修正值)
    // 因为舵机是串联的，为了抵消 J2 和 J3 的旋转，J4 必须反向转回来
    float j2_real = result.j[2] - 90.0f; // 转化为相对于垂直线的偏角
    float j3_real = result.j[3] - 90.0f;
    
    result.j[4] = pitch_deg - (j2_real + j3_real) + 90.0f;

    // // --- 6. J5, J6 (夹爪旋转与开合) ---
    // result.j[5] = 90.0f; // 默认放平
    // result.j[6] = 40.0f; // 默认张开

    // --- 7. 全局限幅 ---
    for(int k=1; k<=6; k++) {
        result.j[k] = Constrain_Angle(result.j[k]);
    }

    return result;
}

/**
 * @brief 对外接口：双模式逆运动学求解
 * @param x, y, z 目标坐标 (mm)
 * @param mode 0:自动巡航模式(最远), 1:平抓模式, 2:下抓模式
 */
IK_Result_t IK_Get_Target_Angle(float x, float y, float z,  IK_Mode mode) {
    float target_pitch = 0.0f;

    if (mode == MODE_AUTO_REACH) {
        // --- 策略1：自动模式 ---
        // 夹爪方向 = 手臂方向。计算从底座(0, L1)指向目标(r, z)的角度
        float r = sqrt(x*x + y*y);
        float h = z - L1;
        
        // 算出角度 (度)
        target_pitch = atan2(h, r) * (180.0f / M_PI);
        
    } else if (mode == MODE_GRAB_FLAT) {
        // --- 策略2：平抓模式 ---
        target_pitch = 0.0f; // 永远保持水平
        
    } else if (mode == MODE_GRAB_DOWN) {
        // --- 策略3：下抓模式 ---
        target_pitch = -90.0f; // 垂直向下掏
    }

    // 调用核心解算
    return IK_Solve_Core(x, y, z, target_pitch);
}

float Constrain_Angle(float angle) {
    if (angle < 0.0f) return 0.0f;
    if (angle > 180.0f) return 180.0f;
    return angle;
}
