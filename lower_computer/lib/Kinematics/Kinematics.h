#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "Arduino.h"

typedef struct
{
    float per_pulse_distance;   // 每个脉冲的前进的距离
    int16_t motor_speed;        // 单位用mm/s
    int64_t last_encoder_ticks; // 上一次点击的编码器读数
} motor_param_t;


typedef struct {
    float x;
    float y;
    float angle;
    float linear_speed;
    float angular_speed;
}odom_t;

/**
 * 1. 运动学正逆解（两个轮子的实时速度->当前实时的角速度和线速度 / 当前目标的角速度和线速度->两个轮子的目标速度）
 *
 */
class Kinematics
{
private:
    /* data */
    motor_param_t motor_param[4];//四个轮子的编码器，脉冲等信息
    int16_t delta_ticks[4] = {0, 0, 0, 0}; // 用于存储上一次读取的编码器数值
    uint64_t last_update_time = 0; // 用于存储上一次更新电机速度的时间,计算速度的时候使用
    float wheel_distance = 0.0;    // 两个轮子之间的距离
    odom_t odom; // 用于存储里程计信息
public:
    Kinematics(/* args */) = default;
    ~Kinematics() = default;

    odom_t& get_odom();
    void update_odom(uint16_t dt);
    void TransAngleInPI(float angle,float& out_angle);

    void set_wheel_distance(float distance); // 设置轮子的间距

    void set_motor_param(uint8_t id,float per_pluse_distance);
    // 运动学正解,将左右轮的速度转换成线速度和角速度
    void kinematics_forward(float left_speed,float right_speed,float* out_linear_speed,float* out_angular_speed);
    // 运动学逆解，将线速度和角速度转换成左右轮的速度
    void kinematics_inverse(float linear_speed,float angular_speed,float* out_left_speed,float* out_right_speed);
    // 更新点击速度和编码器数据
    void update_motor_speed(uint64_t current_time, int32_t leftback_tick, int32_t rightback_tick, 
        int32_t leftforward_tick, int32_t rightforward_tick);
    // 获取电机速度,返回值是速度
    int16_t get_motor_speed(uint8_t id);

};

#endif // __KINEMATICS_H__