#include "Kinematics.h"

// 设置轮子的间距
void Kinematics::set_wheel_distance(float distance)
{
    wheel_distance = distance;
}

void Kinematics::set_motor_param(uint8_t id, float per_pluse_distance)
{
    motor_param[id].per_pulse_distance = per_pluse_distance;
}


// 运动学正解,将左右轮的速度转换成线速度和角速度
void Kinematics::kinematics_forward(float left_speed, float right_speed, float *out_linear_speed, float *out_angular_speed)
{
    *out_linear_speed = (left_speed + right_speed) / 2;
    *out_angular_speed = (right_speed - left_speed) / wheel_distance;   
}


// 运动学逆解，将线速度和角速度转换成左右轮的速度
void Kinematics::kinematics_inverse(float linear_speed, float angular_speed, float *out_left_speed, float *out_right_speed)
{
    *out_left_speed = linear_speed - angular_speed * wheel_distance / 2;
    *out_right_speed = linear_speed + angular_speed * wheel_distance / 2;
}

// 输入：左右轮脉冲数，当前时间，输出更新电机速度和编码器数据
void Kinematics::update_motor_speed(uint64_t current_time, int32_t leftback_tick, int32_t rightback_tick, 
                                    int32_t leftforward_tick, int32_t rightforward_tick)
{
    int16_t dt = current_time - last_update_time; // ms

    delta_ticks[0] = leftback_tick - motor_param[0].last_encoder_ticks;
    delta_ticks[1] = rightback_tick - motor_param[1].last_encoder_ticks;
    delta_ticks[2] = leftforward_tick - motor_param[2].last_encoder_ticks;
    delta_ticks[3] = rightforward_tick - motor_param[3].last_encoder_ticks;

    for(int i=0;i<4;i++){
        motor_param[i].motor_speed = (delta_ticks[i] * 105.805) / dt;
    }
    
    // 为了下次还可以正常的计算速度
    motor_param[0].last_encoder_ticks = leftback_tick;
    motor_param[1].last_encoder_ticks = rightback_tick;
    motor_param[2].last_encoder_ticks = leftforward_tick;
    motor_param[3].last_encoder_ticks = rightforward_tick;
    last_update_time = current_time;

    update_odom(dt);
}
// 获取电机速度,返回值是速度
int16_t Kinematics::get_motor_speed(uint8_t id)
{
    if (id < 0 || id > 3)
    {
        return -1;
    }
    return motor_param[id].motor_speed;
}

 odom_t& Kinematics::get_odom()
 {
    return odom;
 }

void Kinematics::TransAngleInPI(float angle,float& out_angle)
{
    if(angle>PI)
    {
        out_angle -= 2*PI;
    }else if (angle<-PI)
    {
        out_angle += 2*PI;
    }
}

void Kinematics::update_odom(uint16_t dt)
{
    float dt_s = float(dt)/1000.0; // ms -> s
    // 获取实时的角速度和线速度呢？我们拿左右轮实时的速度，进行运动学正解
    this->kinematics_forward(motor_param[0].motor_speed,motor_param[1].motor_speed,&odom.linear_speed,&odom.angular_speed);
    // 计算里程计信息
    odom.linear_speed = odom.linear_speed/1000.0; // 转换成米每秒

    // 角度积分
    odom.angle += odom.angular_speed*dt_s; 
    TransAngleInPI(odom.angle,odom.angle);
    // 计算机器人行走的距离（沿自身前进方向的）
    float delta_distance = odom.linear_speed * dt_s;
    // 分解到X轴和Y轴
    odom.x += delta_distance * std::cos(odom.angle);
    odom.y += delta_distance * std::sin(odom.angle);
}