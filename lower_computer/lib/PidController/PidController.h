#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

class PidController
{
public:
    PidController() = default;
    PidController(float kp, float ki, float kd);

private:
    // PID 参数，可以调节的
    float target_;
    float out_min_;
    float out_max_;
    float kp_;
    float ki_;
    float kd_;
    float intergral_up_ = 2500; // 积分上限
    // pid 中间过程值
    float error_;
    float error_sum_;
    float derror_;
    float prev_error_;

public:
    float update(float current);                   // 提供当前值，返回下次输出值，也就是PID的结果
    void update_target(float target);              // 更新目标值
    void update_pid(float kp, float ki, float kd); // 更新PID参数
    void reset();                                  // 重置PID
    void out_limit(float min, float max);          // 设置输出限制
};

#endif // __PID_CONTROLLER_H__