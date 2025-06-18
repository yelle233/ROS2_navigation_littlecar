#include <Arduino.h>
#include <Esp32PcntEncoder.h>
#include <Esp32McpwmMotor.h>
#include <PidController.h>
#include <Kinematics.h>

// 引入Microros和wifi相关的库
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <micro_ros_utilities/string_utilities.h>

// 声明一些相关的结构体对象
rcl_allocator_t allocator; // 内存分配器，用于动态内存分配管理
rclc_support_t support;    // 用于存储时钟，内存分配器和上下文，用于提供支持
rcl_node_t node;           // 节点，用于创建节点
rclc_executor_t executor;  // 执行器，用于管理订阅和计时器回调的执行
rcl_subscription_t subscriber;//订阅者
geometry_msgs__msg__Twist sub_msg;//存储订阅到的速度信息

rcl_publisher_t odom_publisher;//发布者
nav_msgs__msg__Odometry odom_msg;//里程计信息
rcl_timer_t timer;//定时器

Esp32PcntEncoder encoders[4]; // 创建一个数组用于存储两个编码器
Esp32McpwmMotor motor;        // 创建一个名为motor的对象，用于控制电机
PidController pid_controller[4];//四个电机的pid
Kinematics kinematics;//运动正逆解对象

float target_linear_speed = 0.0; // 单位 毫米每秒
float target_angular_speed = 0.0; // 单位 弧度每秒
float out_left_speed = 0.0;       // 输出的左右轮速度，不是反馈的左右轮速度
float out_right_speed = 0.0;

// 定时器的回调函数
void timer_callback(rcl_timer_t* timer,int64_t last_call_time)
{
    // 完成里程计的发布
    odom_t  odom = kinematics.get_odom(); // 获取当前的里程计
    int64_t stamp = rmw_uros_epoch_millis();  // 获取当前的时间
    odom_msg.header.stamp.sec = static_cast<int32_t>(stamp/1000); // 秒部分
    odom_msg.header.stamp.nanosec = static_cast<int32_t>((stamp%1000)*1e6);  // 纳秒部分
    odom_msg.pose.pose.position.x = odom.x;
    odom_msg.pose.pose.position.y = odom.y;
    odom_msg.pose.pose.orientation.w = cos(odom.angle*0.5);
    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = sin(odom.angle*0.5);
    odom_msg.twist.twist.linear.x = odom.linear_speed;
    odom_msg.twist.twist.angular.z = odom.angular_speed;
    // 发布里程计，把数据发出去
    if(rcl_publish(&odom_publisher,&odom_msg,NULL)!=RCL_RET_OK)
    {
        Serial0.println("error: odom pub failed!");
    }
}

//姿态更新回调函数
void twist_callback(const void * msg_in){
  const geometry_msgs__msg__Twist* twist_msg=(const geometry_msgs__msg__Twist*)msg_in;
  kinematics.kinematics_inverse(twist_msg->linear.x*1000, twist_msg->angular.z, &out_left_speed, &out_right_speed);
  pid_controller[0].update_target(out_left_speed);
  pid_controller[1].update_target(out_right_speed);
  pid_controller[2].update_target(out_left_speed);
  pid_controller[3].update_target(out_right_speed);
}

// 单独创建一个任务运行 micro-ROS 相当于一个线程
void microros_task(void* args)
{
    // 1.设置传输协议并延迟一段时间等待设置的完成
    Serial1.begin(115200);
    set_microros_serial_transports(Serial1);// 设置microros的传输协议为串口通信
    delay(2000); // 等待2秒,等待WIFI连接
    // 2.初始化内存分配器
    allocator = rcl_get_default_allocator(); // 获取默认的内存分配器
    // 3.初始化支持
    rclc_support_init(&support,0,NULL,&allocator); // 初始化支持
    // 4.初始化节点
    rclc_node_init_default(&node,"bot_motion_control","",&support); // 初始化节点
    // 5.初始化执行器
    unsigned int num_handles = 2; // 订阅和计时器的回调数量,注意这是一个要改的参数
    rclc_executor_init(&executor,&support.context,num_handles,&allocator); // 初始化执行器
    //6.初始化订阅者并将其添加到执行器

    //订阅cmd_vel的订阅者
    rclc_subscription_init_best_effort(&subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"/cmd_vel");
    rclc_executor_add_subscription(&executor,&subscriber,&sub_msg,&twist_callback,ON_NEW_DATA);


    //发布odom里程计的发布者
    odom_msg.header.frame_id=micro_ros_string_utilities_set(odom_msg.header.frame_id,"odom");
    odom_msg.child_frame_id=micro_ros_string_utilities_set(odom_msg.child_frame_id,"base_footprint");
    rclc_publisher_init_best_effort(&odom_publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,msg,Odometry),"/odom");
    // 时间同步
    while (!rmw_uros_epoch_synchronized())
    {
       rmw_uros_sync_session(1000);
       delay(10);
    }
    rclc_timer_init_default(&timer,&support,RCL_MS_TO_NS(50),timer_callback);
    rclc_executor_add_timer(&executor,&timer);

    // 循环执行器
    rclc_executor_spin(&executor);
}


// v=wr r = v/w 
void setup()
{
    // 初始化串口
    Serial0.begin(115200); // 初始化串口通信，设置通信速率为115200
    // 初始化电机驱动器
    motor.attachMotor(0,5,4);
    motor.attachMotor(1,15,16);
    motor.attachMotor(2,10,9);
    motor.attachMotor(3,13,14);
    // 初始化编码器
    encoders[0].init(0, 7, 6);
    encoders[1].init(1, 47, 48);
    encoders[2].init(2, 12, 11);
    encoders[3].init(3, 1, 2);
    // 初始化PID控制器的参数
    for(int i=0;i<4;i++){
      pid_controller[i].update_pid(0.625, 0.125, 0.0);
    }
    for(int i=0;i<4;i++){
      pid_controller[i].out_limit(-100, 100);
    }
    // 初始化运动学参数
    kinematics.set_wheel_distance(170); // mm
    for(int i=0;i<4;i++){
      kinematics.set_motor_param(i, 0.154699636);
    }
    // 测试下运动学逆解
    kinematics.kinematics_inverse(target_linear_speed, target_angular_speed, &out_left_speed, &out_right_speed);
    Serial0.printf("OUT:left_speed=%f,right_speed=%f\n", out_left_speed, out_right_speed);
    pid_controller[0].update_target(out_left_speed);
    pid_controller[1].update_target(out_right_speed);
    pid_controller[2].update_target(out_left_speed);
    pid_controller[3].update_target(out_right_speed);

      // 创建一个任务运行 micro-ROS
      xTaskCreatePinnedToCore(microros_task, "microros_task",  10240, NULL, 1, NULL, 1);
}

void loop()
{
    delay(10); // 等待10毫秒
               // 调用PID，获取动态输出值
    kinematics.update_motor_speed(millis(), encoders[0].getTicks(), encoders[1].getTicks(), encoders[2].getTicks(), encoders[3].getTicks());
    for(int i=0;i<4;i++){
      motor.updateMotorSpeed(i, pid_controller[i].update(kinematics.get_motor_speed(i)));
    }
    
    Serial0.printf("x=%f,y=%f,angle=%f\n",kinematics.get_odom().x,kinematics.get_odom().y,kinematics.get_odom().angle);
}

