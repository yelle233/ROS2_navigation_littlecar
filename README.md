7# ROS2导航小车项目

## 项目简介
本项目采用ESP32S3充当下位机，NUC充当上位机。其中ESP32上运行microros发布odom里程计话题，同时订阅cmd_vel话题控制小车运动，控制逻辑为ESP32订阅cmd_vel话题接收线速度与角速度，再通过卡尔曼滤波与PID控制，运动正逆解转化为小车四个电机实际旋转速率。NUC运行ROS2机器人操作系统，接收odom里程计话题，同时运行navigation2导航发布cmd_vel话题，从而控制小车运动进行导航。上位机和下位机采用串口通信，默认通信波特率为115200，默认端口为USB0

项目学习来自于**鱼香ROS**，相关代码引用**鱼香ROS**的开源，ESP32编写采用‌PlatformIO，‌以下是‌PlatformIO的简介：**PlatformIO是一个开源的物联网开发生态系统‌，提供跨平台的代码构建器、集成开发环境（IDE），支持多种硬件平台（如Arduino、ESP8266、STM32等）和开发框架，旨在简化嵌入式系统开发流程并提高效率。‌‌**



## 系统架构概览

```text
lower_computer/                     
├──lib/                    # 存放调用的第三方库文件，包含卡尔曼滤波，PID控制
├──src/                    # 下位机主函数         
└──platformio.ini          # platformio配置文件

upper_computer/            
└──src/                    # 上位机代码
    ├──micro_ros_msgs/     # microros自定义消息接口(未安装，需按照以下步骤安装)
    ├──micro-ROS-Agent/    # 上位机运行的microros客户端，运行后才能与下位机利用microros进行通信（未安装，需按照以下步骤安装）
    ├──mybot_bringup/      # 小车底盘坐标转换
    ├──mybot_description/  # 小车的urdf描述文件
    ├──mybot_navigation2/  # 导航功能包，包含导航参数，导航所需pgm地图
    └──rplidar_ros/        # rplidar雷达的驱动包
```

## 安装说明

### 依赖项安装
进入upper_computer/src/文件夹，运行以下指令
```bash
git clone https://github.com/micro-ROS/micro-ROS-Agent.git -b humble
git clone https://github.com/micro-ROS/micro_ros_msgs.git -b humble
```

### 编译项目

```bash
colcon build
source install/setup.bash
```

### 将下位机代码烧入ESP32

- 将lower_computer的代码通过串口烧入ESP32，若控制引脚不同，可自行在主程序中修改

## 使用方法

### 初始化

- 运行以下代码从而启动上位机控制程序：
```bash
source install/setup.bash
ros2 launch mybot_bringup bringup.launch.py 
```
- 重启ESP32或者按下复位键，令ESP32与上位机重连

### 导航方法

#### 建图
- 打开新终端，运行以下代码,启动建图
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False
```
- 操控小车进行建图，操控方法可见**其他说明第一条**
- 建图完毕之后在终端进入`upper_computer/src/mybot_navigation2/maps`目录运行以下代码保存建图结果,`room`为保存的地图的名称，可自行修改，若修改名称，需再自行去`upper_computer/src/mybot_navigation2/launch/navigation2.launch.py`中修改加载的地图名称
```bash
ros2 run nav2_map_server map_saver_cli  -f room
```

#### 运行导航

- 在上位机与下位机控制程序均运行且已建立联系的情况下，新建终端运行以下代码
```bash
ros2 launch mybot_navigation2 navigation2.launch.py use_sim_time:=False
```
- 运行以上代码后，会打开rviz2终端，选择`2D Pose Estimate`工具为小车初始化位姿
- 接着使用`Nav2 Goal`工具就可以设置目标的进行导航了

## 其他说明

- 因为小车下位机ESP32订阅了cmd_vel话题，所以在上位机上运行以下代码可以进行遥控，做一个遥控车
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
- 自己的电脑如何与小车上位机进行通信，在此提供三个方法：`SSH远程终端进行操控，适合远程遥控小车运动`，`远程桌面与小车上位机进行连接操控`，`直接下个第三方的远程操控软件`
- 若需调整导航参数，请在`upper_computer/src/mybot_navigation2/config/nav2_params.yaml`中进行调整
- 建图完成后，地图文件（`.pgm` 和 `.yaml`）将保存在当前工作目录下，yaml存储着pgm地图路径，因此仅需在`navigation2`的`launch`启动文件中修改地图yaml文件的名称与路径即可


## 鸣谢
- 感谢鱼香ROS大佬的教学视频与开源代码
 [鱼香ROS的B站主页](https://space.bilibili.com/1940177928?spm_id_from=333.337.0.0)
 [鱼香ROS的开源及教学代码](https://gitee.com/ohhuo/ros2bookcode)
- 感谢实验室学长的支持
- 感谢众多开源作者的技术分享与支持


