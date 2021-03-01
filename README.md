# 菜鸟项目 -- 新松机器人-视觉TCP ROS2接口

### 启动 Saisun_driver Package

`ros2 launch saisun_hw saisun_hw.launch`

### 获取机器人状态

`ros2 topic echo /saisun_pose`

### 使用接口

> 该Package使用的是action接口，可参考test/vision_server.cpp 编写action，来响应机器人发来的消息。
