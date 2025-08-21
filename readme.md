# 安装支持mid360仿真
```
sudo apt install liblivox-sdk2-dev ros-humble-livox-ros-driver2 ros-humble-ros2-livox-simulation
```

# 加载world环境
```bash
ros2 launch hunter_world simulation.launch.py world:=WAREHOUSE # 小环境
ros2 launch hunter_world simulation.launch.py world:=RMUL # 小环境
ros2 launch hunter_world simulation.launch.py world:=RMUC # 大环境
```

# 加载和删除机器人

```bash
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity '{name: "robot"}'
```

# Rviz启动
```bash
rviz2 -d rviz/mid360.rviz 
```