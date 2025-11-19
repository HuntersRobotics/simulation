# Add Software Source
```bash
cat << 'EOF' | sudo tee /etc/apt/sources.list.d/kaylordut.list 
deb [signed-by=/etc/apt/keyrings/kaylor-keyring.gpg] http://apt.kaylordut.cn/kaylordut/ kaylordut main
EOF
cat << 'EOF' | sudo tee /etc/apt/preferences.d/kaylordut
Package: *
Pin: release o=kaylordut kaylordut,a=kaylordut,n=kaylordut,l=kaylordut kaylordut,c=main,b=arm64
Pin-Priority: 1099
EOF
sudo mkdir /etc/apt/keyrings -pv
sudo wget -O /etc/apt/keyrings/kaylor-keyring.gpg http://apt.kaylordut.cn/kaylor-keyring.gpg
sudo apt update
```

# Install Mid360 Software Package
```
sudo apt install liblivox-sdk2-dev ros-humble-livox-ros-driver2 ros-humble-ros2-livox-simulation
sudo apt install ros-humble-teleop-twist-keyboard
```


# Compile and Run 

## the 1st terminal
```bash
cd ~
mkdir hunter
cd hunter
git clone https://github.com/HuntersRobotics/simulation.git --depth 1
cd simulation
colcon build
source install/setup.bash # If your shell is Bash
source install/setup.zsh # If your shell is Zsh
ros2 launch hunter_world simulation.launch.py # Load gazebo simulation environment in the current terminal
```

## the 2nd terminal
```bash
./restart_robot_diff.sh # create a diff robot in another terminal
```
## the 3rd terminal
```bash
rviz2 -d rviz/mid360.rviz # Open rviz2 with mid360.rviz configuration
```

## the 4th terminal
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```