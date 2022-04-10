### 建立起始工作空间

- `mkdir -p ~/catkin_ws/src`
- `cd ~/catkin_ws`
- `catkin_make`初始化, 没有任何源代码的进行编译
- `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`
- `source ~/.bashrc`
- `roscd` 查看环境变量是否添加
- `cd ~/catkin_ws`

### 建立july_say的package

- `cd ~/catkin_ws/src`
- `catkin_create_pkg july_say std_msgs roscpp`
- `cd july_say/src`
- `vim july_say_node.cpp`
- `cd ..`
- `vim CMakeLists.txt`
- `cd ~/catkin_ws`
- `catkin_make`编译
- `rosrun july_say july_say_node`运行
    - 此时,因为没有启动ros master节点,所以还无法运行
