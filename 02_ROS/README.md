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

#### july_say_node.cpp

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"july_talker");
    ros::NodeHandle n;
    ros::Publisher july_pub = n.advertise<std_msgs::String>("/july_topic",10);
    ros::Rate loop_rate(10);
    int count = 0;

    while(ros::ok()){
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello july" << count;
        count ++;
        msg.data = ss.str();
        july_pub.publish(msg);
        loop_rate.sleep();
    }

}
```

#### CMakeLists.txt

- `add_executable(${PROJECT_NAME}_node src/july_say_node.cpp)`前的注释去掉
- `add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})`前的注释去掉
- `CATKIN_DEPENDS roscpp std_msgs`前的注释去掉
- `target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})`前的注释去掉

### 启动ROS Master节点

- cd
- roscore

### 检查july_say是否真的发布

- cd
- rostopic list查看topic的列表
- rosnode list查看节点列表
- rostopic echo /july_topic

### 查看july_talker节点的信息,以便编写july_listen

- rosnode info /july_talker查看july_talker节点信息

### 建立july_listen的package

- cd ~/catkin_ws/src
- catkin_create_pkg july_listen std_msgs roscpp
- cd july_listen/src
- vim july_listen_node.cpp
- cd ..
- vim CMakeLists.txt
- cd ~/catkin_ws
- catkin_make
- rosrun july_listen july_listen_node

#### july_listen_node.cpp

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <july_msgs/JulyMsg.h>

void julyCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("I heard %s",msg->data.c_str());
}


int main(int argc, char ** argv){
    ros::init(argc,argv,"july_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("july_topic",10,julyCallback);
    ros::spin();
    return 0;
}
```

#### CMakeLists.txt

- `add_executable(${PROJECT_NAME}_node src/july_say_node.cpp)`前的注释去掉
- `add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})`前的注释去掉
- `CATKIN_DEPENDS roscpp std_msgs`前的注释去掉
- `target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})`前的注释去掉

### 查看july_listen是否获取到july_say里面的数据

- 确保ros master已经开启
    - 回忆上面第三步的roscore命令
- 打开一个新的shell
- `cd ~/catkin_ws`
- `rosrun july_say july_say_node`开启july_say节点
- 打开一个新的shell
- `rosrun july_listen july_listen_node`开启july_listen监听节点

### 建立july_msgs的package

> 目的: 定义自己的message

- `cd ~/catkin_ws/src`
- `catkin_create_pkg july_msgs std_msgs roscpp`
- `cd july_msgs`
- `mkdir msg`
- `cd msg`
- `touch JulyMsg.msg`
- `vim JulyMsg.msg`
- `vim ../CMakeLists.txt`
- `vim ../package.xml`
- `cd ~/catkin_ws`
- `catkin_make`
- `cd devel/include && ls`
    - 此时,catkin在此处添加了一个july_msgs的文件,其内生成了一个JulyMsg.h的头文件,就可以让其他包使用了
- `vim ~/catkin_ws/src/july_say/src/july_say_node.cpp`
- `cd ..`
- `vim CMakeLists.txt`
- `cd ~/catkin_ws`
- `catkin_make`
- `rosrun july_msgs july_msgs_node`

#### JulyMsg.msg

```
string detail
int32 id
```

#### CMakeLists.txt

- `add_message_files(FILES Message1.msg Message2.msg)`去掉注释,且修改为`add_message_files(FILES JulyMsg.msg)`
- `generate_messages(DEPENDENCIES std_msgs)`去掉注释
    - 编译时候自动生成这个msg
- `find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)`修改为`find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)`

#### package.xml

- `<exec_depend>message_runtime</exec_depend>`去掉注释
- `<build_depend>message_generation</build_depend>`去掉注释

### 再次修改july_say的package¶

- `vim ~/catkin_ws/src/july_say/src/july_say_node.cpp`
- `vim ~/catkin_ws/src/july_say/CMakeLists.txt`
- `vim ~/catkin_ws/src/july_say/package.xml`
- `catkin_make`

#### july_say_node.cpp

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "july_msgs/JulyMsg.h"

int main(int argc,char ** argv){
    ros::init(argc,argv,"july_talker");
    ros::NodeHandle n;
    ros::Publisher july_pub = n.advertise<std_msgs::String>("/july_topic",10);
    ros::Publisher july_pub_new = n.advertise<july_msgs::JulyMsg>("/july_topic_new",10);

    ros::Rate loop_rate(10);
    int count = 0;

    while(ros::ok()){
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello july" << count;
        count ++;
        msg.data = ss.str();
        july_pub.publish(msg);

        july_msgs::JulyMsg julyMsg;
        julyMsg.id = count;
        julyMsg.detail = "hello july nes";

        july_pub_new.publish(julyMsg);
        loop_rate.sleep();
    }

}
```

#### CMakeLists.txt

- `find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)` 修改为 `find_package(catkin REQUIRED COMPONENTS roscpp std_msgs july_msgs)`
- `CATKIN_DEPENDS roscpp std_msgs` 修改为 `CATKIN_DEPENDS roscpp std_msgs july_msgs`

#### package.xml

- `<build_depend>july_msgs</build_depend>`添加
- `<exec_depend>july_msgs</exec_depend>`添加

### 从外界传入信息(利用ros的参数中心)

- `vim ~/catkin_ws/src/july_say/src/july_say_node.cpp`
- `cd ~/catkin_ws`
- `catkin_make`
- 确保已经运行了ros master
- `rosrun july_say july_say_node`
- `rosparam set myparam "ok"`
- `rosparam list`
- 再看界面, 回话已经发生改变

#### july_say_node.cpp

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "july_msgs/JulyMsg.h"

int main(int argc,char ** argv){
    ros::init(argc,argv,"july_talker");
    ros::NodeHandle n;
    ros::Publisher july_pub = n.advertise<std_msgs::String>("/july_topic",10);
    ros::Publisher july_pub_new = n.advertise<july_msgs::JulyMsg>("/july_topic_new",10);

    ros::Rate loop_rate(10);
    int count = 0;

    while(ros::ok()){
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello july" << count;
        count ++;
        msg.data = ss.str();
        july_pub.publish(msg);

        std::string param_string;
        n.param<std::string>("myparam", param_string, "hi july");
        july_msgs::JulyMsg julyMsg;
        julyMsg.id = count;
        julyMsg.detail = param_string;

        july_pub_new.publish(julyMsg);
        loop_rate.sleep();
    }

}
```
