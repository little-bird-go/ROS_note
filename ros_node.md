### `ROS`学习记录

#### ROS安装

安装过程参照[ROS官网](https://wiki.ros.org)上的步骤，主要有以下4个步骤：

1. 设置sources.list
2. 设置密钥
3. 命令行安装
4. 设置环境

#### 创建catkin工作空间

```bash
ws_folder/				-- WORKSPACE
	src/				-- SOURCE SPACE
	  CMakeLists.txt
	  package_1/
	    CMakeLists.txt
	    package.xml
	    ...
	  package_n/
	  	...
	build/				-- BUILD SPACE
	  ...
	devel/				-- DEVELOPMENT SPACE
	  bin/
	  etc/
	  include/
	  lib/
	  share/
	  setup.bash
	  ...
	install/			-- INSTALL SPACE
	  bin/
	  etc/
	  include/
	  lib/
	  share/
	  ...
```

一个典型的ROS工作空间由四个部分组成，源文件空间，编译空间，开发空间和安装空间，每个都是一个单独的文件夹。需要重点关注的是源文件空间，里面是各种包的源文件。

创建一个工作空间的步骤如下：

1. 创建工作空间文件夹以及源文件空间文件夹
2. 使用`catkin_make`生成ROS工作空间

```bash
mkdir -p ws_folder/src
cd ws_folder
catkin_make
```

#### 创建软件包

一个典型的软件包如上述的工作空间文件构成，一定包含有`CMakeList.txt`和`package.xml`两个文件。其中`package.xml`中包含了软件包的版本，作者，开源协议以及依赖关系等信息。

创建一个软件包的步骤如下：

1. 转到`src`目录下
2. 使用`catkin_creat_pkg`命令生成一个新包

```bash
cd ws_folder/src
catkin_creat_pkg example_pkg std_msgs rospy roscpp
```

生成的软件包下文件结构如下：

```bash
example_pkg/
	CMakeLists.txt
	package.xml
	src/
	include/
```



#### 编写并运行hello软件包

编写自定义的软件包有以下几个步骤：

1. 在生成的软件包文件夹下的`src`文件夹下编写程序

2. 修改`CMakeLists.txt`文件

3. 在工作空间下使用`catkin_make`进行编译

4. 设置环境（如有设置可忽略）

   ```bash
   source ./devel/setup.bash
   ```

   

编写程序示例：

```cpp
#include "ros/ros.h"

int main(int arg, char * argv[])
{
    ros::init(arg, argv, "hello_world");	// 最后一个参数为节点名称
    ros::NodeHandle n;
    ROS_INFO("hello world!");				// 在终端中打印信息

    return 0;
}
```

#### ROS通信机制

 ros中的通信机制有三种：

+ topic话题通信
+ server服务通信
+ parameter参数通信

##### 话题通信实现

###### 发布者

实现流程为：

1. 初始化`ros::init`
2. 定义节点
3. 创建发布话题
4. 在循环体中填充话题内容并发布

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "publisher");
    ros::NodeHandle nh;
    ros::Publisher pb = nh.advertise<std_msgs::String>("Hello", 10);
    ros::Rate r(1);
    int count = 0;

    while (ros::ok())
    {
        /* code */
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello --- " << count++;
        msg.data = ss.str();
        pb.publish(msg);
        ROS_INFO("pub: %s", msg.data.c_str());
        r.sleep();
        ros::spinOnce();
    }
    
    return 0;
}
```

###### 订阅者

订阅流程如下：

1. 初始化`ros::init`
2. 定义节点
3. 订阅话题
4. 循环接受话题并在回调函数中处理话题内容

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "string"

void ss_callback(const std_msgs::StringConstPtr &msg)
{
    ROS_INFO("sub: %s", msg->data.c_str());
}
int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle nh;
    ros::Subscriber ss = nh.subscribe<std_msgs::String>("Hello", 10, ss_callback);
    ros::spin();
    return 0;
}
```

###### 注意

在话题通信的编程实现中可以使用以下两个工具

1. `rostopic`命令，可以查看发布的`topic`内容
2. `rqt_graph`工具包，可视化订阅方，发布方和话题信息

##### 自定义消息与使用

在`std_msgs`的基础之上，可以自定义不同的消息类型来方便数据的通信。流程如下：

1. 在包文件夹下新建`msg`文件夹，并新建`message_example.msg`文件
2. 在新建的`message_example.msg`文件中定义所需的数据类型如下所示

```cpp
int8 a
uint16 b
float32 c
string d
...
```

3. 修改`package.xml`和`CMakeList.txt`文件
4. 编译生成中间文件
5. 使用时包含生成的自定义消息头文件即可

###### 两个配置文件的修改

+ `package.xml`

```xml
<!-- 编译时需要 -->
<build_depend>message_generation</build_depend>
<!-- 运行时需要 -->
<exec_depend>message_runtime</exec_depend>
```

+ `CMakeLists.txt`

```cmake
# 增加 message_generation (编译需要)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# 增加自定义话题文件
add_message_files(
  FILES
  selfDef.msg
)

# 添加依赖
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 打开第三行注释，增加 message_runtime (运行需要)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES selfDefMsg
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

# 项目编译需要
add_dependencies(sfDfP ${PROJECT_NAME}_generate_messages_cpp)
```

###### 发布方实现

```cpp
#include "ros/ros.h"
#include "selfDefMsg/selfDef.h"


int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "selfDefMsfP");
    ros::NodeHandle nh;
    ros::Publisher pb = nh.advertise<selfDefMsg::selfDef>("Place", 10);
    ros::Rate r(0.5);
    
    while(ros::ok()){
        selfDefMsg::selfDef msg;
        msg.placeName = "BeiJing";
        msg.height = 12.5;
        msg.depth = 0.56;

        pb.publish(msg);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
```

###### 订阅方实现

```cpp
#include "ros/ros.h"
#include "selfDefMsg/selfDef.h"

void msgCallback(const selfDefMsg::selfDef::ConstPtr &msg)
{
    ROS_INFO("PLACENAME: %s, HEIGHT: %.2f, DEPTH: %.2f", msg->placeName.c_str(), msg->height, msg->depth);
}

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "selfDefMsfS");
    ros::NodeHandle nh;
    ros::Subscriber ss = nh.subscribe<selfDefMsg::selfDef>("Place", 10, msgCallback);
    ros::spin();
    return 0;
}
```

##### 服务通信实现

###### 自定义服务消息

1. 在包文件夹下`src`下建立`srv`文件夹和`server.srv`文件
2. 在`server.srv`中编写自定义的消息(以`---`作为请求和响应数据之间的分割)

```cpp
int32 num1
int32 num2
---
float32 div
```

3. 修改`package.xml`和`CMakeList.txt`两个配置文件
4. 调用生成的中间文件来实现服务通信

###### 具体实现

+ 服务端

```cpp
#include "ros/ros.h"
#include "comb_serv_clnt/srv_message.h"

bool srvCallback(comb_serv_clnt::srv_message::Request &req,
                comb_serv_clnt::srv_message::Response &res)
{
    int num1 = req.num1;
    int num2 = req.num2;

    if(num2 != 0){
        res.div = 1.0 * num1 / num2;
    }
    else{
        ROS_ERROR("The divided number could not be zero!");
        return false;
    }
    return true;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "myserver");
    ros::NodeHandle nh;
    ros::ServiceServer ss = nh.advertiseService("Divide", srvCallback);
    ROS_INFO("Server start!");
    ros::spin();
    return 0;
}
```

+ 客户端

```cpp
#include "ros/ros.h"
#include "comb_serv_clnt/srv_message.h"

int main(int argc, char *argv[])
{
    /* code */
    if(argc != 3){
        ROS_ERROR("The params number error!");
        return 1;
    }

    ros::init(argc, argv, "myclient");
    ros::NodeHandle nh;
    ros::ServiceClient sc = nh.serviceClient
    <comb_serv_clnt::srv_message::Request, comb_serv_clnt::srv_message::Response>("Divide");
    // wait for the server start.
    sc.waitForExistence();
    // or use ros::service::waitForService("divide");

    comb_serv_clnt::srv_message msg;
    msg.request.num1 = atoi(argv[1]);
    msg.request.num2 = atoi(argv[2]);
    
    bool flag = sc.call(msg);
    if(flag){
        ROS_INFO("The requestment is responed!");
        ROS_INFO("The result is %.2f", msg.response.div);
    }
    else{
        ROS_ERROR("The requestment failed!");
        return 1;
    }
    return 0;
}
```

+ 配置文件修改

  `package.xml`

  ```xml
  <!-- 编译时需要 -->
  <build_depend>message_generation</build_depend>
  <!-- 运行时需要 -->
  <exec_depend>message_runtime</exec_depend>
  ```

  `CMakeLists.txt`

  ```cmake
  # 增加 message_generation (编译需要)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
  )
  
  # 增加自定义文件
  add_service_files(
    FILES
    srv_message.srv
  )
  
  # 添加依赖
  generate_messages(
    DEPENDENCIES
    std_msgs
  )
  
  # 打开第三行注释，增加 message_runtime (运行需要)
  catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES selfDefMsg
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  #  DEPENDS system_lib
  )
  
  # 项目编译需要
  add_dependencies(sfDfP ${PROJECT_NAME}_gencpp)
  ```

##### 参数服务器实现

###### 参数设置和修改实现

```cpp
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "set_param");
    ros::NodeHandle nh;

    int int_param = 3;
    double double_param = 3.14;
    bool boolean_param = true;
    std::string string_param = "hello";
    std::vector<int> vec_param;
    vec_param.push_back(1);
    vec_param.push_back(2);
    vec_param.push_back(3);

    std::map<std::string, std::string> map_param;
    map_param["Wang"] = "Xinwei";
    map_param["Luo"] = "Siyu";
    map_param["Chen"] = "Long";
    
    nh.setParam("int_param", int_param);
    nh.setParam("double_param", double_param);
    nh.setParam("boolean_param", boolean_param);
    nh.setParam("string_param", string_param);
    nh.setParam("vec_param", vec_param);
    nh.setParam("map_param", map_param);

    // The second method to set parameters.

    // ros::param::set("int_param", int_param);
    // ros::param::set("double_param", double_param);
    // ros::param::set("boolean_param", boolean_param);
    // ros::param::set("string_param", string_param);
    // ros::param::set("vec_param", vec_param);
    // ros::param::set("map_param", map_param);
    
    return 0;
}
```

###### 参数获取实现

```cpp
#include "ros/ros.h"
/*
NodeHandle method:
Some API to get parameters.

1. param() return the value when the param exists, return the default value otherwise.
2. getParam() return true when the param exists, and save the value to the var put in; otherwise return false.
3. getParamCached() the same with getParam(), but has higher effectiency.
4. getParamNames() return all the names of parameters, and save in a vector put in.
5. hasParam() return true when the param exist, otherwise false.
6. searchParam() search param using the key(name) of the param, and save the result in the var(string type) put in.

*/
int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "get_param");
    ros::NodeHandle nh;

    int int_param_v;
    double double_param_v;
    bool boolean_param_v;
    std::string string_param_v;
    std::vector<int> vec_param_v;
    std::map<std::string, std::string> map_param_v;
    
    bool f = nh.getParam("int_param", int_param_v);
    if(f){
        ROS_INFO("The int param value is: %d", int_param_v);
    }

    f = nh.getParam("double_param", double_param_v);
    if (f)
    {
        ROS_INFO("The double param value is: %.2f", double_param_v);
    }

    f = nh.getParam("boolean_param", boolean_param_v);
    if (f)
    {
        ROS_INFO("The boolean param value is: %d", boolean_param_v);
    }

    f = nh.getParam("string_param", string_param_v);
    if (f)
    {
        ROS_INFO("The string param value is %s", string_param_v.c_str());
    }

    f = nh.getParam("vec_param", vec_param_v);
    if(f)
    {
        for(auto v : vec_param_v){
            ROS_INFO("The vector param: %d", v);
        }
    }

    f = nh.getParam("map_param", map_param_v);
    if (f)
    {
        for(auto it : map_param_v){
            ROS_INFO("The map param: %s %s", it.first.c_str(), it.second.c_str());
        }
    }
    
    // The second method to get parameters.

    // ros::param::get("int_param", int_param_v);
    // ros::param::getCached("int_param", int_param_v);
    // std::vector<std::string> v;
    // ros::param::getParamNames(v);
    // ros::param::has("int_param");
    // std::string result_;
    // ros::param::search("int_param", result_);

    
    return 0;
}
```

###### 参数删除实现

```cpp
#include "ros/ros.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "del_param");
    ros::NodeHandle nh;

    if(nh.deleteParam("int_param")){
        ROS_INFO("Delete int_param successful!");
    };
    if(nh.deleteParam("double_param")){
        ROS_INFO("Delete double_param successful!");
    };
    if(nh.deleteParam("boolean_param")){
        ROS_INFO("Delete boolean_param successful!");
    };
    if(nh.deleteParam("string_param")){
        ROS_INFO("Delete string_param successful!");
    };

    if(ros::param::del("vec_param")){
        ROS_INFO("Delete vec_param successful!");
    }
    if(ros::param::del("map_param")){
        ROS_INFO("Delete map_param successful!");
    }

    return 0;
}
```

##### 常用的命令

###### `rosnode`

```bash
rosnode ping	测试到节点的连接状态
rosnode list	列出活动节点
rosnode info	打印节点信息
rosnode machine	列出指定设备上的节点
rosnode kill	杀死某个节点
rosnode cleanup	清除无用节点
```

###### `rostopic`

```bash
rostopic echo   打印消息到屏幕
rostopic find   根据类型查找主题
rostopic hz     显示主题的发布频率
rostopic info   显示主题相关信息
rostopic list   显示所有活动状态下的主题
rostopic pub    将数据发布到主题
rostopic type   打印主题类型
```

###### `rosservice`

```bash
rosservice args		打印服务参数
rosservice call		使用提供的参数调用服务
rosservice find		按照服务类型查找服务
rosservice info		打印有关服务的信息
rosservice list		列出所有活动的服务
rosservice type		打印服务类型
rosservice uri		打印服务的 ROSRPC uri
```

###### `rosmsg`

```bash
rosmsg show			显示消息描述
rosmsg info			显示消息信息
rosmsg list			列出所有消息
rosmsg package		显示某个功能包下的所有消息
rosmsg packages		列出包含消息的功能包
```

###### `rossrv`

```bash
rossrv show			显示消息描述
rossrv info			显示消息信息
rossrv list			列出所有消息
rossrv package		显示某个功能包下的所有消息
rossrv packages		列出包含消息的功能包
```

###### `rosparam`

```bash
rosparam set		设置参数
rosparam get		获取参数
rosparam load		从外部文件加载参数
rosparam dump		将参数写出到外部文件
rosparam delete		删除参数
rosparam list		列出所有参数
```

#### 使用`roslaunch`来发布节点

##### 命令格式

```bash
roslaunch packageName lauchFileName.launch
// for example:
roslauch mySubscriber start.launch
```

##### `launch`文件格式（实际上为一个`xml`文件）

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node"     name="myTurtle" output="screen" />
    <node pkg="turtlesim" type="turtle_teleop_key"  name="myTurtleContro" output="screen" />
</launch>
```

##### `launch`文件标签

+ `<launch>`

  是launch文件的根标签，只有一个属性`deprecated`

  ```xml
  <launch deprecated="This file is no use!">
  </launch>
  ```

+ `<node>`

  用于指定**ROS**节点，多个节点发布不能依赖声明的顺序来启动

  ```xml
  <!--node 标签中常用的属性有 
  	pkg		包名
  	type	节点类型（可执行文件）
  	name 	节点名称
  	respawn	是否重启
  	respawn_delay	是否重启延时
  	required	是否必须
  	output	日志输出
  -->
  <launch>
  	<node pkg="turtlesim" type="turtlesim_node" name="myturtle" respawn="true" respawn_delay="10" required="true" output="screen" />
  </launch>
  ```

+ `<remap>`

  `<node>`子标签，用于话题重命名

  ```xml
  <launch>
      <node pkg="turtlesim" type="turtlesim_teleop_key"     name="myTurtleKey" output="screen">
          <remap from="/turtle1/cmd_vel" to="/cmd_vel"
      </node>
  </launch>
  ```

+ 其他一些标签

  参阅[赵虚左老师教程](http://www.autolabor.com.cn/book/ROSTutorials/)

#### 常用APIs

在编程实现中有一些常用的API，可以参考以下两个网站

+ http://wiki.ros.org/APIs
+ https://docs.ros.org/en/api/roscpp/html

具体为以下几类函数：

1. 初始化`ros::init()`
2. 通信相关（参考上面的实现中函数）
3. 回旋函数`spin()`和`spinOnce()`
4. 日志输出
   1. `ROS_DEBUG()`
   2. `ROS_INFO()`
   3. `ROS_WARN()`
   4. `ROS_ERROR()`
   5. `ROS_FATAL()`
5. 时间相关

```cpp
// 1. 时刻 ros::Time
ros::init(argc, argv, "ros_time_func");
ros::NodeHandle nh;
ros::Time right_now = ros::Time::now();		// 获取当前时刻
ROS_INFO("Time right now: %.2f", right_now.toSec());		// 距离1970年1月1日0时 的秒数,有小数
ROS_INFO("Time right now: %d", right_now.sec());			// 整数
ros::Time sometime(100, 100000000);			// 参数1：s， 参数2：ns
ros::Time sometime(100.5);

// 2. 一段时间
ros::Duration du(10);
du.sleep();

// 3. 时刻与时间之间可以相加减

// 4. 频率
ros::Rate r(2);
r.sleep();			// 放在循环体内

// 5. 定时器
// 使用定时器之间必须创建节点句柄，这一步中会对相关配置进行初始化

// 函数原型
// Timer createTimer(Duration period, const TimerCallback& callback, bool oneshot = false, bool autostart = true) const;

/*
参数1： 时间间隔    
参数2： 回调函数
参数3： 是否只执行一次
参数4： 是否自动开启
*/

ros::TImer timer = nh.createTimer(ros::Duration(0.5), timerCallBack, false, true);
timer.start() // 手动开启定时器
ros::spin();
```

#### 自定义头文件和源文件调用

##### 在`include/packageName/`下添加自定义头文件

```cpp
// myhead.h
#pragma once
#include <string>

// 注意要自定命名空间，否则容易混淆
namespace myns{
    class myclass{
    private:
        int age;
        std::string name;
    public:
        myclass();
        ~myclass();
        int getAge();
        void setAge(int age);
        std::string getName();
        void setName(std::string name);
    };
};
```

##### 在`src/`下添加自定义源文件

```cpp
// myhead.cpp
#include "use_head_src/myhead.h"

using namespace myns;
myclass::myclass()
{
    age = 23;
    name = "wxw";
}

myclass::~myclass(){}

void myclass::setAge(int age)
{
    this->age = age;
}

int myclass::getAge()
{
    return age;
}

void myclass::setName(std::string name)
{
    this->name = name;
}

std::string myclass::getName()
{
    return name;
}
```

##### 同一目录下添加主程序文件

```cpp
// use_it.cpp
#include "ros/ros.h"
#include "use_head_src/myhead.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "use_head");
    myns::myclass me = myns::myclass();
    int age = me.getAge();
    std::string name = me.getName();
    ROS_INFO("The age is %d, the name is %s", age, name.c_str());
    return 0;
}
```

##### 修改配置文件

```cmake
# 1
## Declare a C++ library
add_library(mylib
  include/use_head_src/myhead.h
  src/myhead.cpp
)

# 2
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(mylib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 3
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(mytry src/use_it.cpp)

# 4
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(mytry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 5
target_link_libraries(mytry
  mylib 
  ${catkin_LIBRARIES}
)
```

#### ROS元功能包

将多个不同的功能包打包，构建成一个新的功能包的集合，该功能包称为**元功能包**(**metapackage**)。

##### 新建一个功能包

##### 修改`package.xml`文件

```xml
<buildtool_depend>catkin</buildtool_depend>
<!-- 添加要集成的功能包 -->
<exec_depend>comb_pub_sub</exec_depend>
<exec_depend>comb_serv_clnt</exec_depend>
<exec_depend>param_set_get</exec_depend>

<!-- The export tag contains other, unspecified, tags -->
<export>
    <!-- Other tools can request additional information be placed here -->
    <metapackage />
</export>
```

##### 修改`CMakeLists.txt`文件

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(mymetapackage)
find_package(catkin REQUIRED)
catkin_metapackage()
```

##### 编译使用

#### TF2功能包使用

##### 坐标`msg`消息

+ `geometry_msgs/TransformStamped`

  ```bash
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string child_frame_id
  geometry_msgs/Transform transform
    geometry_msgs/Vector3 translation
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion rotation
      float64 x
      float64 y
      float64 z
      float64 w
  ```

+ `geometry_msgs/PointStamped`

  ```bash
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Point point
    float64 x
    float64 y
    float64 z
  ```

##### 静态坐标变换

利用`TF2`功能包来实现不同坐标系下的坐标变换，依赖的功能包有`tf2`,`tf2_ros`,`tf2_geometry_msgs`,`roscpp`,`rospy`,`std_msgs`和`geometry_msgs`

###### 发布方程序

```cpp
#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "static_tf_node");
    ros::NodeHandle nh;

    // 需要头文件 tf2_ros/static_transform_broadcaster.h
    tf2_ros::StaticTransformBroadcaster s_tf_pub;
    // 需要头文件 geometry_msgs/TransformStamped.h
    geometry_msgs::TransformStamped tfs;
    
    // 定义一个具体的坐标变换
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = "base_link";      // 参考坐标
    tfs.child_frame_id = "laser";           // 本坐标
    // 平移变换
    tfs.transform.translation.x = 0.2;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.5;
    
    // 使用欧拉角变换成四元数
    // 需要头文件 tf2/LinearMath/Quaternion.h
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, 0);
    // 旋转变换
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();

    s_tf_pub.sendTransform(tfs);
    ros::spin();
    return 0;
}
```

###### 订阅方程序

```cpp
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "static_ft_node_");
    ros::NodeHandle nh;

    // 下面两者并用，来实现坐标变换话题的订阅
    // 需要头文件 tf2_ros/buffer.h
    tf2_ros::Buffer buffer;
    // 需要头文件 tf2_ros/transform_listener.h
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(1);
    while(ros::ok())
    {
        // 模拟从雷达得到的位置数据
        geometry_msgs::PointStamped ps_laser;
        ps_laser.header.frame_id = "laser";
        ps_laser.header.stamp = ros::Time::now();
        ps_laser.point.x = 5;
        ps_laser.point.y = 1;
        ps_laser.point.z = 2;

        geometry_msgs::PointStamped ps_out;
        // 使用 try 模块防止程序运行开始还没有传入坐标变换的消息
        try
        {
            // 内置函数实现坐标的变换
            ps_out = buffer.transform(ps_laser, "base_link");
            ROS_INFO("The transformed point: (%.2f, %.2f, %.2f). The reference frame: %s.", 
                        ps_out.point.x, 
                        ps_out.point.y, 
                        ps_out.point.z, 
                        ps_out.header.frame_id.c_str());
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        r.sleep();
        ros::spinOnce();

    }
    return 0;
}
```

###### 可视化结果

通过`rviz`工具可以实现不同坐标之间关系的可视化结果。

###### 官方静态坐标变换节点功能包

推荐使用官方的功能包，可配合`launch`文件来实现，这样更简洁

```bash
## 使用方式
rosrun tf2_ros static_transform_publisher x y z yaw pitch roll ref_frame self_frame

## 示例
rosrun tf2_ros static_transform_publisher 0.2 0 0.3 0 0 0 /base_link /camera
```

##### 动态坐标变换

###### 发布方程序

```cpp
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

void callbackfn(const turtlesim::Pose::ConstPtr &pose)
{
    static tf2_ros::TransformBroadcaster pub;
    geometry_msgs::TransformStamped d_tf;
    d_tf.header.frame_id = "world";
    d_tf.header.stamp = ros::Time::now();
    d_tf.child_frame_id = "turtle";
    d_tf.transform.translation.x = pose->x;
    d_tf.transform.translation.y = pose->y;
    d_tf.transform.translation.z = 0;

    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, pose->theta);

    d_tf.transform.rotation.x = qtn.getX();
    d_tf.transform.rotation.y = qtn.getY();
    d_tf.transform.rotation.z = qtn.getZ();
    d_tf.transform.rotation.w = qtn.getW();

    pub.sendTransform(d_tf);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dynamic_tf_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<turtlesim::Pose> \
                            ("turtle1/pose", 10, callbackfn);
    ros::spin();
    return 0;
}
```

###### 订阅方程序

```cpp
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dynamic_tf_sub");
    ros::NodeHandle nh;

    // 下面两者并用，来实现坐标变换话题的订阅
    // 需要头文件 tf2_ros/buffer.h
    tf2_ros::Buffer buffer;
    // 需要头文件 tf2_ros/transform_listener.h
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(1);
    while(ros::ok())
    {
        // 模拟从雷达得到的位置数据
        geometry_msgs::PointStamped ps_laser;
        ps_laser.header.frame_id = "turtle";
        ps_laser.header.stamp = ros::Time(); // 不能使用now()
        ps_laser.point.x = 5;
        ps_laser.point.y = 1;
        ps_laser.point.z = 2;

        geometry_msgs::PointStamped ps_out;
        // 使用 try 模块防止程序运行开始还没有传入坐标变换的消息
        try
        {
            // 内置函数实现坐标的变换
            ps_out = buffer.transform(ps_laser, "world");
            ROS_INFO("The transformed point: (%.2f, %.2f, %.2f). The reference frame: %s.", 
                        ps_out.point.x, 
                        ps_out.point.y, 
                        ps_out.point.z, 
                        ps_out.header.frame_id.c_str());
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        r.sleep();
        ros::spinOnce();

    }
    return 0;
}
```

##### 多坐标变换

已知`frame1`与`base_frame`，`frame2`与`base_frame`之间的坐标关系，求`frame2`中一点在`frame1`中的坐标。

主要涉及的函数有：

```cpp
lookupTransform()
transform()
```

###### 发布方

发布frame2下点坐标

```cpp
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "point_pub");
    if(argc != 4){
        ROS_INFO("the number of params error");
        return -1;
    }

    double x = atoi(argv[1]);
    double y = atoi(argv[2]);
    double z = atoi(argv[3]);

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("point", 10);
    geometry_msgs::PointStamped point;
    point.header.frame_id = "frame1";
    point.point.x = x;
    point.point.y = y;
    point.point.z = z;
    
    ros::Rate r(1);
    while(ros::ok()){
        point.header.stamp = ros::Time::now();
        pub.publish(point);
        ROS_INFO("pub a point once");
        r.sleep();
    }

    return 0;
}
```

###### 接收方

接收点坐标并进行坐标转换。

```cpp
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

void doPoint(const geometry_msgs::PointStamped::ConstPtr &point)
{
    static int i = 0;
    // ros::NodeHandle nh;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    
    // 注意要休眠一段时间，否则会找不到坐标系的相关信息
    ros::Duration du(1);
    du.sleep();
    try
    {
        /* code */
        geometry_msgs::TransformStamped tfs = buffer.lookupTransform("frame2", "frame1", ros::Time(0));
        if (i == 0)
        {
            ROS_INFO("the transform from frame1 to frame2: ");
            ROS_INFO("the base frame is: %s", tfs.header.frame_id.c_str());
            ROS_INFO("the child frame is: %s", tfs.child_frame_id.c_str());
            ROS_INFO("translation: x: %.2f, y: %.2f, z: %.2f", tfs.transform.translation.x, tfs.transform.translation.y, tfs.transform.translation.z);
            ROS_INFO("rotation: x: %.2f, y: %.2f, z: %.2f, w: %.2f", tfs.transform.rotation.x, tfs.transform.rotation.y, tfs.transform.rotation.z, tfs.transform.rotation.w);
            ros::Duration du(0.5);
            du.sleep();
            i++;
        }
        geometry_msgs::PointStamped pin = *point;
        geometry_msgs::PointStamped pout;
        pout = buffer.transform(pin,"frame2");
        ROS_INFO("the base frame is: %s", pout.header.frame_id.c_str());
        ROS_INFO("point: x: %.2f, y: %.2f, z: %.2f", pout.point.x, pout.point.y, pout.point.z);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "multi_tf_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<geometry_msgs::PointStamped>("point", 10, doPoint);
    ros::spin();
    return 0;
}
```

###### 脚本发布

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="myturtle" output="screen" />
    <node pkg="turtlesim" type="turtle_teleop_key" name="teleop_key" output="screen" />
    <node pkg="dynamic_tf" type="pub" name="tf_pub" output="screen" />
    <node pkg="dynamic_tf" type="sub" name="tf_sub" output="screen" />
</launch>
```

##### 坐标关系查看

###### 下载工具包

```bash
sudo apt install ros-noetic-tf2-tools
```

###### 使用

```bash
rosrun tf2_tools view_frames.py
```

即会在当前目录下生成两个文件，一个是`frames.gv`，另一个是`frames.pdf`。坐标关系图绘制在`PDF`文件中。

##### turtle跟随案例

程序启动，在窗口中生成两只乌龟，一只为系统自动生成，另一只通过服务发布来生成。通过键盘功能包实现对系统自动生成乌龟的控制，而通过服务发布的乌龟能够跟随运动。

###### 案例分析

1. 启动乌龟显示节点，键盘控制节点
2. 通过服务生成第二只乌龟
3. 编写发布方节点，能够发布两只乌龟相对窗口的坐标
4. 编写订阅方节点，订阅两只乌龟坐标信息，通过坐标变换得到相对位置关系，发布速度信息

###### `launch`文件

```xml
<launch>
    <!-- 发布 turtlesim GUI -->
    <node pkg="turtlesim" type="turtlesim_node" name="turtle1" output="screen"/>
    <!-- 发布 键盘控制节点 -->
    <node pkg="turtlesim" type="turtle_teleop_key" name="key" output="screen"/>
    <!-- 发布第二只小乌龟 -->
    <node pkg="trace_turtle" type="pub_turtle" name="turtle2" output="screen"/>
    <!-- 发布两只乌龟的坐标信息 -->
    <node pkg="trace_turtle" type="pub_turtleframe" name="frame1" args="turtle1" output="screen"/>
    <node pkg="trace_turtle" type="pub_turtleframe" name="frame2" args="turtle2" output="screen"/>
    <!-- 发布第二只乌龟的速度信息， 使其跟随 第一只乌龟i -->
    <node pkg="trace_turtle" type="trace" name="trace_final" output="screen"/>
    
</launch>
```

###### 生成乌龟服务

```cpp
#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "turtle_pub");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");

    ros::service::waitForService("/spawn");

    turtlesim::Spawn turtle;
    turtle.request.x = 1;
    turtle.request.y = 1;
    turtle.request.theta = 1.57;
    turtle.request.name = "turtle2";

    bool flag = client.call(turtle);
    if (flag)
    {
        ROS_INFO("A new turtle create.");
    }
    else{
        ROS_INFO("A new turtle create failed.");
        return 1;
    }
    
    return 0;
}
```

###### 发布乌龟坐标

```cpp
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

std::string frame;

void callbackfn(const turtlesim::Pose::ConstPtr &pose)
{
    static tf2_ros::TransformBroadcaster pub;
    geometry_msgs::TransformStamped d_tf;
    d_tf.header.frame_id = "world";
    d_tf.header.stamp = ros::Time::now();
    d_tf.child_frame_id = frame;
    d_tf.transform.translation.x = pose->x;
    d_tf.transform.translation.y = pose->y;
    d_tf.transform.translation.z = 0;

    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, pose->theta);

    d_tf.transform.rotation.x = qtn.getX();
    d_tf.transform.rotation.y = qtn.getY();
    d_tf.transform.rotation.z = qtn.getZ();
    d_tf.transform.rotation.w = qtn.getW();

    pub.sendTransform(d_tf);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dynamic_tf_node");
    ros::NodeHandle nh;

    if (argc != 2)
    {
        ROS_INFO("param number error.");
        return 1;
    }
    frame = argv[1];  // 获取坐标信息

    ros::Subscriber sub = nh.subscribe<turtlesim::Pose> \
                            (frame + "/pose", 10, callbackfn);
    ros::spin();
    return 0;
}
```

###### 跟踪运动

```cpp
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trace");
    ros::NodeHandle nh;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listen(buffer);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    geometry_msgs::Twist twist;
    geometry_msgs::TransformStamped trans;
    ros::Rate r(3);
    ros::Duration du(1);
    du.sleep();
    double x;
    double y;
    double theta;
    int i = 0;

    while(ros::ok())
    {
        
        try
        {
            trans = buffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
            if (i == 0)
            {
                ROS_INFO("the base frame: %s", trans.header.frame_id.c_str());
                ROS_INFO("the child frame: %s", trans.child_frame_id.c_str());
                i++;
            }
            x = trans.transform.translation.x;
            y = trans.transform.translation.y;
            ROS_INFO("x: %.2f, y: %.2f, z: %.2f", 
                    trans.transform.translation.x, 
                    trans.transform.translation.y, 
                    trans.transform.translation.z);

            ROS_INFO("x: %.2f, y: %.2f, z: %.2f, w: %.2f",
                    trans.transform.rotation.x, 
                    trans.transform.rotation.y, 
                    trans.transform.rotation.z,
                    trans.transform.rotation.w);

            twist.linear.x = sqrt(x*x + y*y);
            twist.angular.z = atan2(y, x);
            pub.publish(twist);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        r.sleep();
        ros::spinOnce();

    }
    return 0;
}
```

##### `tf`和`tf2`比较

- `TF2`已经替换了`TF`，`TF2`是TF的超集，建议学习 `TF2` 而非 `TF`
- `TF2` 功能包的增强了内聚性，`TF` 与 `TF2` 所依赖的功能包是不同的，`TF` 对应的是`tf`包，`TF2` 对应的是`tf2`和`tf2_ros`包，在 `TF2` 中不同类型的 `API` 实现做了分包处理。
- `TF2` 实现效率更高，比如`TF2` 的静态坐标实现、`TF2` 坐标变换监听器中的 `Buffer` 实现等

#### `rosbag`使用

##### 命令行使用

###### 创建文件夹保存录制文件

```bash
mkdir ./bags
```

###### 开始录制

```bash
rosbag record -a -o ./bags/hello.bag
```

###### 查看文件

```bash
rosbag info hello.bag
```

###### 回放文件

```bash
rosbag play hello.bag
```

##### 自定义编程使用

###### 录制

```cpp

```

###### 回放

```cpp

```











