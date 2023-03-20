# bezier_local_planner

## 目录

* [安装依赖](#1-安装依赖)
* [使用benchmark](#2-使用benchmark)
* [定量评估](#3-定量评估)

## 1. 安装依赖
我们假设您已经创建了名为catkin_ws的工作空间并完成了初始化。如果您的ROS版本是melodic，可直接用二进制安装navigation包
```
sudo apt install ros-melodic-navigation
```
如果您的ROS版本是kinetic，请先从GitHub上下载navigation源码到本地，并切换到melodic分支，再编译
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ros-planning/navigation.git
$ git checkout melodic-devel
$ cd ../
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

完成上述准备工作之后，再将该仓库克隆到本地
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/NKU-MobFly-Robotics/bezier_local_planner.git
```

完成BezierLocalPlanner类之后，编译程序
```
$ cd ~/catkin_ws/
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```
P.S. 在BezierLocalPlanner类里面，我们已经帮您定义了一些用于可视化路径的publisher和相应的publish函数，需要您在代码实现的时候调用相应的函数发布路径话题

## 2. 使用benchmark
本次作业我们采用的机器人仿真模型依旧是Pioneer 3-DX，在上一次DWA的作业中我们已经安装过，这里就不再赘述。下面请先下载benchmark源码到本地并编译
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/NKU-MobFly-Robotics/local-planning-benchmark.git
$ cd ../
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

然后打开move_base_benchmark/launch目录下的move_base_benchmark.launch，找到
```
<param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS" />
<rosparam file="$(find move_base_benchmark)/params/teb_local_planner_params.yaml" command="load" />
```
将其替换为：
```
<param name="base_local_planner" value="bezier_local_planner/BezierLocalPlannerROS" />
<rosparam file="$(find bezier_local_planner)/params/bezier_local_planner_params.yaml" command="load" />
```
如果您想使用上一次作业的DWA局部规划算法，仅需修改为：
```
<param name="base_local_planner" value="dwa_planner/DWAPlannerROS" />
<rosparam file="$(find dwa_planner)/params/dwa_planner_params.yaml" command="load" />
```
接着找到
```
<node name="rviz" pkg="rviz" type="rviz" args="-d $(find move_base_benchmark)/launch/rviz_navigation.rviz"/>
```
将其替换为
```
<node name="rviz" pkg="rviz" type="rviz" args="-d $(find bezier_local_planner)/params/rviz.rviz"/>
```

P.S. 修改launch文件以及相关的yaml配置文件后无需编译，运行程序时直接在外部进行参数传递

接下来打开一个新的终端，运行仿真
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch move_base_benchmark move_base_benchmark.launch
```
在弹出的RViz可视化界面里，点击工具栏的2D Nav Goal并在地图中设置终点，即可运行导航程序

您也可以运行move_base_benchmark/launch目录下的simple_navigation_goals.launch来指定导航的终点
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch move_base_benchmark simple_navigation_goals.launch
```

## 3. 定量评估
每次运行move_base_benchmark.launch之后，我们会将导航过程的中间数据打印到move_base_benchmark/launch目录下的log.txt，log.txt每一行记录如下数据：
```
{timestamp, x, y, theta, v, omega, d, c}
```
其中，timestamp为本次局部规划被调用时的时间戳，(x, y, theta)为当前机器人的位姿，(v, omega)为当前机器人的码盘速度，d为当前机器人到最近障碍物的距离，c为本次局部规划的耗时

我们采用如下几个指标对局部规划算法进行评估：

### 3.1 安全性
我们用导航过程中机器人在危险区域内经过的时间与整个导航过程的总耗时的百分比来评估安全性，其中危险区域定义为到机器人本体质心的距离小于机器人外接圆直径的区域

### 3.2 运动效率
我们用机器人完成给定导航任务的时间来评估局部规划算法的运动效率，这里的给定导航任务是指机器人完成固定起点和终点的定点导航

### 3.3 计算效率
我们用导航过程中局部规划算法的平均计算耗时来评估算法的计算效率

### 3.4 平滑性
我们用导航过程中平均线加速度来评估速度平滑性

上述指标具体的计算公式可参考move_base_benchmark/doc目录下的main.pdf。此外，在doc目录下还提供了一个cpp文件metrics.cpp用于计算上述指标。您需要创建一个文件夹，我们暂且将文件夹命名为metric_evaluation
```
mkdir metric_evaluation
```
然后将metrics.cpp和CMakeLists.txt放在metric_evaluation目录下，再编译
```
$ cd metric_evaluation
$ mkdir build
$ cd build/
$ cmake ..
$ make -j4
```
您只需要将log.txt放在metric_evaluation目录下，然后运行程序
```
$ cd build/
$ ./metric_evaluation ../log.txt
```
即可在终端看到指标计算结果

**为了对不同局部规划算法进行公平的对比，每组对比测试需要指定相同的起点和终点**

起点可在move_base_benchmark.launch中设置

```
<arg name="initial_pose_x" default="0.0"/>
<arg name="initial_pose_y" default="0.0"/>
<arg name="initial_pose_a" default="1.5708"/>
```
终点可在simple_navigation_goals.launch中设置
```
<arg name="goal_pose_x" default="9.904"/>
<arg name="goal_pose_y" default="8.226"/>
<arg name="goal_pose_a" default="-3.139"/>
```

**在您的作业报告中，需要给出每组对比测试的起点和终点配置**