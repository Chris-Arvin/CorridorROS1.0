# dwa_planner
该仓库代码已经在安装Ubuntu 18.04/ROS Melodic的电脑上测试通过，如果您的系统是Ubuntu 16.04，请切换分支到kinetic-devel再阅读说明文档。

## 1. 安装依赖

```
sudo apt install ros-melodic-navigation
```
我们假设您已经创建了名为catkin_ws的工作空间并完成了初始化，将该仓库克隆到本地
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/NKU-MobFly-Robotics/dwa_planner.git
```
完成DWAPlanner类之后，编译程序
```
$ cd ~/catkin_ws/
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

## 2. 仿真模型配置
本次作业采用Gazebo来进行仿真测试，刚装好ROS的机器，第一次运行Gazebo的时候，Gazebo会下载模型，导致打开卡死。为解决该问题，请提前先从下面网址下载好模型文件：

https://github.com/osrf/gazebo_models

解压后将整个文件夹命名为models，然后放在.gazebo/目录下，这样再重新打开Gazebo就不会卡在打开界面了。小tips，可以用ctrl+H命令显示隐藏文件

<img src="fig/gazebo.png" width="90%" />
<img src="fig/gazebo_models.png" width="90%" />

另外，我们采用Pioneer 3-DX作为机器人仿真模型，请按照下面网址的说明安装p3dx仿真模型
https://github.com/NKU-MobFly-Robotics/p3dx

## 3. 运行仿真
打开新的终端，source一下工作路径，并运行launch文件
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch dwa_planner move_base.launch
```
终端会有如下图所示的警告和错误提示，这是正常的，可以忽略

<img src="fig/warn.png" width="100%" />
在弹出的RViz可视化界面里，点击工具栏的2D Nav Goal并在地图中设置终点，即可运行导航程序