AgileX Scout ROS 使用手册
一、简介
本手册旨在指导用户如何在 AgileX Scout Mini 机器人上使用 ROS 进行激光雷达（Mid360Lidar）数据处理、建图、导航以及 SLAM（同时定位与建图）等功能的启动和操作。
二、环境准备
该系统已安装了 ROS（ ROS noetic 版本）以及 AgileX Scout 机器人的相关驱动和软件包。同时，检查机器人硬件连接正常，激光雷达等传感器能够正常工作。
三、操作步骤

（一）网络与设备配置
1.电脑配置
  用户名：nvidia
  密码:nvidia
  本地ip：192.168.1.4
  雷达ip：192.168.1.200
2.路由器配置
   wifi:agilex-2266
   路由器IP：192.168.1.1
   管理员和wifi密码：12345678
   路由器WiFi密码：12345678

3.相机检查
   相机APP：sudo realsense-viewer

备注：以上参数出厂已配置好

（二）底盘通信与功能启动

1.使能地盘can0通信
cd ~/agilex_ws/src/scout_ros/scout_bringup/scripts
./setup_can2usb.bash 
功能：该命令用于启动底盘的 CAN0 通信端口映射，确保能够正常接收和发送控制指令。
candump can0
检查can0数据流

2.启动雷达导航功能
2.1 建图
 roslaunch scout_bringup open_MID360lidar.launch 
 
功能：该命令用于启动激光雷达驱动节点，使激光雷达开始工作并发布激光雷达数据到 ROS 的话题中。激光雷达数据是后续建图和导航等功能的基础数据来源。
roslaunch scout_bringup gmapping.launch
功能：此命令启动 gmapping 建图节点，它会根据激光雷达数据实时构建地图。在建图过程中，用遥控器控制机器人移动，以便收集更多的环境信息来完善地图。
2.2 保存地图
cd ~/agilex_ws/src/scout_ros/scout_description/maps
rosrun map_server map_saver -f map
功能：当建图完成到满意程度后，使用此命令将地图保存到指定的路径下。map 是保存的地图文件名(默认使用地图文件名“map”)。
2.3 启动导航
roslaunch scout_bringup navigation_4wd.launch 

3.相机使用
启动
roslaunch realsense2_camera rs_camera.launch 
查看图像
rqt_image_view

4.imu使用(/dev/ttyUSB0)
sudo chmod 777 /dev/ttyUSB0
roslaunch imu_launch imu_msg.launch
查看imu
rostopic echo /imu/data_raw 


四、常见问题和解决：
1、如果雷达启动失败，检查连接，再重新运行：
roslaunch scout_bringup open_rslidar.launch 

2、如果运行导航 navigation_4wd.launch 时rviz可视化未正常出现雷达数据和代价地图等，
请检查地盘can0通信是否正常，按操作（二）的“1.使能地盘can0通信”步骤执行。

3、如果新建地图使用其他文件名，需要修改~/agilex_ws/src/scout_ros/scout_bringup/launch/
navigation_4wd.launch的：            
    <node name="map_server" pkg="map_server" type="map_server" args="$(find scout_description)/maps/map.yaml" output="screen">
的'map.yaml'代码替换成新建的文件名，并对工程包重新编译

4、使用时务必把雷达帽移走

5、imu的usb端口已固定绑定，调换插口会连接失败





