## 使用说明
1. 创建工作空间
``` 
mkdir ~/RflySim_ws
cd ~/RflySim_ws
```

2. 下载本仓库
```
git clone https://github.com/KennethYangle/RflySim_ws.git
mv RflySim_ws src
```

3. 下载压缩插件
```
# 其中kinetic根据自己ROS发行版替换，可能是melodic或indigo等等
sudo apt-get install ros-kinetic-compressed-image-transport
```

4. 编译
```
catkin_make
```

5. 刷新ROS环境变量
```
# USER_NAME替换为自己的用户名，用zsh的是...../setup.zsh
source /home/USER_NAME/RflySim_ws/devel/setup.bash
# 或者把这句话加在~/.bashrc（或者~/.zshrc），然后重开一个终端，就不用每次都执行上面这句了
```

6. 运行单目或双目图像
```
# 确保仿真环境正在运行，向本机发送数据。单目或双目运行其中一个就好
# 单目
roslaunch rflysim_ros_pkg cameras.launch
# 双目
roslaunch rflysim_ros_pkg stereo.launch
```

7. 其他节点接收消息。图像话题包括原始图像/camera/left和压缩图像/camera/left/compressed，根据需要自行定义。可以使用下面语句快速查看。
```
rqt_image_view /camera/left/compressed
# 或者用本仓库提供的接收脚本。
# 第一次使用前添加可执行权限
roscd examples/scripts/
chmod +x imgread.py imgread_compressed.py
# 接收压缩图片话题
rosrun examples imgread_compressed.py
# 接收原始图片话题
rosrun examples imgread.py
```

## 多机
1.启动多机Mavros
```
#终端1：启动多机Mavros的launch文件
roslaunch rflysim_ros_pkg multi_mavros.launch
现象：
之前的/mavros......开头的节点名都会变成/drone_i/mavros......开头，因此订阅的消息都要做相应修改
修改项：
<arg name="IP" value="192.168.1.167" />中的“192.168.1.167”
需要改成对应window电脑的IP地址
```
```
注意：
如需添加更多架飞机，在launch文件中依次添加下列代码，其中“<arg name="ID" value="i"/>”为添加的ID编号，这一项不能重名，“<arg name="fcu_url" default="udp://:20101@$(arg IP):20100"/>”中的20101和20100不同飞机端口号，也不能重复

   <group ns="drone_i">
      <arg name="ID" value="i"/>
      <arg name="fcu_url" default="udp://:20101@192.168.199.140:20100"/>
      <!-- MAVROS -->
      <include file="$(find mavros)/launch/px4.launch">
         <arg name="fcu_url" value="$(arg fcu_url)"/>
         <arg name="gcs_url" value=""/>
         <arg name="tgt_system" value="$(arg ID)"/>
         <arg name="tgt_component" value="1"/>
      </include>
   </group>
```
