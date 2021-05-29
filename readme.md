# Swarm_Attack

## 系统结构

- bs_assis
  从mavros读取飞控信息并且通过DDS进行转发出去
  同时从DDS接收自己和从局域网其他NUC发送过来的无人机信息
  DDS接收到信息推到ros中，话题名称为

  ```
  /drone_xx/mavros/local_position/pose
  /drone_xx/mavros/local_position/velocity_local
  ```

- decision

  对于单架飞机而言

  启动了`multi_drone.py`运行`src\decision\scenarios`目录下的剧本

  通过`origin_bias.py`订阅`/drone_xx/mavros/local_position/pose`

  推送出相对于首架飞机的位置`/drone_%s/mavros/local_position/pose_cor`

- rflysim_ros_pkg

  rflysim的ros包，提供了从rflysim获取图像和向rflysim发送指令的API

- visualization

  调用rflysim_ros_pkg发送一些控制的指令控制显示管道之类的

- rflysimindoorcontroller_r2018b_n12_v9

- matlab_gen

- MatabNode

  这三个文件夹大概是从matlab生成的cpp文件，MatabNode有弄好的二进制文件，直接运行，暂时不用改

- attack

- perception

  两个预留的包

## 配置

1. 创建工作空间
``` 
mkdir ~/Swarm_ws
cd ~/Swarm_ws
```

2. 下载本仓库及子模块
```
git clone https://github.com/BrightSoulXYHY/Swarm_attack.git src
```

3. 编译
```
cd ~/Swarm_ws
catkin_make
```

4. 刷新ROS环境变量
```
# USER_NAME替换为自己的用户名，用zsh的是...../setup.zsh
source /home/USER_NAME/Swarm_ws/devel/setup.bash
# 或者把这句话加在~/.bashrc（或者~/.zshrc），然后重开一个终端，就不用每次都执行上面这句了
```

5. 替换RflySim配置文件
在Windows电脑下，把`rflysim-config`文件夹内容拷到PX4PSP对应路径下。其中`oldfactory_HITL.bat`和`oldfactory_SITL.bat`是硬件在环和软件在环仿真快速启动脚本，`client_ue4_broadcast.py`是加入小球和传图脚本。根据RflySim说明文档，修改其中的IP地址。




## 使用
1. 在Windows电脑下运行`oldfactory_SITL.bat`或者`oldfactory_HITL.bat`。加入小球、传图需要运行`client_ue4_broadcast.py`。其中传图需要把`client_ue4_broadcast.py`内注释打开，对电脑性能要求高，待测试。
```
若Linux环境为VMware虚拟机，需要在Windows电脑下将需要将oldfactory_SITL.bat第39行修改为
SET IS_BROADCAST=IP_Virtual
其中IP_Virtual为虚拟机ip地址
虚拟机网络适配器需采用 桥接模式（自动），选中 复制物理网络连接状态
```

2. 在Linux下运行。接收各种需要的消息
```
# 终端1。其中IP为Windows电脑的IP地址
roslaunch mavros px4.launch fcu_url:="udp://:20101@IP:20100"

# 终端2[可选]。运行后得到图像话题，可以后续操作
roslaunch rflysim_ros_pkg cameras.launch
```

3. 打开MATLAB和Simulink，开始Simulink仿真。可以在Windows或Linux电脑，注意需要自行判断ROSCORE的IP。
```
rosinit("IP_Virtual")
```
4. 在QgroundControl中手动解锁、起飞

5. 执行任务
```
# 终端3。注意需要为offboard.py添加可执行权限
rosrun decision offboard.py
```

6. 聚焦第5步弹出窗口，按a键进入offboard模式，执行第一个航路点。




## 开发
```
# 开发前获取最新版本
git pull
# 可选，更新子模块
git submodule update
# 开发过程中，可以回到~/Swarm_ws/src目录下，提交代码 
git add .
git commit -m "say something"
# 开发完成后，上传代码
git push
```
## 多机
1.更新Rflyim配件
```
在windows电脑下，把rflysim-config文件夹下的“SITL_swam_oldfactor.bat”和“PX4MavCtrlV6.py”
文件拷贝到PX4PSP对应路徑下，“SITL_swam_oldfactor.bat”为多机软件再换启动脚本（场景为废旧工厂），
在windows电脑下双击，然后输入集群飞机数量；“PX4MavCtrlV6.py”为Rflysim最新Python接口，
“sendUE4PosScale(self,copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,Scale=[1,1,1],windowID=-1)”
可改变添加物体（例如小球、油桶）尺寸大小，其他接口与原版相同。
注：使用上诉接口需更新最新版的Rflysim3D软件和Copterim软件
```

2.启动多机Mavros
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
如需添加更多架飞机，在launch文件中依次添加下列代码，其中“<arg name="ID" value="i"/>”为添加的ID编号，
这一项不能重名，“<arg name="fcu_url" default="udp://:20101@$(arg IP):20100"/>”中的
20101和20100不同飞机端口号，也不能重复

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

3.终端2：启动坐标系系校准程序
```
roslaunch decision multi_drone.launch
现象：
可订阅“/drone_i/mavros/local_position/pose_cor”节点消息，drone_i为相对应的飞机编号，
此节点为矫正后（统一坐标系后的local_position坐标），为Point32类型。
修改：
如果添加更多项飞机，则需要修改所有的下列项
1）“<arg name="drone_id" value="1"/>”中的“value="1"”以此编号，与步骤2中的飞机编号相对应
2）“<arg name="drone_num" value="3"/>”中的“value="3"”要改成添加的飞机数量，例如飞机数量时8，则需修改为8
```

```
注意：
如果不需要飞机自动飞行，屏蔽launch文件中的这些项（目前一共三项），这些启动的是测试时所用的飞机控制程序
   <include file="$(find decision)/launch/drone.launch">
        <arg name="drone_id" value="1"/>
        <arg name="drone_num" value="3"/>
   </include>

保留下列项即可：
   <include file="$(find decision)/launch/bias.launch">
        <arg name="drone_id" value="1"/>
        <arg name="drone_num" value="3"/>
   </include>
```

## 一键启动
1.在windows电脑下启动.bat文件

2.Linux电脑下启动
```
# GitHub最新版本，第一次使用时先输入下列指令：
roscd main/shell/
chmod +x 12_drone.sh

# 之后启动只需运行一个终端，依次运行如下指令：
roscd main/shell/
./12_drone.sh
```
