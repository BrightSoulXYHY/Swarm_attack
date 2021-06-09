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
### Win端

将`_Swarm_Attack`目录下的东西拷贝到Rflysim主机上

```
_mav_reboot.py					让无人机重启的脚本，pyserial需要在3.5以上

硬件在环的仿真脚本，只是修改了飞机数目啥的
拷贝到另外的飞机上要修改start_index
oldfactory_HITL.bat
oldfactory_HITL_NUCx6.bat
oldfactory_HITL_NXx2.bat

软件在环的一些脚本
oldfactory_SITL.bat
oldfactory_SITL_bs.bat
SITL_swam_oldfactor.bat


Rflysim的库，大部分是通过UDP封包发送，可以自己搞
PX4MavCtrlV6.py
RflyVisionAPI.py
ScreenCapApiV4.py

参考用的脚本
client_ue4_broadcast.py
Python38Run.bat
```

然后在`_Swarm_Attack_bs`下面主要是自己写的一些脚本

```
multi_img_client.py				设置窗口然后向不同的硬件发送图片
multi_img_client_NX.py

基本的API
RflyVisionAPI2_bs.py
ScreenCapApiV4_bs.py
```

