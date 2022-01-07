# Swarm_Attack

本项目演示了无人机集群硬件在环打击废旧工厂内的目标，[演示视频](https://hexo-blog-bs.oss-cn-beijing.aliyuncs.com/Rflysim-MAVx8-20210730.mp4)。

## 快速上手

本项目的架构如下，局域网内有多台运行Rflysim的主机，PX4飞控和其连接组成硬件在环仿真。板载计算单元为NX/NUC，运行Linux。控制程序基于ROS，通过串口和飞控通信并控制无人机完成任务。

**有关Rflysim的接口参见Rflysim的[文档](https://rflysim.com/docs/#/)和Rflysim安装目录下的RflySimAPIs文件夹内的例程。**

![分布式Rflysim.jpg](http://ww1.sinaimg.cn/large/008eYjoEgy1gsxt0jeodej60r10g0wg102.jpg)

### 下载代码

代码基于ros开发，`_Swarm_Attack`文件夹内的脚本是在Rflysim主机上运行的脚本。

- 在NX/NUC上的操作

```
mkdir ~/Swarm_ws
cd ~/Swarm_ws
git clone https://github.com/BrightSoulXYHY/Swarm_attack.git src
catkin build
```

- Rflysim主机上的操作

拷贝`_Swarm_Attack`文件夹至`%PSP_PATH%\RflySimAPIs`

### 运行demo

详细步骤参见`测试用例的使用手册.docx`

- Rflysim主机上的操作

一台主机连接三个飞控，双击运行`oldfactory_HITLRunCom.bat`，飞控会重启，等待串口号稳定输入串口号（飞控重启后串口号可能会跳动，以设备管理器的串口号为准）

![HITL启动.jpg](http://ww1.sinaimg.cn/large/008eYjoEgy1gsxto2whlsj60lp0fwtbs02.jpg)

等待Rflysim、CopterSim等程序启动，多台电脑联合仿真步骤同上。

切换至NX/NUC运行一键启动脚本，推荐使用ssh

```
cd ~/Swarm_ws
src/run.h
```

## 系统结构

- bs_assis
  从mavros读取飞控信息并且通过DDS进行转发出去
  同时从DDS接收自己和从局域网其他NUC发送过来的无人机信息
  DDS接收到信息推到ros中，话题名称为

  ```
  /drone_xx/mavros/local_position/pose
  /drone_xx/mavros/local_position/velocity_local
  ```

- rflysim_ros_pkg

  rflysim的ros包，提供了从rflysim获取图像和向rflysim发送指令的API

  ![HITL启动.jpg](http://ww1.sinaimg.cn/large/008eYjoEgy1gsxuh0vcqjj60s70hgab202.jpg)

- decision

  对于单架飞机而言

  启动了`multi_drone.py`运行`src\decision\scenarios`目录下的剧本

  通过`origin_bias.py`订阅`/drone_xx/mavros/local_position/pose`

  推送出相对于首架飞机的位置`/drone_%s/mavros/local_position/pose_cor`

- visualization

  调用rflysim_ros_pkg发送一些控制的指令控制显示管道之类的

- rflysimindoorcontroller_r2018b_n12_v9/matlab_gen/MatabNode

  上述三个文件夹是从matlab生成的cpp文件，话题大致如下

  ![](http://ww1.sinaimg.cn/large/008eYjoEgy1gsxuh0xgj1j61hc0rttd502.jpg)


## 其他
### Rflysim主机相关的说明

需要在启动脚本`oldfactory_HITLRunCom.bat`中指定无人机总数和起始序号

```
SET /a START_INDEX=%COMPUTERNAME:~-1%*3-2
SET /A TOTOAL_COPTER=10
```

如果需要使用视觉方面的识别，运行`img_client_multi.py`发送至NX/NUC，需要指定无人机总数和ip列表

```
max_mav = 10
ipList = [
    f"192.168.111.{i+20+1}" for i in range(max_mav)
]
```

### NX/NUC相关的说明

一键运行脚本需要在`~/Swarm_ws`目录下运行

飞控序号要和NX/NUC的hostname一致，参数来自与hostname

```
MAVID=`expr ${HOSTNAME:4:2} + 0`
```

### 飞控和NUC设置序号的方法

参见`飞控的ID设置.docx`和`NX的ID设置.doc`

## 项目依赖的环境

- [ROS](https://www.ros.org/)
- [Fast-DDS](https://fast-dds.docs.eprosima.com/en/latest/)