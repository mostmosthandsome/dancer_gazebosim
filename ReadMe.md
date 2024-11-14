# 环境配置
## OS
建议使用Ubuntu版本>=22.04（20.04好像有个库没有）

## ros2 humble 安装
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html


## gazebo安装
挂上梯子，然后参见https://gazebosim.org/docs/harmonic/install_ubuntu/ 
然后
```bash
sudo apt install python3-colcon-common-extensions
sudo apt-get install ros-humble-ros-gzharmonic
```

## 依赖库安装
### Eigen安装
可以直接用apt安装
```bash
sudo apt install libeigen3-dev
```
也可以从https://gitlab.com/libeigen/eigen下载tar.gz
```
git clone https://gitlab.com/libeigen/eigen.git
mkdir build
cd build
cmake ..
sudo make install

sudo cp -r /usr/local/include/eigen3/Eigen /usr/local/include
```


# 如何编译
首先编译控制器  

将src/Parameters.cpp里第10行的config_path改成自己的config文件夹路径  

输入
```bash
colcon build
```
如果报了类似于如下的错误
```
Could not find a package configuration file provided by "ament_cmake" with
  any of the following names:

    ament_cmakeConfig.cmake
    ament_cmake-config.cmake
```
 回到$home目录下，修改.bashrc文件并加入
 ```bash
source /opt/ros/humble/setup.bash
```

# 如何运行
```bash
cd到dancer_gazebosim文件夹下
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build/motion_controller
. install/setup.bash 
ros2 launch gazebo_env env.launch.py 

```

## 其他
![alt text](<截图 2024-08-29 11-10-01.png>)
场地如图所示，横向为x轴，纵向为y轴，圆点位于场地正中央

## 日后
如果修改了urdf，比如增加了关节数  
一定要记得修改IO.h里的joint_order和Parameters.h里的cur_servo_angles

## 关于sdf
### sdf的属性
http://sdformat.org
### urdf转sdf
gz sdf -p A.urdf > A.sdf

## plugin框架
### 整体框架
Configure, PreUpdate, PostUpdate   
IO.h包含的小函数
### 创建新的sensor
https://github.com/gazebosim/gz-sensors
1231243124




## 爬起动作的代码讲解



