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
https://gitlab.com/libeigen/eigen
参见(https://mbd.baidu.com/ug_share/mbox/4a83aa9e65/share?product=smartapp&tk=6bb662b2a445a97d4121ed7126c4c5ab&share_url=https%3A%2F%2Fyebd1h.smartapps.cn%2Fpages%2Fblog%2Findex%3FblogId%3D137562851%26_swebfr%3D1%26_swebFromHost%3Dbaiduboxapp&domain=mbd.baidu.com)


# 如何编译
首先编译控制器  

将src/Parameters.cpp里第10行的config_path改成自己的config文件夹路径  

输入
```bash
colcon build
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



