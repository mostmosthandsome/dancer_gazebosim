# 环境配置
挂上梯子，然后参见https://gazebosim.org/docs/harmonic/install_ubuntu/

# 如何编译
首先编译控制器
将src/Parameters.cpp里第10行的motion_hub_param_path改成自己的config文件夹路径
cd src/gazebo_
cmake -B build
cmake --build build

# 如何运行
```bash
cd到sim文件夹下
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build/motion_controller
. install/setup.bash 
ros2 launch gazebo_env env.launch.py 

```

## 其他
![alt text](<截图 2024-08-29 11-10-01.png>)
场地如图所示，横向为x轴，纵向为y轴，圆点位于场地正中央