# 环境配置
挂上梯子，然后参见https://gazebosim.org/docs/harmonic/install_ubuntu/

# 如何编译
首先编译控制器
将src/Parameters.cpp里第10行的motion_hub_param_path改成自己的config文件夹路径
cd src/gazebo_
cmake -B build
cmake --build build

# 如何运行
找到build文件夹
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=build文件夹(`pwd`/build/motion_controller)

```
