//
// Created by zjudancer on 18-10-29.
// E-mail: zjufanwu@zju.edu.cn
// Edited by handsome on 24-4-2
// E-mail: 
//
#pragma once
#include <vector>
#include <memory>
#include "Parameters.h"
#include "Utility/dmotion_math.hpp"


namespace dmotion {

    class InvKin {

    public:
        // 这些是单个腿上的关节值
        // 这些关节值都以机器人初值基础为零点,按照惯用旋转方向为正方向给出数值的
        double hip_yaw_,hip_roll_,hip_pitch_;
        double knee_pitch_,ankle_pitch_,ankle_roll_;
        // 是左腿还是右腿
        bool isRight_;

    private:
        std::shared_ptr<Parameters> parameters;
        // 以下都是一些计算过程中需要使用到的参数,不需要理解
        double foot_vertical_x,foot_vertical_y,foot_vertical_z;
        double ankle_x_to_hip;
        double ankle_y_to_hip;
        double ankle_z_to_hip;
        double ankle_norm;
        double ankle_axis_x;
        double ankle_axis_y;
        double ankle_axis_z;
        double vertical_x;
        double vertical_z;
        double vertical_unitz;
        double hip_yaw_delta;
        double ankle_to_hip_yaw_roll_x;
        double ankle_to_hip_yaw_roll_z;
        double hip_pitch_absolute;
        double foot_hip_rpy_x;
        double foot_hip_rpy_y;
        double foot_hip_rpy_z;
        double ankle_pitch_absolute;
        std::vector<double> finals;


    public:
        // 静态常成员变量便于调用,这些参数比较固定,机器人装好后一般绝对不会变,故没有写在参数文件中
        // 存在几何改动时到InverseKinematics.cpp中改动
        // 参数的单位是cm
        // 构造函数简单的构造一下就可以
        InvKin(bool isRight,std::shared_ptr<Parameters> param_node);


        // 逆运动学,以身体中心为原点,前x左y上z的方向建立坐标系,输入左/右脚的位置(x,y,z)和姿态(r,p,y)
        // 为了方便理解和调参,这里的RPY是ZYX欧拉角
        // 输出从上到下的共6个舵机角度值(角度值).
        std::vector<double> LegInvKin(std::vector<double> foot_pose);


    };


}
