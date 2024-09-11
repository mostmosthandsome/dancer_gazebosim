//
// Created by zjudancer on 19-1-19
// E-mail: zjufanwu@zju.edu.cn
//

#ifndef PROJECT_FORWARDKINEMATICS_H
#define PROJECT_FORWARDKINEMATICS_H

#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include "Utility/dmotion_math.hpp"
#include "Parameters.h"

namespace dmotion{
class ForKin{
public:
    ForKin( std::shared_ptr<Parameters> _parameters, const std::vector<double> angles, bool isRight);

    std::vector<double> FK_neck_to_foot(const std::vector<double> angles, bool isRight);
    std::vector<double>  angles_;
    ForKin();

    bool isRight_;

    double x_result;
    double y_result;
    double z_result;
    double roll_result;
    double pitch_result;
    double yaw_result;

    std::vector<double> result_vector;
    Eigen::Isometry3d T;
private:
    double neck_z_from_hip;//之前的注释里写是颈关节点相对于身体中心原点的z，但是具体我还没有仔细看
    double hip_z;//yaw下部至hip舵机轴
    std::vector<double> alpha;
    std::vector<double> a;
    std::vector<double> d;
    std::vector<double> theta;
    std::shared_ptr<Parameters> parameters;

};


class ForKinPlus
{
public:
    //输入为支撑腿相当于规划起点的x,y,z,r,p,y、摆动腿相当于规划起点的x,y,z,r,p,y
    ForKinPlus(std::vector<double> supporting, std::vector<double> hanging);

    Eigen::Isometry3d S;
    Eigen::Isometry3d H;
    //身体中心点相对于支撑脚的x,y,z,r,p,y
    std::vector<double> center2support;
    //摆动脚相对于支撑脚的x,y,z,r,p,y
    std::vector<double> hang2support;

};




//把旋转矩阵转换为x,y,z,r,p,y
std::vector<double> Matrix2Pose(Eigen::Isometry3d M);


}

#endif //PROJECT_FORWARDKINEMATICS_H