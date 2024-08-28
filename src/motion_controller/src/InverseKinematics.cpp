//
// Created by zjudancer on 18-10-29.
// E-mail: zjufanwu@zju.edu.cn
//

#include "InverseKinematics.h"
#include <iostream>
#include <cmath>
#include <vector>
#include "Parameters.h"

using namespace std::chrono_literals;

namespace dmotion {

    InvKin::InvKin(bool isRight,std::shared_ptr<Parameters> parameters_):isRight_(isRight),parameters(parameters_) {
        // 初始化几个基本的变量
        hip_yaw_ = hip_roll_ = hip_pitch_ = 0;
        knee_pitch_ = ankle_pitch_ = ankle_roll_ = 0;
        // 初始化私有变量都为零
        foot_vertical_x = foot_vertical_y = foot_vertical_z = 0;
        ankle_x_to_hip = ankle_y_to_hip = ankle_z_to_hip = 0;
        ankle_norm = ankle_axis_x = ankle_axis_y = ankle_axis_z = 0;
        vertical_x = vertical_z = 0;
        vertical_unitz = hip_yaw_delta = 0;
        ankle_to_hip_yaw_roll_x = ankle_to_hip_yaw_roll_z = 0;
        hip_pitch_absolute = 0;
        foot_hip_rpy_x = foot_hip_rpy_y = foot_hip_rpy_z = 0;
        ankle_pitch_absolute = 0;
        finals.clear();
    }

    /**
     * 求解脚相对身体中心点的逆运行学
     * @param foot_pose 脚底的x y z r p y
     * @return
     */
    std::vector<double> InvKin::LegInvKin(std::vector<double> foot_pose) {
        /** 下面三个坐标是脚踝点相对于髋关节点的坐标 **/
        foot_pose[3] = Deg2Rad(foot_pose[3]);
        foot_pose[4] = dmotion::Deg2Rad(foot_pose[4]);
        foot_pose[5] = dmotion::Deg2Rad(foot_pose[5]);

        foot_vertical_x = -(parameters->one_foot_landing_param.ANKLE_FROM_GROUND *
                            (std::sin(foot_pose[3]) *
                             std::sin(foot_pose[5]) +
                             std::cos(foot_pose[3]) *
                             std::cos(foot_pose[5]) *
                             std::sin(foot_pose[4])));//s1s3+c1c3s2
        foot_vertical_y = -(-parameters->one_foot_landing_param.ANKLE_FROM_GROUND *
                            (std::cos(foot_pose[5]) *
                             std::sin(foot_pose[3]) -
                             std::cos(foot_pose[3]) *
                             std::sin(foot_pose[4]) *
                             std::sin(foot_pose[5])));
        foot_vertical_z = -(parameters->one_foot_landing_param.ANKLE_FROM_GROUND * std::cos(foot_pose[4]) *
                            std::cos(foot_pose[3]));

        ankle_x_to_hip = foot_pose[0] - foot_vertical_x - parameters->one_foot_landing_param.HIP_X_FROM_ORIGIN;
        ankle_y_to_hip = foot_pose[1] - foot_vertical_y - (isRight_ ? (-parameters->one_foot_landing_param.HALF_HIP_WIDTH) : parameters->one_foot_landing_param.HALF_HIP_WIDTH);
        ankle_z_to_hip = foot_pose[2] - foot_vertical_z + parameters->one_foot_landing_param.HIP_Z_FROM_ORIGIN;
//        std::cout << "x : " << ankle_x_to_hip << std::endl;
//        std::cout << "y : " << ankle_y_to_hip << std::endl;
//        std::cout << "z : " << ankle_z_to_hip << std::endl;
        ankle_norm = std::sqrt(
                ankle_x_to_hip * ankle_x_to_hip + ankle_y_to_hip * ankle_y_to_hip + ankle_z_to_hip * ankle_z_to_hip);
        /** 获得了knee_pitch的角度 **/
        knee_pitch_ = M_PI - dmotion::CosineTheorem(parameters->one_foot_landing_param.UPPER_LEG_LENGTH, parameters->one_foot_landing_param.LOWER_LEG_LENGTH, ankle_norm);
        /** 这里计算脚踝roll关节的转轴的矢量 **/
        ankle_axis_x = std::cos(foot_pose[4]) * std::cos(foot_pose[5]);
        ankle_axis_y = std::cos(foot_pose[4]) * std::sin(foot_pose[5]);
        ankle_axis_z = -std::sin(foot_pose[4]);
        /** 求ankle_axis和ankle_to_hip的公垂线向量,默认y方向是1**/
        vertical_x = (ankle_z_to_hip * ankle_axis_y - ankle_y_to_hip * ankle_axis_z) /
                     (ankle_x_to_hip * ankle_axis_z - ankle_axis_x * ankle_z_to_hip);
        vertical_z = (ankle_x_to_hip * ankle_axis_y - ankle_axis_x * ankle_y_to_hip) /
                     (ankle_z_to_hip * ankle_axis_x - ankle_x_to_hip * ankle_axis_z);
        /** 这里获得这个公垂线与水平面的夹角 **/
        vertical_unitz = dmotion::GetDelta(0, 0, 1, vertical_x, 1, vertical_z);
        /** 获得了hip_roll的角度 **/
        hip_roll_ = isRight_ ? (vertical_unitz - M_PI / 2) : (M_PI / 2 - vertical_unitz);
        /** 获得了hip_yaw的角度 **/
        hip_yaw_delta = dmotion::GetDelta(0, 1, vertical_x, 1);
        if (vertical_x > 0)
            hip_yaw_ = isRight_ ? hip_yaw_delta : (-hip_yaw_delta);
        else
            hip_yaw_ = isRight_ ? (-hip_yaw_delta) : hip_yaw_delta;

        
        /**分左右腿获得hip_pitch的角度**/
        if (!isRight_) {
            /** 先得到ankle_to_hip在ankle只经过yaw和roll变换之后的坐标系中的坐标 **/
            ankle_to_hip_yaw_roll_x = ankle_x_to_hip * std::cos(hip_yaw_) + ankle_y_to_hip * std::sin(hip_yaw_);
            /** 因为估计此时的y分量应该为0，所以不算ankle_to_hip_yaw_roll_y **/

            ankle_to_hip_yaw_roll_z =
                    ankle_z_to_hip * std::cos(hip_roll_) - ankle_y_to_hip * std::cos(hip_yaw_) * std::sin(hip_roll_) +
                    ankle_x_to_hip * std::sin(hip_roll_) * std::sin(hip_yaw_);
        } else if (isRight_) {
            ankle_to_hip_yaw_roll_x = ankle_x_to_hip * std::cos(-hip_yaw_) + ankle_y_to_hip * std::sin(-hip_yaw_);
            /** 因为估计此时的y分量应该为0，所以不算ankle_to_hip_yaw_roll_y **/

            ankle_to_hip_yaw_roll_z = ankle_z_to_hip * std::cos(-hip_roll_) -
                                      ankle_y_to_hip * std::cos(-hip_yaw_) * std::sin(-hip_roll_) +
                                      ankle_x_to_hip * std::sin(-hip_roll_) * std::sin(-hip_yaw_);
        }
        /** 获得只经过yaw和roll变换后的-z_unit向量和上面这个向量的夹角
         * 获得hip_pitch_**/
        hip_pitch_absolute = std::atan2(ankle_to_hip_yaw_roll_x, (-ankle_to_hip_yaw_roll_z));
        hip_pitch_ = hip_pitch_absolute + dmotion::CosineTheorem(parameters->one_foot_landing_param.UPPER_LEG_LENGTH, ankle_norm, parameters->one_foot_landing_param.LOWER_LEG_LENGTH);
        /** 这里获得脚底向下的向量在经过hip的ypr变换之后的坐标系中的坐标**/
        if (!isRight_) {
            foot_hip_rpy_x = foot_vertical_x * std::cos(-hip_pitch_absolute) * std::cos(hip_yaw_) -
                             foot_vertical_z * std::cos(hip_roll_) * std::sin(-hip_pitch_absolute) +
                             foot_vertical_y * std::cos(-hip_pitch_absolute) * std::sin(hip_yaw_) +
                             foot_vertical_y * std::cos(hip_yaw_) * std::sin(-hip_pitch_absolute) *
                             std::sin(hip_roll_) -
                             foot_vertical_x * std::sin(-hip_pitch_absolute) * std::sin(hip_roll_) *
                             std::sin(hip_yaw_);
            foot_hip_rpy_y =
                    foot_vertical_z * std::sin(hip_roll_) + foot_vertical_y * std::cos(hip_roll_) * std::cos(hip_yaw_) -
                    foot_vertical_x * std::cos(hip_roll_) * std::sin(hip_yaw_);
            foot_hip_rpy_z = foot_vertical_z * std::cos(-hip_pitch_absolute) * std::cos(hip_roll_) +
                             foot_vertical_x * std::cos(hip_yaw_) * std::sin(-hip_pitch_absolute) +
                             foot_vertical_y * std::sin(-hip_pitch_absolute) * std::sin(hip_yaw_) -
                             foot_vertical_y * std::cos(-hip_pitch_absolute) * std::cos(hip_yaw_) *
                             std::sin(hip_roll_) +
                             foot_vertical_x * std::cos(-hip_pitch_absolute) * std::sin(hip_roll_) *
                             std::sin(hip_yaw_);

            ankle_pitch_absolute = std::atan2(foot_hip_rpy_x, -foot_hip_rpy_z);

            ankle_pitch_ =
                    ankle_pitch_absolute + dmotion::CosineTheorem(parameters->one_foot_landing_param.LOWER_LEG_LENGTH, ankle_norm, parameters->one_foot_landing_param.UPPER_LEG_LENGTH);

            ankle_roll_ = std::atan2(
                    -foot_hip_rpy_y, std::sqrt(foot_hip_rpy_x * foot_hip_rpy_x + foot_hip_rpy_z * foot_hip_rpy_z));

        } else if (isRight_) {
            foot_hip_rpy_x = foot_vertical_x * std::cos(-hip_pitch_absolute) * std::cos(-hip_yaw_) -
                             foot_vertical_z * std::cos(-hip_roll_) * std::sin(-hip_pitch_absolute) +
                             foot_vertical_y * std::cos(-hip_pitch_absolute) * std::sin(-hip_yaw_) +
                             foot_vertical_y * std::cos(-hip_yaw_) * std::sin(-hip_pitch_absolute) *
                             std::sin(-hip_roll_) -
                             foot_vertical_x * std::sin(-hip_pitch_absolute) * std::sin(-hip_roll_) *
                             std::sin(-hip_yaw_);
            foot_hip_rpy_y =
                    foot_vertical_z * std::sin(-hip_roll_) +
                    foot_vertical_y * std::cos(-hip_roll_) * std::cos(-hip_yaw_) -
                    foot_vertical_x * std::cos(-hip_roll_) * std::sin(-hip_yaw_);
            foot_hip_rpy_z = foot_vertical_z * std::cos(-hip_pitch_absolute) * std::cos(-hip_roll_) +
                             foot_vertical_x * std::cos(-hip_yaw_) * std::sin(-hip_pitch_absolute) +
                             foot_vertical_y * std::sin(-hip_pitch_absolute) * std::sin(-hip_yaw_) -
                             foot_vertical_y * std::cos(-hip_pitch_absolute) * std::cos(-hip_yaw_) *
                             std::sin(-hip_roll_) +
                             foot_vertical_x * std::cos(-hip_pitch_absolute) * std::sin(-hip_roll_) *
                             std::sin(-hip_yaw_);
            ankle_pitch_absolute = std::atan2(foot_hip_rpy_x, -foot_hip_rpy_z);

            ankle_pitch_ =
                    ankle_pitch_absolute + dmotion::CosineTheorem(parameters->one_foot_landing_param.LOWER_LEG_LENGTH, ankle_norm, parameters->one_foot_landing_param.UPPER_LEG_LENGTH);

            ankle_roll_ = std::atan2(
                    foot_hip_rpy_y, std::sqrt(foot_hip_rpy_x * foot_hip_rpy_x + foot_hip_rpy_z * foot_hip_rpy_z));
        }
        /** 更新当前参数**/
        finals.clear();
        finals.emplace_back(hip_yaw_);
        finals.emplace_back(hip_roll_);
        finals.emplace_back(hip_pitch_);
        finals.emplace_back(knee_pitch_);
        finals.emplace_back(ankle_pitch_);
        finals.emplace_back(ankle_roll_);
        dmotion::Rad2Deg(finals);
        return finals;

    }

}

