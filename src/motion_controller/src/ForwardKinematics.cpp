//
// Created by zjudancer on 19-1-19
// E-mail: zjufanwu@zju.edu.cn
//

#include <ForwardKinematics.h>
#include "ForwardKinematics.h"


namespace dmotion {



    ForKin::ForKin( std::shared_ptr<Parameters> _parameters, const std::vector<double> angles, bool isRight):parameters(_parameters) {
        neck_z_from_hip = 17.4;
        hip_z = 6;
        for (unsigned i = 0; i < angles.size(); i++) {
            angles_.emplace_back(angles[i] * M_PI / 180.0);
        }
        isRight_ = isRight;
        double distance_tmp = std::sqrt(parameters->one_foot_landing_param.HALF_HIP_WIDTH * parameters->one_foot_landing_param.HALF_HIP_WIDTH + parameters->one_foot_landing_param.HIP_X_FROM_ORIGIN * parameters->one_foot_landing_param.HIP_X_FROM_ORIGIN);
        double angle_tmp = std::atan(parameters->one_foot_landing_param.HIP_X_FROM_ORIGIN / parameters->one_foot_landing_param.HALF_HIP_WIDTH);
        //机器人左腿SDH参数表
        alpha = {0, M_PI / 2, M_PI / 2, 0, 0, -M_PI / 2, M_PI / 2};
        a = {distance_tmp, 0, 0, parameters->one_foot_landing_param.UPPER_LEG_LENGTH, parameters->one_foot_landing_param.LOWER_LEG_LENGTH, 0, parameters->one_foot_landing_param.ANKLE_FROM_GROUND};
        d = {0, -parameters->one_foot_landing_param.HIP_Z_FROM_ORIGIN, 0, 0, 0, 0, 0,};
        theta = {M_PI / 2 - angle_tmp,
                 angle_tmp + angles_[0],
                 angles_[1] - M_PI / 2,
                 angles_[2],
                 -angles_[3],
                 angles_[4],
                 -angles_[5]};

        T = Eigen::Isometry3d::Identity();

        for (int i = 0; i < 7; i++) {
            Eigen::AngleAxisd rotate_yaw(theta[i], Eigen::Vector3d(0, 0, 1));
            Eigen::AngleAxisd rotate_roll(alpha[i], Eigen::Vector3d(1, 0, 0));
            T.rotate(rotate_yaw);
            T.translate(Eigen::Vector3d(0, 0, d[i]));
            T.translate(Eigen::Vector3d(a[i], 0, 0));
            T.rotate(rotate_roll);
        }
        //这两步是用于保证脚末端的坐标系在所有关节角都为0时，和世界坐标系统一
        Eigen::AngleAxisd rotate_pitch90(-M_PI / 2, Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd rotate_yaw90(M_PI / 2, Eigen::Vector3d(0, 0, 1));
        T.rotate(rotate_pitch90);
        T.rotate(rotate_yaw90);

        // std::cout << T.matrix() << std::endl;


        if (0 == isRight_) {
            result_vector = Matrix2Pose(T);
        } else {
            result_vector = Matrix2Pose(T);
            result_vector[1] = -result_vector[1];
            result_vector[3] = -result_vector[3];
            result_vector[5] = -result_vector[5];

        }




        // dmotion::PrintVector(result_vector);

    }
    
    ForKin::ForKin() {}
    //标准DH
    //坐标系01分别在：脖子上端　髋部下端
    std::vector<double> ForKin::FK_neck_to_foot(const std::vector<double> angles, bool isRight){
        
        angles_.clear();
        for (unsigned i = 0; i < angles.size(); i++) angles_.emplace_back(angles[i] * M_PI / 180.0);
        isRight_ = isRight;
        double distance_tmp = std::sqrt(parameters->one_foot_landing_param.HALF_HIP_WIDTH * parameters->one_foot_landing_param.HALF_HIP_WIDTH + parameters->one_foot_landing_param.HIP_X_FROM_ORIGIN * parameters->one_foot_landing_param.HIP_X_FROM_ORIGIN);
        double angle_tmp = std::atan(parameters->one_foot_landing_param.HIP_X_FROM_ORIGIN / parameters->one_foot_landing_param.HALF_HIP_WIDTH);
        //机器人左腿标准DH参数表
        alpha = {0, M_PI / 2, M_PI / 2, 0, 0, -M_PI / 2, M_PI / 2};
        a = {distance_tmp, 0, 0, parameters->one_foot_landing_param.UPPER_LEG_LENGTH, parameters->one_foot_landing_param.LOWER_LEG_LENGTH, 0, parameters->one_foot_landing_param.ANKLE_FROM_GROUND};
        d = {-neck_z_from_hip, -hip_z, 0, 0, 0, 0, 0,};
        theta = {M_PI / 2 - angle_tmp,
                 angle_tmp + angles_[0],
                 angles_[1] - M_PI / 2,
                 angles_[2],
                 -angles_[3],
                 angles_[4],
                 -angles_[5]};

        T = Eigen::Isometry3d::Identity();

        for (int i = 0; i < 7; i++) {
            Eigen::AngleAxisd rotate_yaw(theta[i], Eigen::Vector3d(0, 0, 1));
            Eigen::AngleAxisd rotate_roll(alpha[i], Eigen::Vector3d(1, 0, 0));
            T.rotate(rotate_yaw);
            T.translate(Eigen::Vector3d(0, 0, d[i]));
            T.translate(Eigen::Vector3d(a[i], 0, 0));
            T.rotate(rotate_roll);
        }
        //这两步是用于保证脚末端的坐标系在所有关节角都为0时，和世界坐标系统一
        Eigen::AngleAxisd rotate_pitch90(-M_PI / 2, Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd rotate_yaw90(M_PI / 2, Eigen::Vector3d(0, 0, 1));
        T.rotate(rotate_pitch90);
        T.rotate(rotate_yaw90);


        T.translate(Eigen::Vector3d(0, -parameters->pendulum_walk_param.ANKLE_DIS / 2.0, 0));




        T=T.inverse();



        // std::cout << T.matrix() << std::endl;
        result_vector.clear();

        if (0 == isRight_) {
            result_vector = Matrix2Pose(T);
        } else {
            result_vector = Matrix2Pose(T);
            result_vector[1] = -result_vector[1];
            result_vector[3] = -result_vector[3];
            result_vector[5] = -result_vector[5];

        }

        return result_vector;




    }


    ForKinPlus::ForKinPlus(std::vector<double> supporting, std::vector<double> hanging) {

        S = Eigen::Isometry3d::Identity();
        H = Eigen::Isometry3d::Identity();

        S.translate(Eigen::Vector3d(supporting[0], 0, 0));
        S.translate(Eigen::Vector3d(0, supporting[1], 0));
        S.translate(Eigen::Vector3d(0, 0, supporting[2]));
        Eigen::AngleAxisd support_yaw(dmotion::Deg2Rad(supporting[5]), Eigen::Vector3d(0, 0, 1));
        Eigen::AngleAxisd support_pitch(dmotion::Deg2Rad(supporting[4]), Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd support_roll(dmotion::Deg2Rad(supporting[3]), Eigen::Vector3d(1, 0, 0));
        S.rotate(support_yaw);
        S.rotate(support_pitch);
        S.rotate(support_roll);


        H.translate(Eigen::Vector3d(hanging[0], 0, 0));
        H.translate(Eigen::Vector3d(0, hanging[1], 0));
        H.translate(Eigen::Vector3d(0, 0, hanging[2]));
        Eigen::AngleAxisd hang_yaw(dmotion::Deg2Rad(hanging[5]), Eigen::Vector3d(0, 0, 1));
        Eigen::AngleAxisd hang_pitch(dmotion::Deg2Rad(hanging[4]), Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd hang_roll(dmotion::Deg2Rad(hanging[3]), Eigen::Vector3d(1, 0, 0));
        H.rotate(hang_yaw);
        H.rotate(hang_pitch);
        H.rotate(hang_roll);
        //身体中心相对于支撑脚的变换矩阵为S_inv，也就是S的逆矩阵
        Eigen::Isometry3d S_inv = S.inverse();
        center2support = Matrix2Pose(S_inv);
        //摆动脚相对于支撑脚的变换矩阵为H2S
        Eigen::Isometry3d H2S(S_inv.matrix()*H.matrix());
        hang2support = Matrix2Pose(H2S);

    }


    std::vector<double> Matrix2Pose(Eigen::Isometry3d M) {
        std::vector<double> pose;
        pose.emplace_back(M.matrix()(0, 3));
        pose.emplace_back(M.matrix()(1, 3));
        pose.emplace_back(M.matrix()(2, 3));

        pose.emplace_back(dmotion::Rad2Deg(dmotion::Atan(M.matrix()(2, 1), M.matrix()(2, 2))));
        pose.emplace_back(dmotion::Rad2Deg(std::asin(-M.matrix()(2, 0))));
        pose.emplace_back(dmotion::Rad2Deg(dmotion::Atan(M.matrix()(1, 0), M.matrix()(0, 0))));

        return pose;
    }


}