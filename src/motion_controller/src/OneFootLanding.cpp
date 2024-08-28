//
// Created by zjudancer on 2018-12-25.
//

#include "OneFootLanding.h"
#include <Eigen/Dense>
using namespace Eigen;

namespace dmotion {
        OneFootLanding::OneFootLanding(bool isRight,std::shared_ptr<Parameters> parameters_) : isRight_(isRight),parameters(parameters_)
        {
                left_leg_ = new InvKin(false, parameters);
                right_leg_ = new InvKin(true, parameters);  
        }

        OneFootLanding::~OneFootLanding(){
                delete left_leg_;
                delete right_leg_;
                left_leg_ = nullptr;
                right_leg_ = nullptr;
        }

        /**
         * 进行求解，这些参数都是相对于立足脚中心点的，最稳妥重心投影点根据脚板的安装位置做调整
         * @param hang_foot 悬荡脚的xyzrpy,度数制
         * 
         * @param whole_body_com 整机的重心
         * @param upbody_yaw 上半身的yaw
         * @return 12个舵机的值
         */
        std::vector<double>  OneFootLanding::GetOneStep(std::vector<double> hang_foot, const std::vector<double> &com_pos)
        {
                return GetOneStep(hang_foot,com_pos,{parameters->one_foot_landing_param.UPBODY_CONSTANT_ROLL,parameters->one_foot_landing_param.UPBODY_CONSTANT_PITCH,0},constant_value);
        }

        std::vector<double>    OneFootLanding::GetOneStep(std::vector<double> hang_foot, const std::vector<double> &com_pos,
                               std::vector<double> upbody_pose_,upbody_mode mode) {
        hang_foot[3] = dmotion::Deg2Rad(hang_foot[3]);
        hang_foot[4] = dmotion::Deg2Rad(hang_foot[4]);
        hang_foot[5] = dmotion::Deg2Rad(hang_foot[5]);
        upbody_yaw = dmotion::Deg2Rad(upbody_pose_[2]);
        AngleAxisd Roll(hang_foot[3],Vector3d::UnitX()),Pitch(hang_foot[4],Vector3d::UnitY()),Yaw(0.785398163,Vector3d::UnitZ());
        Matrix3d Rot = (Yaw * Pitch * Roll).toRotationMatrix();
        Vector3d ankle_offset(parameters->one_foot_landing_param.ANKLE_OFFSET_X,parameters->one_foot_landing_param.ANKLE_OFFSET_Y,parameters->one_foot_landing_param.ANKLE_OFFSET_Z),hang_foot_pos(hang_foot[0],hang_foot[1],hang_foot[2]);
        Vector3d hang_foot_com = Rot * ankle_offset + hang_foot_pos,landingfoot_com = ankle_offset,whole_body_com(com_pos[0],com_pos[1],com_pos[2]);
        double upbody_mass = parameters->one_foot_landing_param.UPBODY_MASS,foot_mass = parameters->one_foot_landing_param.FOOT_MASS;
        Vector3d upbody_com = ((upbody_mass + 2 * foot_mass) * whole_body_com - foot_mass * hang_foot_com - foot_mass * landingfoot_com) / upbody_mass;
        Vector3d TA1 = upbody_com - landingfoot_com,TA2 = upbody_com - hang_foot_com; TA1 = TA1 / TA1.norm(),TA2 = TA2 / TA2.norm();
        Vector3d v = TA1 + TA2; v = v / v.norm();
//      确定一下使用什么类型的上半身姿态
        if (mode == upbody_still )
        {
                upbody_roll = 0;
                upbody_pitch = 0;
        }else if(mode == constant_value){
                upbody_roll = parameters->one_foot_landing_param.UPBODY_CONSTANT_ROLL;
                upbody_pitch = parameters->one_foot_landing_param.UPBODY_CONSTANT_PITCH;
        }else if(mode == self_adaption){
                //s3,-c3s2,c3c2
                upbody_roll = std::atan2(-v[1],v[2])*4.0/5.0;
                upbody_pitch = std::atan2(v[0], v[2] / std::cos(upbody_roll))*4.0/5.0;
        }else if (mode == customized_value){
                upbody_roll = dmotion::Deg2Rad(upbody_pose_[0]);
                upbody_pitch = dmotion::Deg2Rad(upbody_pose_[1]);
        }
        AngleAxisd upbody_Roll(upbody_roll,Vector3d::UnitX()),upbody_Pitch(upbody_pitch,Vector3d::UnitY()),upbody_Yaw(upbody_yaw,Vector3d::UnitZ());
        Matrix3d upbody_Rot = (upbody_Roll * upbody_Pitch * upbody_Yaw).toRotationMatrix();//还是ZYX欧拉角
        Vector3d body_centreVec(parameters->one_foot_landing_param.BODY_CENTER_X,parameters->one_foot_landing_param.BODY_CENTER_Y,parameters->one_foot_landing_param.BODY_CENTER_Z);
        body_centreVec = upbody_Rot * body_centreVec + upbody_com;

        //        std::cout << upbody_roll  <<" " << upbody_pitch << " "  << upbody_yaw  << std::endl;
        //给立足脚的逆运动学向量加入值
               
                //坐标系原点移动到身体中心，根据XYZ旋转矩阵将身体放平
        Vector3d landing_foot_inverse = upbody_Rot.transpose() * (-body_centreVec),hang_foot_inverse = upbody_Rot.transpose() * (hang_foot_pos - body_centreVec);
        landing_invkin.clear();
        for(int i = 0; i < 3; ++i) landing_invkin.emplace_back(landing_foot_inverse(i));
        landing_invkin.emplace_back(-dmotion::Rad2Deg(upbody_roll));
        landing_invkin.emplace_back(-dmotion::Rad2Deg(upbody_pitch));
        landing_invkin.emplace_back(-dmotion::Rad2Deg(upbody_yaw));
        //给悬荡脚的逆运动学向量加入值
        
        hanging_invkin.clear();
        for(int i = 0; i < 3; ++i) hanging_invkin.emplace_back(hang_foot_inverse(i));
        T1_1 = std::cos(upbody_pitch) * std::cos(hang_foot[4]) * std::cos(hang_foot[5]) * std::cos(upbody_yaw) -
               std::sin(hang_foot[4]) * std::sin(upbody_roll) * std::sin(upbody_yaw) +
               std::cos(upbody_roll) * std::cos(upbody_yaw) * std::sin(upbody_pitch) * std::sin(hang_foot[4]) +
               std::cos(hang_foot[4]) * std::cos(upbody_roll) * std::sin(hang_foot[5]) * std::sin(upbody_yaw) +
               std::cos(hang_foot[4]) * std::cos(upbody_yaw) * std::sin(upbody_pitch) * std::sin(upbody_roll) *
               std::sin(hang_foot[5]);
        T2_1 = std::cos(hang_foot[4]) * std::cos(upbody_roll) * std::cos(upbody_yaw) * std::sin(hang_foot[5]) -
               std::cos(upbody_pitch) * std::cos(hang_foot[4]) * std::cos(hang_foot[5]) * std::sin(upbody_yaw) -
               std::cos(upbody_yaw) * std::sin(hang_foot[4]) * std::sin(upbody_roll) -
               std::cos(upbody_roll) * std::sin(upbody_pitch) * std::sin(hang_foot[4]) * std::sin(upbody_yaw) -
               std::cos(hang_foot[4]) * std::sin(upbody_pitch) * std::sin(upbody_roll) * std::sin(hang_foot[5]) *
               std::sin(upbody_yaw);
        T3_1 = std::cos(hang_foot[4]) * std::cos(hang_foot[5]) * std::sin(upbody_pitch) -
               std::cos(upbody_pitch) * std::cos(upbody_roll) * std::sin(hang_foot[4]) -
               std::cos(upbody_pitch) * std::cos(hang_foot[4]) * std::sin(upbody_roll) * std::sin(hang_foot[5]);
        T3_2 = std::cos(upbody_pitch) * std::cos(hang_foot[4]) * std::cos(upbody_roll) * std::sin(hang_foot[3]) -
               std::cos(hang_foot[3]) * std::sin(upbody_pitch) * std::sin(hang_foot[5]) -
               std::cos(upbody_pitch) * std::cos(hang_foot[3]) * std::cos(hang_foot[5]) * std::sin(upbody_roll) +
               std::cos(hang_foot[5]) * std::sin(upbody_pitch) * std::sin(hang_foot[4]) * std::sin(hang_foot[3]) -
               std::cos(upbody_pitch) * std::sin(hang_foot[4]) * std::sin(hang_foot[3]) * std::sin(upbody_roll) *
               std::sin(hang_foot[5]);
        T3_3 = std::sin(upbody_pitch) * std::sin(hang_foot[3]) * std::sin(hang_foot[5]) +
               std::cos(upbody_pitch) * std::cos(hang_foot[4]) * std::cos(hang_foot[3]) * std::cos(upbody_roll) +
               std::cos(hang_foot[3]) * std::cos(hang_foot[5]) * std::sin(upbody_pitch) * std::sin(hang_foot[4]) +
               std::cos(upbody_pitch) * std::cos(hang_foot[5]) * std::sin(hang_foot[3]) * std::sin(upbody_roll) -
               std::cos(upbody_pitch) * std::cos(hang_foot[3]) * std::sin(hang_foot[4]) * std::sin(upbody_roll) *
               std::sin(hang_foot[5]);

        hanging_invkin.emplace_back(dmotion::Rad2Deg(dmotion::Atan(T3_2, T3_3)));
        hanging_invkin.emplace_back(dmotion::Rad2Deg(std::asin(-T3_1)));
        hanging_invkin.emplace_back(dmotion::Rad2Deg(dmotion::Atan(T2_1, T1_1)));

//        dmotion::PrintVector(hanging_invkin);
//        dmotion::PrintVector(landing_invkin);
        one_foot_result.clear();
        if (isRight_) {
//            std::cout << "right" << std::endl;
            dmotion::AddElements(one_foot_result, right_leg_->LegInvKin(landing_invkin));
            dmotion::AddElements(one_foot_result, left_leg_->LegInvKin(hanging_invkin));
        } else if (!isRight_) {
//            std::cout << "left" << std::endl;
            dmotion::AddElements(one_foot_result, right_leg_->LegInvKin(hanging_invkin));
            dmotion::AddElements(one_foot_result, left_leg_->LegInvKin(landing_invkin));
        }

        return one_foot_result;

    }


    void OneFootLanding::unit_arrow(std::vector<double> &arrow) {
        double len = std::sqrt(arrow[0] * arrow[0] + arrow[1] * arrow[1] + arrow[2] * arrow[2]);
        for (unsigned int i = 0; i < arrow.size(); i++)
            arrow[i] = arrow[i] / len;
    }


}