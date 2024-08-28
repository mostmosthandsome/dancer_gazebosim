//
// Created by zjudancer on 18-12-18.
//

#ifndef INVERSEKINEMATICS_ONEFOOTLANDING_H
#define INVERSEKINEMATICS_ONEFOOTLANDING_H

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "Utility/dmotion_math.hpp"
#include "InverseKinematics.h"
#include "Parameters.h"
#include "rclcpp/rclcpp.hpp"

// #define UPBODY_CONSTANT_ROLL 0 //用于GetOneStep中的参数设置
// #define UPBODY_CONSTANT_PITCH 0.2

namespace dmotion
{ 
  // 用于指示上半身状态，upbody_still=上半rpy(0,0,0)、self_adaption=根据对角线准则自适应（参考img/OneFootLanding.jpg）
  // constant_value=固定的值，yaw人为制定,roll和pitch根据两个固定值确定
  // customized_value = 完全听取定制的值
  enum upbody_mode
  {
    upbody_still,
    self_adaption,
    constant_value,
    customized_value
  };
  class OneFootLanding
  {
  public:
    bool isRight_;

  private:
    friend class PendulumWalk;
    InvKin *left_leg_;
    InvKin *right_leg_;

    std::vector<double> body_centre; //身体单腿逆运动学起点(M)
    //已下三个变量是三维空间向量，参考 https://github.com/hannbusann/one-foot_landing_InvKin/blob/master/img/-75909a348f7ef85.jpg
    std::vector<double> TA1;
    std::vector<double> TA2;
    std::vector<double> v;
    double upbody_roll;
    double upbody_pitch;
    double upbody_yaw;
    //单腿逆运动学接口
    std::vector<double> landing_invkin;
    std::vector<double> hanging_invkin;
    //临时变量
    double T1_1;
    double T2_1;
    double T3_1;
    double T3_2;
    double T3_3;
    //最后得出的12个舵机的值
    std::vector<double> one_foot_result;
    std::shared_ptr<Parameters> parameters;

  public:
    OneFootLanding(bool isRight,std::shared_ptr<Parameters> parameters);

    ~OneFootLanding();
    /**
     * @brief compute the angles of servos using const roll and pitch from parameters
     */
    std::vector<double> GetOneStep(std::vector<double> hang_foot, const std::vector<double> &com_pos);

    /**
     * @brief 根据hang_foot, com, upbody_pose计算腿部的关节角
     * @return 按照先左后右，从上到下，ZYX的顺序返回关节角
     */
    std::vector<double> GetOneStep(std::vector<double> hang_foot, const std::vector<double> &whole_body_com,std::vector<double> upbody_pose, upbody_mode mode = constant_value);

    //矢量单位化
    void unit_arrow(std::vector<double> &arrow);
  };

} // namespace dmotion
#endif //INVERSEKINEMATICS_ONEFOOTLANDING_H
