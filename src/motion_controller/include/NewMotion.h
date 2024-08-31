/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-06-02 08:00:21
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-06-04 07:48:06
 * @FilePath: \dancer-workspace\dancer-workspace\workspaces\core\src\dmotion\include\newMotion.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef NEWMOTION_H
#define NEWMOTION_H

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <time.h>
#include <sys/time.h>
#include "Utility/dmotion_math.hpp"
#include "OneFootLanding.h"
#include "ThreeInterpolation.h"
#include "ForwardKinematics.h"
#include "PendulumWalk.h"
#include "Climb.h"
#include "Kick.h"
#include "GenerateMotion.h"
#include "Parameters.h"
#include <memory>

namespace dmotion
{
    class newKick : public GenerateMotion {
    public:
        ros::Publisher *pbr; //用于发给IO舵机值
        ros::Rate *lopr;     //用于控制发值周期
        newKick(ros::Publisher *publisher, ros::Rate *loop_rate, bool kick);
        void run() override;
    private:
        Kick kicker;


    };

    class newClimb : public GenerateMotion {
    public:
        ros::Publisher *pbr; //用于发给IO舵机值
        ros::Rate *lopr;     //用于控制发值周期
        newClimb(ros::Publisher *publisher, ros::Rate *loop_rate, std::vector<double> position_start, std::string climbDirection);
        void run() override;
    private:
        Climb climber;


    };

    class newPendulumWalk : public GenerateMotion {
    public:
        ros::Publisher *pbr; //用于发给IO舵机值
        ros::Rate *lopr;     //用于控制发值周期
        newPendulumWalk(gait_element gait,std::shared_ptr< PendulumWalk > walk_ptr);
        virtual void run() override;
        gait_element get_gait() {return local_gait;}
        void fuse(gait_element new_gait);
    private:
        std::shared_ptr< PendulumWalk > walker;
        gait_element local_gait;
    };

}


#endif // GENERATEMOTION_H