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
#include "Parameters.h"
#include <memory>
#include <queue>

namespace dmotion
{

    class GenerateMotion {
    public:
    // 构造函数
        GenerateMotion(StatusCode _status):status(_status) {}

        // 纯虚函数：run
        virtual void run() = 0; // 纯虚函数，使GenerateMotion成为抽象类

        // 虚析构函数
        virtual ~GenerateMotion() = default;

        StatusCode get_status() {return status;}
    protected:
        StatusCode status; 
    };

    // class newKick : public GenerateMotion {
    // public:
    //     newKick(ros::Publisher *publisher, ros::Rate *loop_rate, bool kick);
    //     void run() override;
    // private:
    //     Kick kicker;


    // };

    class newClimb : public GenerateMotion {
    public:
        newClimb(std::shared_ptr< std::queue< std::vector<double> > > _action_list,  std::shared_ptr<Parameters> _parameters, std::string climbDirection);
        void run() override;
    private:
        Climb climber;


    };

    class newPendulumWalk : public GenerateMotion {
    public:
        newPendulumWalk(std::shared_ptr< PendulumWalk > walk_ptr, std::shared_ptr<Parameters> _param);
        virtual void run() override;
        gait_element get_gait() {return local_gait;}
        void fuse(gait_element new_gait);
    private:
        std::shared_ptr< PendulumWalk > walker;
        std::shared_ptr<Parameters> parameters;
        gait_element local_gait;
    };

}


#endif // GENERATEMOTION_H