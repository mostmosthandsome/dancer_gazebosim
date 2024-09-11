//
// Created by ZJUDancer on 2018-12-25 Christmas Day!
// author Wu Fan
// E-mail zjufanwu@zju.edu.cn
// This is the demo version of straight walk
#pragma once

#include "OneFootLanding.h"
#include "ThreeInterpolation.h"
#include "Parameters.h"
#include <iostream>
#include <time.h>
#include <cmath>
#include <math.h>
#include <sys/time.h>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#define FALL_FOR_TOLERANCE 1
#define FALL_SIDE_TOLERANCE 1  
#define FALL_TICK_NUM 5
#define USE_WALK_FEEDBACK 0       // 为1则开启步态反馈， 0则关闭
// #define ANKLE_DIS   15.0        //通过观察,肉包的踝间距差不多是11到12左右,单位为cm
// #define TAO 0.35                //通过观察,肉包的步态单元的时长是0.35s
// #define TICK_NUM 35             //每个步态周期发35个值
// #define COM_H 37.0              /*通过把机器人固定成crouch姿势然后倒挂测量周期，测得其在全身不动的情况下特征
//                                 摆长度为37cm*/
// #define ACC_COEF_X    0.15      //把本次质心前进dx和上一回dx进行做差对比，乘以系数的数字作为质心的位置移动，插值后在本次中叠加在倒立摆x轨迹上
// #define ACC_COEF_Y    0.3
// #define COM_HEIGHT    27        //默认规划重心高度
// #define Y_HALF_AMPLITUDE  3.0   //y方向倒立摆起点坐标长度

using namespace std;
namespace dmotion
{

vector<double> OneStepTransform(double global_start_x, double global_start_y, double global_start_yaw, double global_end_x, double global_end_y, double global_end_yaw);
class PendulumWalk
{
  public:
    PendulumWalk(std::shared_ptr< std::queue< std::vector<double> > > _action_list, std::shared_ptr<Parameters> param);
    PendulumWalk(std::shared_ptr< std::queue< std::vector<double> > > _action_list, std::shared_ptr<Parameters> param, const double &unusual_ankle_dis);
    //相对于理想情况的重心轨迹曲线的x、y方向的偏移
    double x_i, y_i, yaw_i;       //假想的每步移动量直接施加在这个虚拟位移上面
    double com_ac_x;              //用于加速的x和y方向的偏移
    double com_ac_y;
    double com_x_changed;           //上一回x方向的单步前进大小
    double com_y_changed;
    bool support_is_right;         //上一步的支撑脚是否是右脚
    std::vector<double> hang_foot; //上一步hang_foot最后落地时的x,y,yaw,单位是cm和弧度制
    std::vector<double> com_pos;   //上一步规划com最后落地时的x,y,yaw,单位是cm和弧度制，不包含com_ac_x
    std::vector<double> last_rpy;  //上一个周期收到的RPY值

    ~PendulumWalk();
    void GiveAStep(double dx, double dy, double dyaw);
    void GiveATick();
    void GiveAStepTick(gait_element now_gait);

    bool CheckWillFall(); //新函数，判断是否要调整落脚点
    bool AdjustFoothold(double dx, double dy, double d_yaw);  //新函数，在腿摆动的时候调整落脚点
    vector<double> CalculateTurnSequence(double turn_angle);
    void SlowDown();//匀速减少
    void LittleAdjust(double start_x, double start_y, double start_angle, double end_x, double end_y, double end_angle=1000);    
    void TurnAround(double angle);

    int tick_num;
    int fall_pre_tick;

  private:
    // 以下参数的含义参见仿人机器人的书和MATLAB程序pendulum_walk.m
    double x0; //这两个决定了步距
    double xt;
    double vx;
    double com_h;
    double Tc;
    double x;
    double tao;

    //这里固定y0为3cm,和理论不符<，实际把这个轨迹进行了平移到了6.5（ANKLE_DIS/2）处
    //真实的相对于的支撑脚的y方向起点是y00
    double y00;
    double ytt;
    double y0;
    double yt;
    double m;
    double vy;
    double y;
    //行走插值序列
    std::vector<double> akZ;
    std::vector<double> comYaw;
    std::vector<double> comY;
    std::vector<double> accX;
    std::vector<double> accY;
    std::vector<double> akX;
    std::vector<double> akY;
    std::vector<double> akYaw;

    dmotion::OneFootLanding *Support;
    std::shared_ptr<Parameters> parameters;
    //动作序列
    std::shared_ptr< std::queue< std::vector<double> > > action_list;

};
} // namespace dmotion