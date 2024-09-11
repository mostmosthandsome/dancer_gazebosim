/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-06-02 08:06:23
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-06-04 07:02:20
 * @FilePath: \dancer-workspace\dancer-workspace\workspaces\core\src\dmotion\src\NewMotion.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "NewMotion.h"

using namespace std;

namespace dmotion
{
// newKick 类实现
// newKick::newKick(std::shared_ptr< std::queue< std::vector<double> > > _action_list,  bool kick) : pbr(publisher), lopr(loop_rate) ,GenerateMotion(false, false, ""), kicker(publisher,loop_rate, false){} //  变量不确定

// void newKick::run() {
//     if (isKick()) {
//         kicker.working();
//         }
        
//     }



// newClimb 类实现
newClimb::newClimb(std::shared_ptr< std::queue< std::vector<double> > > _action_list, std::shared_ptr<Parameters> _parameters, std::string climbDirection): GenerateMotion(StatusCode::FORWARD_FALL_GETUP), climber(_action_list , climbDirection, _parameters)
{
    if(climbDirection == "BACK")    status = StatusCode::BACKWARD_FALL_GETUP;
}

void newClimb::run() 
{
    climber.working();        
}

// WalkMotion 类实现
newPendulumWalk::newPendulumWalk(std::shared_ptr< PendulumWalk > walk_ptr,std::shared_ptr<Parameters>  _param):walker(walk_ptr),local_gait(_param->stp.tmp_gait),GenerateMotion(StatusCode::WALK),parameters(_param)
{
}

/**
 * @brief 根据parameters 里的tmp_gait进行一步动作
 */
void newPendulumWalk::run() {
    parameters->stp.last_gait = local_gait;
    walker->GiveAStepTick(local_gait);
}

void  newPendulumWalk::fuse(gait_element new_gait)
{
    local_gait.x = (local_gait.x + new_gait.x) / 2,local_gait.y = (local_gait.y + new_gait.y) / 2,local_gait.t = (local_gait.t + new_gait.t) / 2;
}

}