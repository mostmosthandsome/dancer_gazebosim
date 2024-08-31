#include "MotionController.h"
#include "PendulumWalk.h"
#include "OneFootLanding.h"
#include <vector>
#include <gz/plugin/Register.hh>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <queue>

using namespace gz;
using namespace sim;
using namespace systems;
using namespace zjudancer;
using namespace dmotion;

template <typename T>
void DELETE(T *&x) {
  if (x != NULL) {
    delete x;
    x = NULL;
  }
}

void paramater_server_thread_func(std::shared_ptr<dmotion::ParamNode> param_server)
{
  rclcpp::spin(param_server);
  RCLCPP_INFO(rclcpp::get_logger("parameter_thread"),"param server end");
}

class MotionController::MotionControllerPrivate
{
public:
    /**
     * @brief 初始化IO和参数节点，同时将默认姿势赋值为站立姿势
     */
    MotionControllerPrivate();
    bool is_working;

    std::shared_ptr<IO> IOptr;
    /// \brief Gazebo communication node.
    transport::Node node;
    std::thread parameter_server_thread;
    /// \brief 参数表节点
    std::shared_ptr<dmotion::ParamNode> param_server_node;
    /// \brief 记忆上一次的指令，为了模仿舵机上电后会维持当前位置的特性，
    std::vector< double > joint_command;
    std::queue< std::vector<double> > action_list;
    PendulumWalk *pendulum_global = NULL;
};

MotionController::MotionControllerPrivate::MotionControllerPrivate():IOptr(std::make_unique<IO>()),is_working(false)
{
    char a[] = "zjudancer"; char* b[1] = {a};
    rclcpp::init(1,b);
    param_server_node = std::make_shared<dmotion::ParamNode>();
    parameter_server_thread = std::thread(paramater_server_thread_func,param_server_node);
    gzmsg << "param server init successfully\n";

    double com_x = -param_server_node->parameters->pendulum_walk_param.COM_X_OFFSET;
    double com_y = -param_server_node->parameters->pendulum_walk_param.ANKLE_DIS / 2;
    double com_z = param_server_node->parameters->pendulum_walk_param.COM_HEIGHT;
    dmotion::OneFootLanding f(false,param_server_node->parameters);
    std::vector<double> hangfoot({0,com_y * 2,0,0,0,0}),com({com_x,com_y,com_z}),upbody_pose({0,0,0});
    joint_command = f.GetOneStep(hangfoot,com,upbody_pose);
    for(int i = 0; i < 2; ++i)  joint_command.push_back(param_server_node->parameters->stp.UPARM_ANGLE),joint_command.push_back(param_server_node->parameters->stp.LOWARM_ANGLE);
    for(int i = 0; i < 6; ++i)   joint_command[i + 6] = joint_command[i];
    joint_command = dmotion::Deg2Rad(joint_command);
}

MotionController::MotionController():dataPtr(std::make_unique<MotionControllerPrivate>())
{
}

MotionController::~MotionController()
{
    rclcpp::shutdown();
    if(dataPtr->parameter_server_thread.joinable()) dataPtr->parameter_server_thread.join();
}

void MotionController::Configure(const Entity &_entity,const std::shared_ptr<const sdf::Element> &_sdf,EntityComponentManager &_ecm, EventManager &_eventMgr)
{
    dataPtr->IOptr->Init(_entity,_sdf, _ecm, _eventMgr);
}

void MotionController::ComputeMotion()
{
  switch (this->dataPtr->param_server_node->parameters->status_code)
  {
    case StatusCode::CROUCH: //默认姿态是CROUCH状态
    {
      if (this->dataPtr->param_server_node->parameters->stp.gait_queue.size() == 0) //防止突然停下，加一个特判
      {
        if (this->dataPtr->param_server_node->parameters->stp.last_gait.x != 0 || this->dataPtr->param_server_node->parameters->stp.last_gait.y != 0 || this->dataPtr->param_server_node->parameters->stp.last_gait.t != 0) 
        {
          dataPtr->param_server_node->parameters->stp.tmp_gait.isRight = !dataPtr->param_server_node->parameters->stp.tmp_gait.isRight;
          dataPtr->param_server_node->parameters->stp.tmp_gait.label = "before crouch";
          dataPtr->param_server_node->parameters->stp.tmp_gait.x = dataPtr->param_server_node->parameters->stp.tmp_gait.y = dataPtr->param_server_node->parameters->stp.tmp_gait.t = 0;
          dataPtr->param_server_node->parameters->stp.gait_queue.push_back(dataPtr->param_server_node->parameters->stp.tmp_gait);
          dataPtr->param_server_node->parameters->stp.tmp_gait.isRight = ! dataPtr->param_server_node->parameters->stp.last_gait.isRight;
          dataPtr->param_server_node->parameters->stp.tmp_gait.label = "before crouch 2";
          dataPtr->param_server_node->parameters->stp.gait_queue.push_back(dataPtr->param_server_node->parameters->stp.tmp_gait);
        }
      }
      break;
    }

    case StatusCode::FORWARD_FALL_GETUP: 
    {
      
    }
    
  }
    
  if (dataPtr->param_server_node->parameters->stp.gait_queue.size() != 0) 
  {
    gzmsg << "the step x is " << this->dataPtr->param_server_node->parameters->stp.gait_queue[0].x
          << " y is " << this->dataPtr->param_server_node->parameters->stp.gait_queue[0].y
           << " yaw is " << this->dataPtr->param_server_node->parameters->stp.gait_queue[0].t << std::endl;
    //  pendulum_global->GiveAStepTick(this->dataPtr->param_server_node->parameters->stp.gait_queue[0]);
    //   this->dataPtr->param_server_node->parameters->stp.last_gait = this->dataPtr->param_server_node->parameters->stp.gait_queue[0];
    //   tmp_queue = this->dataPtr->param_server_node->parameters->stp.gait_queue;
    //   this->dataPtr->param_server_node->parameters->stp.gait_queue.clear();
    //   for (unsigned i = 1; i < tmp_queue.size(); i++) {
    //     this->dataPtr->param_server_node->parameters->stp.gait_queue.push_back(tmp_queue[i]);
    //   }
    //   if (this->dataPtr->param_server_node->parameters->stp.gait_queue.size() == 0 &&
    //       gettimes() - action_now_time > action_crouch_time &&
    //       parameters.status_code != 113 && parameters.status_code != 123 &&
    //       parameters.status_code != 314 && parameters.status_code != 324 &&
    //       parameters.status_code != 220 && parameters.status_code != 500) {
    //     parameters.status_code = StatusCode::CROUCH;
    //   }
    //   if (this->dataPtr->param_server_node->parameters->stp.last_gait.label == "before 400 2") {
    //     Delay(1500000);  //停下就停下，防止还没停好就直接开始走新的一步
    //   }
    //   // ROS_FATAL("step gived done");
    // } else {
    //   if (gettimes() - action_now_time > action_crouch_time &&
    //       parameters.status_code != 113 && parameters.status_code != 123 &&
    //       parameters.status_code != 314 && parameters.status_code != 324 &&
    //       parameters.status_code != 220 && parameters.status_code != 500) {
    //     // ROS_FATAL("long time no action_command");
    //     gait_type = 0;
    //     parameters.status_code = StatusCode::CROUCH;
    //   }
    }  
}

void MotionController::PreUpdate(const gz::sim::UpdateInfo &_info,gz::sim::EntityComponentManager &_ecm)
{

    if (_info.paused)    return; // 如果暂停就直接return
    
    if(this->dataPtr->action_list.empty())  ComputeMotion();//如果动作都已经执行完毕，运行上位机程序
    else   this->dataPtr->joint_command = this->dataPtr->action_list.front(),this->dataPtr->action_list.pop(); 
    
    dataPtr->IOptr->ServoControl(this->dataPtr->joint_command, _info, _ecm);
}

void MotionController::PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm)
{
    if (_info.paused)    return; // 如果暂停就直接return

    this->dataPtr->IOptr->update_observation(_info,_ecm);
    this->dataPtr->param_server_node->parameters->stp.robot_global = this->dataPtr->IOptr->get_robot_global();
    this->dataPtr->param_server_node->parameters->stp.ball_global = std::vector<double>({1.2,0,0});

    //计算ball_field
    Eigen::Vector2d ball_field;
    for(int i = 0; i < 2; ++i)   ball_field[i] = this->dataPtr->param_server_node->parameters->stp.ball_global[i] - this->dataPtr->param_server_node->parameters->stp.robot_global[i];
    Eigen::Rotation2D rotation(-this->dataPtr->param_server_node->parameters->stp.robot_global[2]);
    ball_field = rotation * ball_field;

    this->dataPtr->param_server_node->parameters->stp.ball_field[0] = ball_field[0],this->dataPtr->param_server_node->parameters->stp.ball_field[1] = ball_field[1];//球在机器人坐标系下的位置
    this->dataPtr->param_server_node->parameters->stp.ball_field[2] = this->dataPtr->param_server_node->parameters->stp.ball_global[2] - this->dataPtr->param_server_node->parameters->stp.robot_global[2];
    std::string str;
    for(int i = 0; i < 3; ++i)  str += std::to_string(this->dataPtr->param_server_node->parameters->stp.ball_field[i]) + ',';
    gzmsg << str << std::endl;
}


GZ_ADD_PLUGIN(MotionController, System, MotionController::ISystemConfigure, MotionController::ISystemPreUpdate, MotionController::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(MotionController, "gz::sim::systems::motion_controller")