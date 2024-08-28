#include "MotionController.h"
#include "OneFootLanding.h"
#include <vector>
#include <gz/plugin/Register.hh>
#include "rclcpp/rclcpp.hpp"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace zjudancer;



void paramater_server_thread_func(std::shared_ptr<dmotion::ParamNode> param_server)
{
  rclcpp::spin(param_server);
  RCLCPP_INFO(rclcpp::get_logger("parameter_thread"),"param server end");
}

class MotionController::MotionControllerPrivate
{
public:
    MotionControllerPrivate();
    bool is_working;

    std::shared_ptr<IO> IOptr;
    /// \brief Gazebo communication node.
    transport::Node node;
    std::thread parameter_server_thread;
    /// \brief 参数表节点
    std::shared_ptr<dmotion::ParamNode> param_server_node;
    std::vector< double > joint_command;
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

void MotionController::PreUpdate(const gz::sim::UpdateInfo &_info,gz::sim::EntityComponentManager &_ecm)
{

    if (_info.paused)    return; // 如果暂停就直接return

    if(dataPtr->is_working)
    {

    }
    
    dataPtr->IOptr->ServoControl(dataPtr->joint_command, _info, _ecm);
}


GZ_ADD_PLUGIN(MotionController, System, MotionController::ISystemConfigure, MotionController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(MotionController, "gz::sim::systems::motion_controller")