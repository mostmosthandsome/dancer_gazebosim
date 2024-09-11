#include "MotionController.h"
#include "PendulumWalk.h"
#include "OneFootLanding.h"
#include "NewMotion.h"
#include <vector>
#include <gz/plugin/Register.hh>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <queue>
#include <list>
#include <mutex>
#include <thread>
#include "Planner.h"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace zjudancer;
using namespace dmotion;

using MotionList = std::list< std::shared_ptr< GenerateMotion> >;

#define STABLE_COUNT 30

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
//用来辅助判断是否倒下的枚举类
enum StableState
{
  STABLE,
  UNSTABLE,
  FRONTDOWN,
  BACKDOWN,
  RIGHTDOWN,
  LEFTDOWN
};

class MotionController::MotionControllerPrivate
{
public:
  /**
   * @brief 初始化IO和参数节点，同时将默认姿势赋值为站立姿势
   */
  MotionControllerPrivate();

  /**
   * @brief 用来根据rpy判断机器人是否已经倒下
   */
  void checkStableState(std::vector<double> rpy);

  /**
   * 将动作进行融合
   */
  void motion_fuse();

  void make_walk_plan(Eigen::Vector2d target_pos,double theta);

  void store_plan();


  bool is_working;

  std::shared_ptr<IO> IOptr;
  /// \brief Gazebo communication node.
  transport::Node node;
  std::thread parameter_server_thread;
  std::shared_ptr<dplanner::BasePlanner> planner;
  /// \brief 参数表节点
  std::shared_ptr<dmotion::ParamNode> param_server_node;
  /// \brief 记忆上一次的指令，为了模仿舵机上电后会维持当前位置的特性，
  std::vector< double > joint_command;
  MotionList motions;
  std::shared_ptr< std::queue< std::vector<double> > > action_list;
  /// \brief 维护一个全局的pendulum global来进行左右脚的切换
  std::shared_ptr<PendulumWalk> pendulum_global;
  /// \brief 轨迹规划线程和主线程之间通信的标志位，用来判断轨迹规划线程是否已经完成
  bool planner_ready = false;
  /// \brief 轨迹规划线程和主线程之间通信的共享内存，用来记录规划的结果
  std::vector<gait_element> temp_motions;
  /// \brief 用来保护规划线程和主线程之间共享内存的锁
  std::mutex planner_mutex;

  /// @brief  用来记录轨迹融合开始的位置
  int fusing_window_pos;
  //用来辅助计算是否倒下
  int frontcount, backcount, leftcount, rightcount, stablecount;
  /// \brief 用来进行慢启动，防止摔倒
  int walk_start = 0;

  std::chrono::steady_clock::duration nextControlTime{std::chrono::steady_clock::duration::zero()};
  std::chrono::steady_clock::duration get_next_control_time();

  std::thread planner_thread;
  StableState m_stable_state;


};

MotionController::MotionControllerPrivate::MotionControllerPrivate():IOptr(std::make_unique<IO>()),is_working(false),frontcount(0),backcount(0),leftcount(0),rightcount(0),stablecount(0),m_stable_state(StableState::STABLE),
action_list(std::make_shared< std::queue< std::vector<double> > >())
{
  if(!rclcpp::ok())
  {
    char a[] = "zjudancer"; char* b[1] = {a};
    rclcpp::init(1,b);
  }
  
  param_server_node = std::make_shared<dmotion::ParamNode>();
  parameter_server_thread = std::thread(paramater_server_thread_func,param_server_node);
  gzmsg << "param server init successfully\n";
  planner = std::make_shared<dplanner::SimplePlanner>(param_server_node->parameters);
  double com_x = -param_server_node->parameters->pendulum_walk_param.COM_X_OFFSET;
  double com_y = -param_server_node->parameters->pendulum_walk_param.ANKLE_DIS / 2;
  double com_z = param_server_node->parameters->pendulum_walk_param.COM_HEIGHT;
  dmotion::OneFootLanding f(false,param_server_node->parameters);
  std::vector<double> hangfoot({0,com_y * 2,0,0,0,0}),com({com_x,com_y,com_z}),upbody_pose({0,0,0});
  joint_command = f.GetOneStep(hangfoot,com,upbody_pose);
  for(int i = 0; i < 2; ++i)  joint_command.push_back(param_server_node->parameters->stp.UPARM_ANGLE),joint_command.push_back(param_server_node->parameters->stp.LOWARM_ANGLE);
  for(int i = 0; i < 6; ++i)   joint_command[i + 6] = joint_command[i];
  joint_command = dmotion::Deg2Rad(joint_command);
  pendulum_global = std::make_shared<PendulumWalk>(this->action_list,param_server_node->parameters);
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
  if(this->dataPtr->planner_ready)  this->dataPtr->motion_fuse(); //如果planner规划好链，则先进行动作融合
  if(!this->dataPtr->motions.empty())
  {
    ++this->dataPtr->fusing_window_pos;
    if(this->dataPtr->walk_start < 5)  ++this->dataPtr->walk_start;
    if(this->dataPtr->motions.front()->get_status() != StatusCode::WALK)  this->dataPtr->walk_start = 0;
    this->dataPtr->motions.front()->run();
    this->dataPtr->motions.pop_front();
  }

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
          auto step = newPendulumWalk(dataPtr->pendulum_global,dataPtr->param_server_node->parameters);
          step.run();
          dataPtr->param_server_node->parameters->stp.tmp_gait.isRight = ! dataPtr->param_server_node->parameters->stp.last_gait.isRight;
          dataPtr->param_server_node->parameters->stp.tmp_gait.label = "before crouch 2";
          dataPtr->param_server_node->parameters->stp.gait_queue.push_back(dataPtr->param_server_node->parameters->stp.tmp_gait);
          step = newPendulumWalk(dataPtr->pendulum_global,dataPtr->param_server_node->parameters);
          step.run();
        }
      }
      break;
    }
    case StatusCode::WALK_TO_BALL:
    {  
      //先求出踢球点在机器人坐标系下的位置
      Eigen::Vector2d robot_pos(this->dataPtr->param_server_node->parameters->stp.robot_global[0], this->dataPtr->param_server_node->parameters->stp.robot_global[1]),ball_pos(this->dataPtr->param_server_node->parameters->stp.ball_global[0],this->dataPtr->param_server_node->parameters->stp.ball_global[1]);
      Eigen::Vector2d target_pos(this->dataPtr->param_server_node->parameters->stp.ball_global[0] - this->dataPtr->param_server_node->parameters->stp.robot_global[0],this->dataPtr->param_server_node->parameters->stp.ball_global[1] - this->dataPtr->param_server_node->parameters->stp.robot_global[1]);
      Eigen::Vector2d right_kick_bias(this->dataPtr->param_server_node->parameters->kick_param.RIGHT_KICK_X,this->dataPtr->param_server_node->parameters->kick_param.RIGHT_KICK_Y);
      double rotation_angle = this->dataPtr->param_server_node->parameters->stp.action_global[2] - this->dataPtr->param_server_node->parameters->stp.robot_global[2];
      Eigen::Rotation2Dd rot(Deg2Rad(-this->dataPtr->param_server_node->parameters->stp.robot_global[2]));
      target_pos = rot * target_pos + right_kick_bias;
      if(target_pos.norm() < 1e-3 && rotation_angle < 1e-3)
      {
        this->dataPtr->motions.clear();
        // this->dataPtr->motions.push_back(std::make_shared<newKick>(&ServoInfo_pub,&loop_rate,true));
        this->dataPtr->param_server_node->parameters->status_code = StatusCode::RIGHT_KICK;
        break;
      }
      if(this->dataPtr->planner_ready)
      {
        this->dataPtr->planner_ready = false;
        if(this->dataPtr->planner_thread.joinable()) this->dataPtr->planner_thread.join();
        this->dataPtr->planner_thread = std::thread(&MotionControllerPrivate::make_walk_plan,this->dataPtr.get(),target_pos,rotation_angle);
        this->dataPtr->fusing_window_pos = 0;
      }     
        break;
    }
    case StatusCode::FORWARD_FALL_GETUP: 
    {
      this->dataPtr->pendulum_global.reset();
      auto climber = newClimb(this->dataPtr->action_list,this->dataPtr->param_server_node->parameters,"FORWARD");
      climber.run();
      this->dataPtr->pendulum_global = std::make_shared<PendulumWalk>(this->dataPtr->action_list, this->dataPtr->param_server_node->parameters);
      this->dataPtr->param_server_node->parameters->status_code = CROUCH;
      break;
    }
    case StatusCode::BACKWARD_FALL_GETUP: 
    {
      this->dataPtr->pendulum_global.reset();
      auto climber = newClimb(this->dataPtr->action_list,this->dataPtr->param_server_node->parameters,"BACK");
      climber.run();
      this->dataPtr->pendulum_global = std::make_shared<PendulumWalk>(this->dataPtr->action_list, this->dataPtr->param_server_node->parameters);
      this->dataPtr->param_server_node->parameters->status_code = CROUCH;
      break;
    }
  }
  // gzmsg << "status : " << this->dataPtr->param_server_node->parameters->status_code << std::endl;
}

void MotionController::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  if (_info.paused)    return; // 如果暂停就直接return
    
  if(this->dataPtr->action_list->empty())  ComputeMotion();//如果动作都已经执行完毕，运行上位机程序，更新action_list
  if(!this->dataPtr->action_list->empty())//如果更新后有动作，则取出并执行
  {
    if(this->dataPtr->get_next_control_time() <= _info.simTime)
    {
      auto delta = std::chrono::duration_cast< std::chrono::milliseconds>(std::chrono::duration< double >(1.0 / this->control_frequency));//转成ms是因为std::chrono::steady_clock::duration不支持与秒的相加
      this->dataPtr->nextControlTime += delta;
      auto top_action = this->dataPtr->action_list->front();  this->dataPtr->action_list->pop();
      if(top_action.size() == 1)    //我们规定如果取出来的front 为长度为1的动作，则是delay动作，唯一一个元素表示时间长度（s为单位）
      {
        auto delay_time = std::chrono::duration_cast< std::chrono::milliseconds>(std::chrono::duration< double >(top_action[0]));
        this->dataPtr->nextControlTime += delay_time;
      }
      else if(top_action.size() >= 16)  this->dataPtr->joint_command = Deg2Rad(top_action); //如果包括了手上的动作，就直接使用
      else if(top_action.size() == 12)//如果不包括手上的动作，需要让手回到原来的位置
      {
        this->dataPtr->joint_command = Deg2Rad(top_action);
        for(int i = 0; i < 2; ++i)  this->dataPtr->joint_command.push_back(this->dataPtr->param_server_node->parameters->stp.UPARM_ANGLE),this->dataPtr->joint_command.push_back(this->dataPtr->param_server_node->parameters->stp.LOWARM_ANGLE);
      }
    }
  }
  dataPtr->IOptr->ServoControl( this->dataPtr->joint_command, _info, _ecm);
}

void MotionController::PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm)
{
  if (_info.paused)    return; // 如果暂停就直接return

  this->dataPtr->IOptr->update_observation(_info,_ecm);//让IO从仿真中更新机器人的位置、imu、舵机位置等等

  auto servo_angles = this->dataPtr->IOptr->get_cur_servo_angles(_info,_ecm);
  if(servo_angles.empty())  return;
  this->dataPtr->param_server_node->parameters->stp.cur_servo_angles = Rad2Deg(servo_angles);

  this->dataPtr->param_server_node->parameters->stp.robot_global = this->dataPtr->IOptr->get_robot_global();
  
  //如果收到了球的坐标消息，则计算ball_field
  if (this->dataPtr->param_server_node->parameters->stp.see_ball)
  {
    Eigen::Vector2d ball_field;
    for(int i = 0; i < 2; ++i)   ball_field[i] = this->dataPtr->param_server_node->parameters->stp.ball_global[i] - this->dataPtr->param_server_node->parameters->stp.robot_global[i];
    Eigen::Rotation2D rotation(-this->dataPtr->param_server_node->parameters->stp.robot_global[2]);
    ball_field = rotation * ball_field;
    this->dataPtr->param_server_node->parameters->stp.ball_field[0] = ball_field[0],this->dataPtr->param_server_node->parameters->stp.ball_field[1] = ball_field[1];//球在机器人坐标系下的位置
    this->dataPtr->param_server_node->parameters->stp.ball_field[2] = this->dataPtr->param_server_node->parameters->stp.ball_global[2] - this->dataPtr->param_server_node->parameters->stp.robot_global[2];
    this->dataPtr->param_server_node->parameters->stp.ball_field_distance = ball_field.norm();
  }

  //检查是否倒地
  this->dataPtr->checkStableState(this->dataPtr->IOptr->get_rpy());
  if(this->dataPtr->m_stable_state != StableState::STABLE)
  {
    if(this->dataPtr->m_stable_state == StableState::FRONTDOWN) this->dataPtr->param_server_node->parameters->status_code = StatusCode::FORWARD_FALL_GETUP;
    else this->dataPtr->param_server_node->parameters->status_code = StatusCode::BACKWARD_FALL_GETUP;
    this->dataPtr->param_server_node->parameters->stp.see_ball = false;//摔倒后重置看见球的状态
  }
  
   
}

/////////////////////////////////
void MotionController::MotionControllerPrivate::checkStableState(std::vector<double> rpy)
{
  float deg_roll = rpy[0] * 180 / M_PI, deg_pitch = rpy[1] * 180 / M_PI,deg_yaw = rpy[2] * 180 / M_PI;
  // gzmsg << "deg roll = " << deg_roll << ", deg_pitch = " << deg_pitch << ", yaw = " << deg_yaw << std::endl;
  if (abs(deg_pitch) > 20|| abs(deg_roll) > 25) 
  {
    stablecount = 0;

    // TODO hard code ....
    if (deg_pitch > 40.0 && deg_pitch < 120.0)
    {
      ++frontcount;
      m_stable_state = UNSTABLE;
    }
    else
    {
      frontcount = 0;
    }

    if (deg_pitch < -40.0 && deg_pitch > -120.0) {
      ++backcount;
      m_stable_state = UNSTABLE;
    }
    else
    {
      backcount = 0;
    }

    if (deg_roll > 60 && deg_roll < 120)
    {
      ++rightcount;
      m_stable_state = UNSTABLE;
    }
    else
    {
      rightcount = 0;
    }

    if (deg_roll < -60 && deg_roll > -120)
    {
      ++leftcount;
      m_stable_state = UNSTABLE;
    }
    else
    {
      leftcount = 0;
    }
    }
    else
    {
      ++stablecount;
      frontcount = backcount = leftcount = rightcount = 0;
    }

    if (frontcount > STABLE_COUNT)      m_stable_state = FRONTDOWN;
    else if (backcount > STABLE_COUNT)  m_stable_state = BACKDOWN;
    else if (leftcount > STABLE_COUNT)  m_stable_state = LEFTDOWN;
    else if (rightcount > STABLE_COUNT) m_stable_state = RIGHTDOWN;
    else if (stablecount > 100)         m_stable_state = STABLE;

}

/////////////////////////////////
std::chrono::steady_clock::duration MotionController::MotionControllerPrivate::get_next_control_time()
{
  return this->nextControlTime;
}


/////////////////////////////////
void MotionController::MotionControllerPrivate::motion_fuse()
{
  std::unique_lock<mutex> planner_lock(this->planner_mutex);
  this->planner_ready = false;
  int len0 = motions.size(),len1 = temp_motions.size();
  auto it = motions.begin();
  //fusing
  for(int i = fusing_window_pos; i < len1 && i - fusing_window_pos < len0; ++it,++i)//the public part of motion and temp
  {
    if((*it)->get_status() != StatusCode::WALK) break;//如果最开始的动作不是行走，就不能融合
    std::shared_ptr<newPendulumWalk> old_motion = reinterpret_cast<std::shared_ptr<newPendulumWalk> &>(*it);
    old_motion->fuse(temp_motions[i]);
  }
  for(int i = fusing_window_pos + len0; i < len1; ++i) param_server_node->parameters->stp.tmp_gait = temp_motions[i],motions.push_back(std::make_shared<newPendulumWalk>(pendulum_global,param_server_node->parameters));
  temp_motions.clear();
  planner_lock.unlock();
}

/////////////////////////////////
void MotionController::MotionControllerPrivate::make_walk_plan(Eigen::Vector2d target_pos,double theta)
{
  std::unique_lock<std::mutex> planner_lock(planner_mutex,std::defer_lock);
  if(planner_lock.try_lock())
  {
    planner->plan(target_pos(0),target_pos(1),Deg2Rad(theta));
    store_plan();
    planner_lock.unlock();
    planner_ready = true;
  }
  return;
}

void MotionController::MotionControllerPrivate::store_plan()
{
  std::vector<Eigen::Vector3d> solution = planner->get_sol();
  int len = solution.size();
  gait_element new_gait;
  //store two stop gait

  if(walk_start < 5)
  {
    for(int i = 0; i + walk_start < 5; ++i)  solution[i] = solution[i] / 5 * (i + walk_start);
  } 
  //ensure robot is walking
  temp_motions.clear();
  for(int i = 0; i < len; ++i)
  {
    gait_element new_gait;
    new_gait.x = solution[i](0),new_gait.y = solution[i](1),new_gait.t = solution[i](2);
    if(new_gait.x > 0.001 || new_gait.t > 0.0001)    temp_motions.push_back(new_gait);
  }
}

GZ_ADD_PLUGIN(MotionController, System, MotionController::ISystemConfigure, MotionController::ISystemPreUpdate, MotionController::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(MotionController, "gz::sim::systems::motion_controller")