#include "Climb.h"
#include <fstream>
#include "ThreeInterpolation.h"
#include "Utility/dmotion_math.hpp"
#include "OneFootLanding.h"
#include "PendulumWalk.h"
#include "ForwardKinematics.h"
#define DATA_COL_NUM 19//前 

using namespace std;

namespace dmotion
{

void Climb::Prepare()
{
  std::string path = parameters->config_path + "climb_param/";
    //示教爬起过程
  std::ifstream infile;
  if (label_ == "BACK")  infile.open(path + "back_climb.txt", std::ios::in | std::ios::out);
  else if (label_ == "FORWARD") infile.open(path + "forward_climb.txt", std::ios::in | std::ios::out);
  std::vector<double> SinglePosition;
  bool flag = true;//judge if read successfully
  while (true)
  {
    double buffer; SinglePosition.clear();
    for (int i = 0; i < DATA_COL_NUM && flag; i++)
    {
      if(infile >> buffer)  SinglePosition.push_back(buffer);
      else flag = false;
    } 
    if(!flag) break;
    AllPosition_time.push_back(SinglePosition),AllPosition = AllPosition_time;
    for (unsigned i = 0; i < AllPosition.size(); i++) AllPosition[i].pop_back();
    init_pose = AllPosition[0];
  }
  infile.close();
  // RCLCPP_INFO(rclcpp::get_logger("Climb"),"load file succeessfully");
}

Climb::Climb(std::shared_ptr< std::queue< std::vector<double> > > _action_list, std::string label, std::shared_ptr<Parameters> _parameters):parameters(_parameters),label_(label),action_list(_action_list)
{
  label_ = label,value.resize(2),time.resize(2);
  //从IO订阅消息得到跌倒时的舵机值
  position_now = _parameters->stp.cur_servo_angles;
  //从跌倒状态规划到示教爬起开始姿态
  Prepare();

}

void Climb::working()
{
  std::vector<std::vector<double> > servo_position;//use to record all the calculate result of servo position
  std::vector<double> servo_points,position_aftclimb,angle_leftleg,angle_rightleg;
  servo_points.resize(DATA_COL_NUM - 1);

  //过渡段
  for (int i = 0; i < DATA_COL_NUM - 1; i++)
  {
    value[0] = position_now[i],value[1] = init_pose[i],time[0] = 0, time[1] = parameters->climb_param.WHOLE_TIME;
    ThreeInterpolation offset(parameters, time, value);
    auto servo_split = offset.GetPoints();
    servo_position.push_back(servo_split);
  }
  unsigned int len = servo_position[0].size();
  for (unsigned i = 0; i < len; ++i)
  {
    for (unsigned j = 0; j < DATA_COL_NUM - 1; ++j)  servo_points[j] = servo_position[j][i];
    this->action_list->push(servo_points);
  }

  //示教数据播放
  unsigned step = ((label_ == "BACK")?(parameters->stp.back_climb_speed):(parameters->stp.forward_climb_speed));
  for (unsigned i = 0; i < AllPosition.size(); i += step)
  {   
    this->action_list->push(AllPosition[i]);
    position_aftclimb = AllPosition[i];
  }

  //用正向运动学计算现在双脚间的距离
  std::vector<double> aa = position_aftclimb;
  angle_leftleg = {aa[6],aa[7],aa[8],aa[9],aa[10],aa[11]};
  angle_rightleg = {aa[0], aa[1], aa[2], aa[3], aa[4], aa[5]};
  dmotion::ForKin left_leg(parameters, angle_leftleg, false),right_leg(parameters, angle_rightleg, true);
  std::vector<double> lfoot2center = left_leg.result_vector,rfoot2center = right_leg.result_vector;
  dmotion::ForKinPlus body(lfoot2center, rfoot2center);
  std::vector<double> center2left = body.center2support,right2left = body.hang2support;
  parameters->stp.cur_ankle_dis = -right2left[1];

  //恢复到走路状态下的质心高度
  OneFootLanding Support(false,parameters);
  motion_tick tmptick;
  tmptick.upbody_pose.clear();
  tmptick.whole_com.clear();
  tmptick.hang_foot.clear();
  tmptick.time_stamp = 10000000 * 1; //10毫秒
  tmptick.upbody_pose.emplace_back(0);
  tmptick.upbody_pose.emplace_back(0);
  tmptick.upbody_pose.emplace_back(0);
  tmptick.whole_com.emplace_back(parameters->pendulum_walk_param.COM_X_OFFSET); //(x * 100 +1.5);
  tmptick.whole_com.emplace_back(-parameters->stp.cur_ankle_dis / 2.0);
  tmptick.whole_com.emplace_back(parameters->pendulum_walk_param.COM_HEIGHT); //0.308637

  tmptick.hang_foot.emplace_back(0); //(200 * x);
  tmptick.hang_foot.emplace_back(-parameters->stp.cur_ankle_dis);
  tmptick.hang_foot.emplace_back(0);
  tmptick.hang_foot.emplace_back(0);
  tmptick.hang_foot.emplace_back(0);
  tmptick.hang_foot.emplace_back(0);

  std::vector<double> whole_end_pos = Support.GetOneStep(tmptick.hang_foot, tmptick.whole_com, tmptick.upbody_pose);//逆运动学求解一下应该有的关节角

  auto FinalAdjustServoMatrix = dmotion::ServoTransition(parameters, position_aftclimb, whole_end_pos);
  for (unsigned i = 0; i < FinalAdjustServoMatrix.size(); i++)
  {
      action_list->push(FinalAdjustServoMatrix[i]);
  }
  Delay(action_list, 500000);



  gait_element tmp_gait;
  PendulumWalk walk_climb(action_list, parameters, parameters->stp.cur_ankle_dis);
  tmp_gait.isRight = true;
  tmp_gait.label = "recover_from_climb";
  tmp_gait.x = 0;
  tmp_gait.y = 0;
  tmp_gait.t = 0;
  walk_climb.GiveAStepTick(tmp_gait);
  
  Delay(action_list, 700000);
}

}