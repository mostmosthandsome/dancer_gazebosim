#include "Climb.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include "ThreeInterpolation.h"
#include "Utility/dmotion_math.hpp"
#define DATA_COL_NUM 17//前 

using namespace std;

namespace dmotion
{

  void Climb::Prepare()
  {
    RCLCPP_INFO(rclcpp::get_logger("Climb"),"loading climb file");
    std::string path = ament_index_cpp::get_package_share_directory("climb") + "/config/climb_param/";
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
    RCLCPP_INFO(rclcpp::get_logger("Climb"),"load file succeessfully");
  }

  Climb::Climb(std::string label, std::vector<double> position_start,std::shared_ptr<Parameters> _parameters,std::shared_ptr<zjudancer::simulation> _env):parameters(_parameters),env(_env),label_(label)
  {
    label_ = label,value.resize(2),time.resize(2);
    //从IO订阅消息得到跌倒时的舵机值
    position_now = position_start;
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
    std::ofstream f;
    f.open("/home/handsome/research/deep_research/sim/gazebo/motion/src/climb/record_data/all_data.txt",std::ios::out);
    unsigned int len = servo_position[0].size();
    for (unsigned i = 0; i < len; ++i)
    {
      for (unsigned j = 0; j < DATA_COL_NUM - 1; ++j)  servo_points[j] = servo_position[j][i];
      env->step(servo_points);
      for(auto it : servo_points)  f << it << ' ';
      f << char(10);
    }

    //示教数据播放
    unsigned step = ((label_ == "BACK")?(parameters->stp.back_climb_speed):(parameters->stp.forward_climb_speed));
    for (unsigned i = 0; i < AllPosition.size(); i += step)
    {   
      env->step(AllPosition[i]);
      position_aftclimb = AllPosition[i];
      for(auto it : AllPosition[i])  f << it << ' ';
      f << char(10);
    }
    

    OneFootLanding Support(false);
    motion_tick tmptick;
    tmptick.upbody_pose = {0,0,0};
    tmptick.whole_com = {parameters->pendulum_walk_param.COM_X_OFFSET, -parameters->stp.cur_ankle_dis / 2.0, parameters.pendulum_walk_param.COM_HEIGHT};

    tmptick.hang_foot = {0,-parameters->stp.cur_ankle_dis);
      // tmptick.hang_foot.emplace_back(0);
      // tmptick.hang_foot.emplace_back(0);
      // tmptick.hang_foot.emplace_back(0);
      // tmptick.hang_foot.emplace_back(0);

      // whole_end_pos = Support.GetOneStep(tmptick.hang_foot, tmptick.whole_com, tmptick.upbody_pose);

      // FinalAdjustServoMatrix = dmotion::ServoTransition(position_aftclimb, whole_end_pos);
      // for (unsigned i = 0; i < FinalAdjustServoMatrix.size(); i++)
      // {
      //     ServoPublish(FinalAdjustServoMatrix[i], "WALK", pbr, lopr);
      // }
      // Delay(1000000);

      // gait_element tmp_gait;
      // PendulumWalk walk_climb(pbr, lopr, parameters.stp.cur_ankle_dis);
      // tmp_gait.isRight = true;
      // tmp_gait.label = "recover_from_climb";
      // tmp_gait.x = 0;
      // tmp_gait.y = 0;
      // tmp_gait.t = 0;
      // walk_climb.GiveAStepTick(tmp_gait);
      // Delay(700000);
  }

}