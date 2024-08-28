#pragma once


#include <gz/sim/System.hh>

#include <memory>
#include <vector>
#include <string>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
namespace zjudancer
{
  /**
   * @brief 用来真正和存储信息的类
   */
  class IOPrivate;

  /**
   * 用来计算位控算法的类
   */
  class IO
  {
  public:
    /**
     * @brief 构造函数
     */
    IO();
    ~IO();

    /**
     * @brief 预处理函数
     * @param _entity 
     */
    void Init(const Entity &_entity,const std::shared_ptr<const sdf::Element> &_sdf,EntityComponentManager &_ecm, EventManager&_eventMgr);

    /**
     * @brief 用来在物理仿真前计算力，给入仿真系统
     * @param servo_angles 关节角度，弧度制
     * @param _info 来自系统的信息，包含了是否停止，dt
     * @param _ecm 仿真系统的句柄
     */
    void ServoControl(std::vector<double> servo_angles, const gz::sim::UpdateInfo &_info,gz::sim::EntityComponentManager &_ecm);
    /// @brief 期望给的命令里关节的顺序
    std::vector<std::string> joint_order = {
      "left_hip_yaw",   "left_hip_roll",  "left_hip_pitch",   "left_knee",        "left_ankle_pitch",   "left_ankle_roll",
      "right_hip_yaw",  "right_hip_roll", "right_hip_pitch",  "right_knee",       "right_ankle_pitch",  "right_ankle_roll",
      "left_arm_upper", "left_arm_lower", "right_arm_upper",  "right_arm_lower",  "head_pitch",         "head_yaw"
    };
    /// @brief  实际仿真里的关节名称和下标的对应关系
    std::unordered_map<std::string, int> actual_joint_name_index;

  private:
    std::unique_ptr<IOPrivate> dataPtr;
    int joint_number;
  };

}
}
}   
}
}

