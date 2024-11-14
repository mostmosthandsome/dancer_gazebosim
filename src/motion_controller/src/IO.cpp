#include "IO.h"
#include <string>
#include <gz/common/Profiler.hh>
#include "gz/math/PID.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Imu.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/LinearAcceleration.hh"

#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/ImuSensor.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace zjudancer;

class gz::sim::systems::zjudancer::IOPrivate
{
public:
    void IMUInit(const EntityComponentManager &_ecm, const Entity _entity, const components::Imu *_imu, const components::ParentEntity *_parent);
    void UpdateOdometry(const gz::sim::UpdateInfo &_info,    const gz::sim::EntityComponentManager &_ecm);
    void UpdateIMU(const gz::sim::UpdateInfo &_info, const EntityComponentManager &_ecm);

    /// \brief Joint Entity
    public: std::vector<Entity> jointEntities;

    /// \brief Joint name
    public: std::vector<std::string> jointNames;

    /// \brief mutex to protect joint commands
    public: std::mutex jointCmdMutex;

    /// \brief Model interface
    public: Model model{kNullEntity};

    /// \brief Position PID controller.
    public: std::vector< math::PID > posPid;

    /// \brief 为了防止以后可能单个机器人出现多个imu
    public: std::unordered_map<Entity,std::unique_ptr<sensors::ImuSensor>> entitySensorMap;

    /// \brief 允许特定的xyz和rpy的offset， 加在里程计偏置上
    public: gz::math::Pose3d offset = {0, 0, 0, 0, 0, 0};

    public: gz::math::Pose3d current_pose = {0, 0, 0, 0, 0, 0};

    /// \brief Keep track of world ID, which is equivalent to the scene's root visual.
    /// Defaults to zero, which is considered invalid by Gazebo.
    public: Entity worldEntity = kNullEntity;
    
    /// \brief gz-sensors sensor factory for creating sensors
    public: sensors::SensorFactory sensorFactory;
};

IO::IO():dataPtr(std::make_unique<IOPrivate>()),joint_number(0)
{
    gzmsg << "IO create\n";
}

IO::~IO(){}

void IO::Init(const Entity &_entity,const std::shared_ptr<const sdf::Element> &_sdf,EntityComponentManager &_ecm,EventManager &_eventMgr)
{
    this->dataPtr->model = Model(_entity);//存储整个模型索引
    std::string robot_name = this->dataPtr->model.Name(_ecm);
    if (!this->dataPtr->model.Valid(_ecm))//检查ecm的合法性
    {
        gzerr << "ZJUDancer plugin should be attached to a model entity. " << "Failed to initialize." << std::endl;
        return;
    }
    gzmsg << "IO Init starts\n";

    //从_ecm中找到所有imu，初始化其中属于当前机器人的imu
    _ecm.Each<components::Imu, components::ParentEntity>(
      [&](const Entity &_entity,const components::Imu *_imu,const components::ParentEntity *_parent)->bool
        {
            Entity parent = _ecm.ParentEntity(_entity);
            while(parent != kNullEntity)
            {
                if(robot_name == _ecm.Component<components::Name>(parent)->Data())
                {
                    // gzerr << " imu in robot " << robot_name << " is ready to init\n";
                    this->dataPtr->IMUInit(_ecm, _entity, _imu, _parent);
                    break;
                }
                parent = _ecm.ParentEntity(parent);
            }
            
            return true;
        });

    auto joints = dataPtr->model.Joints(_ecm);
    // std::cout << "Joints: " << std::endl;
    // for (const auto &joint : joints)
    // {
    //     std::cout << "Joint Entity ID: " << joint << std::endl;
    // }

    joint_number = joints.size();
    std::string joint_name_list;
    for(int i = 0; i < joint_number; ++i)
    {
        auto name = _ecm.Component<components::Name>(joints[i]);
        // std::cout << name->Data() << std::endl;
        this->dataPtr->jointNames.push_back(name->Data());
        dataPtr->posPid.emplace_back();
        actual_joint_name_index[name->Data()] = i;//add a joint and record its index
        joint_name_list += name->Data() + ' ';
    }
    double p = 1, i = 0.1, d = 0.01;//pid参数
    double iMax = 1, iMin = -1;//积分器的上下限，防止积分项过大
    double cmdMax = 1000, cmdMin = -1000;//限位
    double cmdOffset =  0;//命令的零位
    if (_sdf->HasElement("p_gain")) p = _sdf->Get<double>("p_gain");
    if (_sdf->HasElement("i_gain")) i = _sdf->Get<double>("i_gain");
    if (_sdf->HasElement("d_gain")) d = _sdf->Get<double>("d_gain");
    if (_sdf->HasElement("cmd_max")) cmdMax = _sdf->Get<double>("cmd_max");
    if (_sdf->HasElement("cmd_min")) cmdMin = _sdf->Get<double>("cmd_min");
    for(int i = 0; i < joint_number; ++i)    this->dataPtr->posPid[i].Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
    gzmsg << "IO Init successfully! " << " joint order : " << joint_name_list << std::endl ;
}

void IO::ServoControl(std::vector<double> servo_angles, const gz::sim::UpdateInfo &_info,gz::sim::EntityComponentManager &_ecm)
{
    if (_info.dt < std::chrono::steady_clock::duration::zero())//如果时钟有问题，提出warning
    {
        gzwarn << "Detected jump back in time ["
            << std::chrono::duration<double>(_info.dt).count()
            << "s]. System may not work properly." << std::endl;
    }

     // If the joints haven't been identified yet, look for them
    if (this->dataPtr->jointEntities.empty())
    {
        bool warned{false};
        for (const std::string &name : this->dataPtr->jointNames)
        {
            Entity joint = this->dataPtr->model.JointByName(_ecm, name);
            if (joint != kNullEntity)
            {
                this->dataPtr->jointEntities.push_back(joint);
            }
            else if (!warned)
            {
                gzwarn << "Failed to find joint [" << name << "]" << std::endl;
                warned = true;
            }
        }
    }
    // std::string str;

    int cmd_joint_num = servo_angles.size(), total_joint_number = this->joint_order.size();
    if (cmd_joint_num < total_joint_number)//如果有关节没有涉及到（比如头一般不控制）
    {
        for(int i = cmd_joint_num; i < total_joint_number; ++i) servo_angles.push_back(0);
    }
    for(int i = 0; i < total_joint_number; ++i)
    {
        if(actual_joint_name_index.find(this->joint_order[i]) == actual_joint_name_index.end())
        {
            gzerr << "joint " << this->joint_order[i] << "not found in simulation, please check your urdf\n";
            return;
        }
        int index = actual_joint_name_index[this->joint_order[i]];
        //做一些安全性检查，并拿到JointPosition
        auto jointPosComp = _ecm.Component<components::JointPosition>(this->dataPtr->jointEntities[index]);
        if (!jointPosComp)//如果没有就创建一个，然后让系统自己更新
        {
            _ecm.CreateComponent(this->dataPtr->jointEntities[index], components::JointPosition());
            return;
        }   
        //如果有但是还没数据，我们也返回
        if (jointPosComp->Data().empty())   return;
        double current_pos = -1;
        // 获取joint信息
        double error;
        {
            std::lock_guard<std::mutex> lock(this->dataPtr->jointCmdMutex);
            current_pos = jointPosComp->Data().at(0);
            error = current_pos - servo_angles[i];
        }
        // 根据pid计算力
        double force = this->dataPtr->posPid[index].Update(error, _info.dt);
        auto cmd = this->dataPtr->posPid[index].Cmd();
        // str += "i = " + std::to_string(i) + "current_pos = " + std::to_string(current_pos) +  " error = " + std::to_string(error) + ",force = " + std::to_string(force) + " p = " + std::to_string(this->dataPtr->posPid[i].PGain()) +  ',';
        //获取力控组件
        Entity joint = this->dataPtr->jointEntities[index];
        auto forceComp = _ecm.Component<components::JointForceCmd>(joint);
        if (forceComp == nullptr)  
            _ecm.CreateComponent(joint, components::JointForceCmd({force}));
        else    
            *forceComp = components::JointForceCmd({force});
    }
    // gzmsg << str <<  std::endl;
    
}


void IO::update_observation(const gz::sim::UpdateInfo &_info,    const gz::sim::EntityComponentManager &_ecm)
{
    this->dataPtr->UpdateOdometry(_info,_ecm);
    this->dataPtr->UpdateIMU(_info, _ecm);
}

std::vector<double> IO::get_robot_global()
{
    std::vector<double> robot_global;
    robot_global.push_back(this->dataPtr->current_pose.Pos().X());
    robot_global.push_back(this->dataPtr->current_pose.Pos().Y());
    robot_global.push_back(this->dataPtr->current_pose.Rot().Yaw() + M_PI / 2);
    return robot_global;
}

std::vector<double> IO::get_rpy()
{
    std::vector<double> robot_rpy;
    //理论上机器人里应当只有一个imu，所以这里其实只会迭代一次
    for (auto &it : this->dataPtr->entitySensorMap)
    {
        gz::math::v7::Quaternion<double> imu_world_pose = it.second->WorldPose().Rot(),robot_imu;
        robot_imu.SetFromAxisAngle(gz::math::v7::Vector3d::UnitZ,M_PI / 2);
        auto body_world_pose = imu_world_pose * robot_imu; 
        robot_rpy.push_back(body_world_pose.Roll()),robot_rpy.push_back(body_world_pose.Pitch()),robot_rpy.push_back(body_world_pose.Yaw());
    }
    return robot_rpy;
}

std::vector<double> IO::get_cur_servo_angles(const gz::sim::UpdateInfo &_info,const gz::sim::EntityComponentManager &_ecm)
{
    std::vector<double> joint_angles;
    int joint_num = this->joint_order.size();
    joint_angles.resize(joint_num);
    std::lock_guard<std::mutex> lock(this->dataPtr->jointCmdMutex);
    for(int i = 0; i < joint_num; ++i)
    {
        int index = actual_joint_name_index[this->joint_order[i]];
        auto jointPosComp = _ecm.Component<components::JointPosition>(this->dataPtr->jointEntities[index]);
        if (!jointPosComp || jointPosComp->Data().empty())  return std::vector<double>();//如果还没初始化好，返回空的vector
        joint_angles[i] = jointPosComp->Data().at(0);
    }
    return joint_angles;
}


/////////////////////////////////
void IOPrivate::UpdateOdometry(const gz::sim::UpdateInfo &_info,    const gz::sim::EntityComponentManager &_ecm)
{
    const math::Pose3d rawPose = worldPose(this->model.Entity(), _ecm);
    this->current_pose = rawPose * this->offset;
}

/////////////////////////////////
void IOPrivate::IMUInit(const EntityComponentManager &_ecm, const Entity _entity, const components::Imu *_imu, const components::ParentEntity *_parent)
{
    //先记录世界的重力
    if (kNullEntity == this->worldEntity)  this->worldEntity = _ecm.EntityByComponents(components::World());
    auto gravity = _ecm.Component<components::Gravity>(worldEntity);
    if (nullptr == gravity)
    {
        gzerr << "World missing gravity." << std::endl;
        return;
    }
    //通过加入scope的方式确保imu的名字不会和其他imu重复(因为我们默认一个link只能连接一个imu)
    std::string sensorScopedName = removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
    sdf::Sensor data = _imu->Data();
    data.SetName(sensorScopedName);
    std::unique_ptr<sensors::ImuSensor> sensor =  this->sensorFactory.CreateSensor<sensors::ImuSensor>(data);

    if (nullptr == sensor)
    {
        gzerr << "Failed to create sensor [" << sensorScopedName << "]"
            << std::endl;
        return;
    }

    std::string parentName = _ecm.Component<components::Name>(_parent->Data())->Data();
    sensor->SetParent(parentName);

    // set gravity - assume it remains fixed
    sensor->SetGravity(gravity->Data());

    // Get initial pose of sensor and set the reference z pos The WorldPose component was just created and so it's empty
    // We'll compute the world pose manually here
    math::Pose3d p = worldPose(_entity, _ecm);
    sensor->SetOrientationReference(p.Rot());

  // Get world frame orientation and heading. 
  // If <orientation_reference_frame> includes a named frame like NED, that must be supplied to the IMU sensor,
  // otherwise orientations are reported w.r.t to the initial orientation.
    // if (data.Element()->HasElement("imu")) 
    // {
    //     auto imuElementPtr = data.Element()->GetElement("imu");
    //     if (imuElementPtr->HasElement("orientation_reference_frame"))
    //     {

    //         sensor->SetWorldFrameOrientation(math::Quaterniond(0, 0, heading), gz::sensors::WorldFrameEnumType::ENU);
    //     }
    // }

    // Set whether orientation is enabled
    if (data.ImuSensor())    sensor->SetOrientationEnabled(data.ImuSensor()->OrientationEnabled());

    this->entitySensorMap.insert(std::make_pair(_entity, std::move(sensor)));
}

/////////////////////////////////
void IOPrivate::UpdateIMU(const gz::sim::UpdateInfo &_info,const EntityComponentManager &_ecm)
{
    _ecm.Each<components::Imu,
            components::WorldPose,
            components::AngularVelocity,
            components::LinearAcceleration>(
    [&](const Entity &_entity,
        const components::Imu * /*_imu*/,
        const components::WorldPose *_worldPose,
        const components::AngularVelocity *_angularVel,
        const components::LinearAcceleration *_linearAccel)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          const auto &imuWorldPose = _worldPose->Data();
        //   gzerr << "imu world pose : " << imuWorldPose.X() << "," << imuWorldPose.Y() << "," << imuWorldPose.Z() << "," << imuWorldPose.Roll() << "," << imuWorldPose.Pitch() << "," << imuWorldPose.Yaw() << std::endl;
          
          it->second->SetWorldPose(imuWorldPose);

          // Set the IMU angular velocity (defined in imu's local frame)
          it->second->SetAngularVelocity(_angularVel->Data());

          // Set the IMU linear acceleration in the imu local frame
          it->second->SetLinearAcceleration(_linearAccel->Data());
        }
        return true;
      });

    for (auto &it : this->entitySensorMap)
    {
      // Update measurement time
      it.second->Update(_info.simTime, false);
    }
}