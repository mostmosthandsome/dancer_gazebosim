#include "IO.h"
#include <string>
#include <gz/common/Profiler.hh>
#include "gz/math/PID.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/Name.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace zjudancer;

class gz::sim::systems::zjudancer::IOPrivate
{
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
};

IO::IO():dataPtr(std::make_unique<IOPrivate>()),joint_number(0)
{
    gzmsg << "IO create\n";
}

IO::~IO(){}

void IO::Init(const Entity &_entity,const std::shared_ptr<const sdf::Element> &_sdf,EntityComponentManager &_ecm,EventManager &_eventMgr)
{
    this->dataPtr->model = Model(_entity);//存储整个模型索引
    if (!this->dataPtr->model.Valid(_ecm))//检查ecm的合法性
    {
        gzerr << "ZJUDancer plugin should be attached to a model entity. " << "Failed to initialize." << std::endl;
        return;
    }
    auto joints = dataPtr->model.Joints(_ecm);
    joint_number = joints.size();
    std::string joint_name_list;
    for(int i = 0; i < joint_number; ++i)
    {
        auto name = _ecm.Component<components::Name>(joints[i]);
        this->dataPtr->jointNames.push_back(name->Data()),dataPtr->posPid.emplace_back(),actual_joint_name_index[name->Data()] = i;//add a joint and record its index
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
    std::string str = "";
    int cmd_joint_num = servo_angles.size();
    for(int i = 0; i < cmd_joint_num; ++i)
    {
        if(actual_joint_name_index.find(this->joint_order[i]) == actual_joint_name_index.end())
        {
            gzerr << "joint " << this->joint_order[i] << "not found in simulation, please check your urdf\n";
            return;
        }
        int index = actual_joint_name_index[this->joint_order[i]];
        //做一些安全性检查，并拿到JointPositioin
        auto jointPosComp = _ecm.Component<components::JointPosition>(this->dataPtr->jointEntities[index]);
        if (!jointPosComp)//如果没有就创建一个，然后让系统自己更新
        {
            _ecm.CreateComponent(this->dataPtr->jointEntities[index],components::JointPosition());
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
        str += "i = " + std::to_string(i) + "current_pos = " + std::to_string(current_pos) +  " error = " + std::to_string(error) + ",force = " + std::to_string(force) + " p = " + std::to_string(this->dataPtr->posPid[i].PGain()) +  ',';
        //获取力控组件
        Entity joint = this->dataPtr->jointEntities[index];
        auto forceComp = _ecm.Component<components::JointForceCmd>(joint);
        if (forceComp == nullptr)   _ecm.CreateComponent(joint,components::JointForceCmd({force}));
        else    *forceComp = components::JointForceCmd({force});
        // gzmsg << "force = " << force << std::endl;
    }
    // gzmsg << str <<  std::endl;
    
}
