#pragma once


#include "IO.h"
#include <gz/sim/System.hh>
#include <memory>
#include <gz/transport/Node.hh>
#include <thread>
#include "Parameters.h"


namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
namespace zjudancer
{


    class MotionController:public System, public ISystemConfigure, public ISystemPreUpdate, public ISystemPostUpdate
    {
    public:
        MotionController();
        ~MotionController() override;
        void Configure(const Entity &_entity,  const std::shared_ptr<const sdf::Element> &_sdf,   EntityComponentManager &_ecm,  EventManager &_eventMgr) override;
        
        /**
         * @brief 仿真步进前调用，用来更新关节的控制
        */
        void PreUpdate(const gz::sim::UpdateInfo &_info,gz::sim::EntityComponentManager &_ecm) override;
        
        /**
         * @brief 仿真步进后的调用，用来获取观测信息
         */
        void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) override;

        /**
         * @brief 上位机的motionhub.cpp里的程序，计算出一系列关节角，存储在dataPtr的actionList里
         * 步态核心！！！
         */
        void ComputeMotion();

        /// \brief How many times the controller will generate command per second , 单位HZ
        double control_frequency{100};

        class MotionControllerPrivate;
    private:
        //将乱七八糟的数据村到dataPtr里，方便阅读
        std::unique_ptr<MotionControllerPrivate> dataPtr;
        
    };

}
}
}
}
}
