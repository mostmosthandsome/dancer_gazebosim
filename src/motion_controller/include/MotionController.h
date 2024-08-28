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


    class MotionController:public System, public ISystemConfigure, public ISystemPreUpdate
    {
    public:
        MotionController();
        ~MotionController() override;
        void Configure(const Entity &_entity,  const std::shared_ptr<const sdf::Element> &_sdf,   EntityComponentManager &_ecm,  EventManager &_eventMgr) override;
        void PreUpdate(const gz::sim::UpdateInfo &_info,gz::sim::EntityComponentManager &_ecm) override;
        
        class MotionControllerPrivate;
    private:
        std::unique_ptr<MotionControllerPrivate> dataPtr;
        
    };

}
}
}
}
}
