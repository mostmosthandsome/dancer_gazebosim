#pragma once
#include "Parameters.h"
#include <string>

namespace dmotion
{
    class Climb
    {
    public:
        Climb(std::string label, std::vector<double> position_start,std::shared_ptr<Parameters> parameters,std::shared_ptr<zjudancer::simulation> env);
        void Prepare();
        void working();

    private:
        std::string label_;
        //示教过程中的关节位置和时间
        std::vector<std::vector<double> > AllPosition_time, AllPosition;
        std::vector<double> init_pose, value, time;
        std::vector<double> position_now;
        

        std::shared_ptr<Parameters> parameters;
        std::shared_ptr<zjudancer::simulation> env;
    };
}