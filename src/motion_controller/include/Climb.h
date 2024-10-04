#pragma once
#include "Parameters.h"
#include <string>
#include <queue>
#include <memory>

namespace dmotion
{
    class Climb
    {
    public:
        Climb(std::shared_ptr< std::queue< std::vector<double> > > _action_list, std::string label, std::shared_ptr<Parameters> parameters);
        /**
         * @brief 三次插值
         */
        void Prepare();
        /**
         * @brief 数据分发和走一步
         */
        void working();

    private:
        std::string label_;
        //示教过程中的关节位置和时间
        std::vector<std::vector<double> > AllPosition_time, AllPosition;
        std::vector<double> init_pose, value, time;
        std::vector<double> position_now;
        

        std::shared_ptr<Parameters> parameters;
        std::shared_ptr< std::queue< std::vector<double> > > action_list;
    };
}