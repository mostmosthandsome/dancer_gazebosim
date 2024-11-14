#include "sim_env/joint_reader.h"
#include <iostream>
#include <string>

using std::placeholders::_1;
#define JOINT_NUM 20

namespace handsome
{
    JointReaderPrivate::JointReaderPrivate(int id): Node("joint_" + std::to_string(id) + "reader"),sec(0),nano_sencond(0),updated(false)
    {
        sub = this->create_subscription<sensor_msgs::msg::JointState>("rotor" + std::to_string(id) + "_state",10,
        std::bind(&JointReaderPrivate::write_angle,this,_1));
        //reserved for joint observation
        // f.open("/home/handsome/research/deep_research/sim/gazebo/motion/src/climb/record_data/joint" + std::to_string(id) + ".txt",std::ios::out);
        angle.store(0);
    }

    JointReaderPrivate::~JointReaderPrivate()
    {
        f.close();
    }

    void JointReaderPrivate::write_angle(const sensor_msgs::msg::JointState &msg)
    {
        // sec = msg.header.stamp.sec, nano_sencond = msg.header.stamp.nanosec;
        // for(auto it : msg.position)   f << sec + 1.0 * nano_sencond / 1000000000 << ',' << it << std::endl;
        angle.store(msg.position[0] / M_PI * 180),updated.store(true);
    }

    JointReader::JointReader()
    {
        for(int i = 0; i < JOINT_NUM; ++i)
        {
            joint_reader_ptr[i] = std::make_shared< JointReaderPrivate >(i);
            // read_thread[i] = std::thread([=]()->void{
            //     rclcpp::spin(joint_reader_ptr[i]);
            // });
        }
    }

    JointReader::~JointReader()
    {
        for(int i = 0; i < JOINT_NUM; ++i)
            if(read_thread[i].joinable())
                read_thread[i].join();
    }

    std::vector<double> JointReader::get_joint_angle()
    {
        std::vector<double> angles;
        for(int i = 0; i < JOINT_NUM; ++i)    joint_reader_ptr[i]->updated.store(false), rclcpp::spin_some(joint_reader_ptr[i]);
        bool waiting_flag = true;
        while(waiting_flag)
        {
            waiting_flag = false;
            for(int i = 0; i < JOINT_NUM; ++i)
                if(!joint_reader_ptr[i]->updated.load())
                    waiting_flag = true;
        }
        for(int i = 0; i < JOINT_NUM; ++i) angles.push_back(joint_reader_ptr[i]->angle.load());
        return angles;
    }
}
