#include "Parameters.h"
#include <thread>
#include <fstream>
#include <string>
using std::placeholders::_1;

namespace dmotion
{
    ParamNode::ParamNode():Node("parameter_server",rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
    {
        parameters = std::make_shared<Parameters>();
        this->ball_position_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>("ball",10,std::bind(&ParamNode::ball_position_callback,this,_1));
        parameters->config_path = "/home/handsome/competition/sim/src/motion_controller/config/";
        std::string motion_hub_param_path = parameters->config_path + "motion_hub_param.yaml",walk_param_path = parameters->config_path + "walk_param/foot_z.yaml";
        rclcpp::ParameterMap motion_hub_params = rclcpp::parameter_map_from_yaml_file(motion_hub_param_path), walk_params = rclcpp::parameter_map_from_yaml_file(walk_param_path);
        //将参数存进一个map里，方便处理
        std::unordered_map<std::string,rclcpp::Parameter> param_map;
        for(auto it : motion_hub_params)
        {
            std::vector<rclcpp::Parameter> paramVec = it.second;
            for(auto param : paramVec)  param_map[param.get_name()] = param;
        }
        for(auto it : walk_params)
        {
            std::vector<rclcpp::Parameter> paramVec = it.second;
            for(auto param : paramVec)  param_map[param.get_name()] = param;
        }

        //get parameters and store
        parameters->one_foot_landing_param.ANKLE_OFFSET_X = param_map["dmotion.OneFootLanding.ankle_offset_x"].as_double();
        parameters->one_foot_landing_param.ANKLE_OFFSET_Y = param_map["dmotion.OneFootLanding.ankle_offset_y"].as_double();
        parameters->one_foot_landing_param.ANKLE_OFFSET_Z = param_map["dmotion.OneFootLanding.ankle_offset_z"].as_double();
        parameters->one_foot_landing_param.BODY_CENTER_X = param_map["dmotion.OneFootLanding.body_center_x"].as_double();
        parameters->one_foot_landing_param.BODY_CENTER_Y = param_map["dmotion.OneFootLanding.body_center_y"].as_double();
        parameters->one_foot_landing_param.BODY_CENTER_Z = param_map["dmotion.OneFootLanding.body_center_z"].as_double();
        parameters->one_foot_landing_param.UPBODY_MASS = param_map["dmotion.OneFootLanding.upbody_mass"].as_double();
        parameters->one_foot_landing_param.FOOT_MASS = param_map["dmotion.OneFootLanding.foot_mass"].as_double();
        parameters->one_foot_landing_param.UPBODY_CONSTANT_ROLL = param_map["dmotion.OneFootLanding.upbody_constant_roll"].as_double();
        parameters->one_foot_landing_param.UPBODY_CONSTANT_PITCH = param_map["dmotion.OneFootLanding.upbody_constant_pitch"].as_double();

        //inverse_kinematics.cpp
        parameters->one_foot_landing_param.UPPER_LEG_LENGTH = param_map["dmotion.OneFootLanding.upper_leg_length"].as_double();
        parameters->one_foot_landing_param.LOWER_LEG_LENGTH = param_map["dmotion.OneFootLanding.lower_leg_length"].as_double();
        parameters->one_foot_landing_param.ANKLE_FROM_GROUND = param_map["dmotion.OneFootLanding.ankle_from_ground"].as_double();
        parameters->one_foot_landing_param.HALF_HIP_WIDTH = param_map["dmotion.OneFootLanding.half_hip_width"].as_double();
        parameters->one_foot_landing_param.HIP_X_FROM_ORIGIN = param_map["dmotion.OneFootLanding.hip_x_from_origin"].as_double();
        parameters->one_foot_landing_param.HIP_Z_FROM_ORIGIN = param_map["dmotion.OneFootLanding.hip_z_from_origin"].as_double();
        parameters->pendulum_walk_param.COM_X_OFFSET = param_map["dmotion.PendulumWalk.com_x_offset"].as_double();
        parameters->pendulum_walk_param.ANKLE_DIS = param_map["dmotion.PendulumWalk.ankle_dis"].as_double();
        parameters->pendulum_walk_param.COM_HEIGHT = param_map["dmotion.PendulumWalk.com_height"].as_double();    
        parameters->stp.UPARM_ANGLE = param_map["dmotion.Status.uparm_angle"].as_double();
        parameters->stp.LOWARM_ANGLE = param_map["dmotion.Status.lowarm_angle"].as_double();
        
        //ThreeInterpolation.cpp
        parameters->three_interpolation_param.DEFAULT_POINT_INTERVAL = param_map["dmotion.ThreeInterpolation.default_point_interval"].as_double();
        parameters->three_interpolation_param.DEFAULT_BOUNDARY_SLOPE = param_map["dmotion.ThreeInterpolation.default_boundary_slope"].as_double();

        //PendulumWalk.cpp
        parameters->pendulum_walk_param.foot_z_t = param_map["dmotion.foot_z.t"].as_double_array();
        parameters->pendulum_walk_param.foot_z_p = param_map["dmotion.foot_z.p"].as_double_array();
        parameters->pendulum_walk_param.foot_z_s = param_map["dmotion.foot_z.s"].as_double_array();
        parameters->pendulum_walk_param.TAO = param_map["dmotion.PendulumWalk.tao"].as_double();
        parameters->pendulum_walk_param.COM_H = param_map["dmotion.PendulumWalk.com_h"].as_double();
        parameters->pendulum_walk_param.Y_HALF_AMPLITUDE = param_map["dmotion.PendulumWalk.y_half_amplitude"].as_double();
        parameters->pendulum_walk_param.ACC_COEF_X = param_map["dmotion.PendulumWalk.acc_coef_x"].as_double();
        parameters->pendulum_walk_param.TICK_NUM = param_map["dmotion.PendulumWalk.tick_num"].as_int();
        parameters->pendulum_walk_param.max_step_x = param_map["dmotion.PendulumWalk.max_step_x"].as_double();
        parameters->pendulum_walk_param.max_step_y_in = param_map["dmotion.PendulumWalk.max_step_y_in"].as_double();
        parameters->stp.adjust_max_x = param_map["dmotion.Status.adjust_max_x"].as_double();
        parameters->stp.adjust_max_y = param_map["dmotion.Status.adjust_max_y"].as_double();
        parameters->stp.adjust_max_yaw = param_map["dmotion.Status.adjust_max_yaw"].as_double();
        parameters->stp.adjust_max_step_num = param_map["dmotion.Status.adjust_max_step_num"].as_int();

        //Planner.cpp
        parameters->stp.stop_walk_dis =  param_map["dmotion.Status.stop_walk_dis"].as_double();
        parameters->pendulum_walk_param.slow_down_minus_x = param_map["dmotion.PendulumWalk.slow_down_minus_x"].as_double();
        parameters->pendulum_walk_param.slow_down_minus_y = param_map["dmotion.PendulumWalk.slow_down_minus_y"].as_double();
        parameters->pendulum_walk_param.slow_down_minus_yaw = param_map["dmotion.PendulumWalk.slow_down_minus_yaw"].as_double();
        parameters->stp.LADT_for_ball = param_map["dmotion.Status.LADT_for_ball"].as_double();
        parameters->stp.LAAT_for_ball = param_map["dmotion.Status.LAAT_for_ball"].as_double();
        parameters->stp.one_step_y = param_map["dmotion.Status.one_step_y"].as_double();
        parameters->stp.shit_rob_radius = param_map["dmotion.Status.shit_rob_radius"].as_double();




        //init ball at one of the penalty mark by default
        this->parameters->stp.ball_global = std::vector<double>({1.2,0,0});
        RCLCPP_INFO(get_logger(),"parameter server init successfully");
    }

    void ParamNode::ball_position_callback(geometry_msgs::msg::Vector3 msg)
    {
        if(msg.z == 1000)   this->parameters->stp.see_ball = false;
        this->parameters->stp.ball_global[0] = msg.x,this->parameters->stp.ball_global[1] = msg.y,this->parameters->stp.ball_global[2] = msg.z;
        this->parameters->stp.see_ball = true;
    }

    Parameters::Parameters()
    {
    }
}

