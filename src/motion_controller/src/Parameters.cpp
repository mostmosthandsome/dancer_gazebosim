#include "Parameters.h"
#include <thread>
#include <fstream>
#include <string>

namespace dmotion
{
    ParamNode::ParamNode():Node("parameter_server",rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
    {
        parameters = std::make_shared<Parameters>();
        std::string motion_hub_param_path = "/home/handsome/competition/sim/src/motion_controller/config/motion_hub_param.yaml";
        rclcpp::ParameterMap params = rclcpp::parameter_map_from_yaml_file(motion_hub_param_path);
        std::unordered_map<std::string,rclcpp::Parameter> param_map;
        for(auto it : params)
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


        RCLCPP_INFO(get_logger(),"parameter server init successfully");
    }

    Parameters::Parameters()
    {
    }
}

