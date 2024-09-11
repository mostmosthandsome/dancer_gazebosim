#pragma once
#include <vector>
#include <mutex>
#include <casadi/casadi.hpp>
#include "Parameters.h"

namespace dmotion
{

    class Planner
    {
    public:
        /**
         * @brief construct a Planner
         * @param parameters the Parameters class in robot config
         * @param plan_horizon the length of the plan sequence of planner
        */
        Planner(int plan_horizon = 50);
        /**
         * @brief make a plan for target x, y, theta
         * @param dest_t the destiny angle in rad
         */
        void plan(double dest_x,double dest_y, double dest_t);
        void plan(std::vector<double> dest);
        std::vector<double> get_sx() {return step_length;}
        /**
         * @return a vector in rad
         */
        std::vector<double> get_dtheta() {return delta_ang;}
        int get_horizon_len() {return plan_horizon_length;}


    private:
        casadi::MX dynamics(casadi::MX x,casadi::MX sx, casadi::MX dtheta, bool is_Right,casadi::MX sx_old);
        casadi::MX obj(casadi::MX sx, casadi::MX dtheta);
        casadi::MX con_x(casadi::MX sx, casadi::MX dtheta);
        casadi::MX con_y(casadi::MX sx, casadi::MX dtheta);
        casadi::MX con_t(casadi::MX sx, casadi::MX dtheta);


    private:
        double ankle_dis;
        int plan_horizon_length;
        double constraint_bound[30];
        double sy = 0.5;
        double target_x = 200, target_y = 0, target_t = 1.57;
        double max_step_x,max_step_yaw;
        //variable
        casadi::MX sx,dtheta = casadi::MX::sym("dtheta",50);
        //solution
        casadi::DM x0;
        //obj and constraints
        casadi::MX f,g1,g2,g3;
        //solver
        casadi::MXDict nlp;
        casadi::Function F;
        //limit
        casadi::DM lbx,ubx,lbg,ubg;
        casadi::Dict opts;
        std::vector<double> step_length,delta_ang;
    };

}
