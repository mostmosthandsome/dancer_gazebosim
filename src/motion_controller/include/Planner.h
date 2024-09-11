#pragma once
#include <vector>
#include <mutex>
#include <Eigen/Dense>
#include "Parameters.h"
#include <memory>

namespace dplanner
{
    /**
     * @brief planner的模板，所有planner均需要继承自这个类，并实现所有接口
     */
    class BasePlanner
    {
    public:
        /// @brief 规划函数，给定目标位姿，求出一个合法的步态
        virtual void plan(double dest_x,double dest_y, double dest_t) = 0;
        void plan(std::vector<double> dest) {plan(dest[0],dest[1],dest[2]);}
        /**
         * @brief 返回求解出的dtheta
         * @return a vector in rad
         */
        std::vector< Eigen::Vector3d > get_sol() {return sol;}
    
    protected:
        std::vector< Eigen::Vector3d > sol;
    };

    class SimplePlanner: public BasePlanner
    {
    public:
        SimplePlanner(std::shared_ptr<dmotion::Parameters> _param);
        ~SimplePlanner();
        virtual void plan(double dest_x,double dest_y, double dest_t) override;

        class SimplePlannerPrivate;
    private:
        std::shared_ptr<SimplePlannerPrivate> dataPtr;


    };


    //这个类被注释掉化是因为，这其实是一个轨迹优化器
    // class Planner: public BasePlanner
    // {
    // public:
    //     /**
    //      * @brief construct a Planner
    //      * @param parameters the Parameters class in robot config
    //      * @param plan_horizon the length of the plan sequence of planner
    //     */
    //     Planner(int plan_horizon = 50);
    //     /**
    //      * @brief make a plan for target x, y, theta
    //      * @param dest_t the destiny angle in rad
    //      */
    //     virtual void plan(double dest_x,double dest_y, double dest_t) override;
    //     int get_horizon_len() {return plan_horizon_length;}


    // private:
    //     casadi::MX dynamics(casadi::MX x,casadi::MX sx, casadi::MX dtheta, bool is_Right,casadi::MX sx_old);
    //     casadi::MX obj(casadi::MX sx, casadi::MX dtheta);
    //     casadi::MX con_x(casadi::MX sx, casadi::MX dtheta);
    //     casadi::MX con_y(casadi::MX sx, casadi::MX dtheta);
    //     casadi::MX con_t(casadi::MX sx, casadi::MX dtheta);


    // private:
    //     double ankle_dis;
    //     int plan_horizon_length;
    //     double constraint_bound[30];
    //     double sy = 0.5;
    //     double target_x = 200, target_y = 0, target_t = 1.57;
    //     double max_step_x,max_step_yaw;
    //     //variable
    //     casadi::MX sx,dtheta = casadi::MX::sym("dtheta",50);
    //     //solution
    //     casadi::DM x0;
    //     //obj and constraints
    //     casadi::MX f,g1,g2,g3;
    //     //solver
    //     casadi::MXDict nlp;
    //     casadi::Function F;
    //     //limit
    //     casadi::DM lbx,ubx,lbg,ubg;
    //     casadi::Dict opts;
    // };

}
