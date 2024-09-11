#include "Planner.h"
#include "Utility/dmotion_math.hpp"

using namespace dmotion;
namespace dplanner
{

SimplePlanner::SimplePlanner(std::shared_ptr<Parameters> _param):dataPtr(std::make_shared<SimplePlannerPrivate>(_param))
{

}

class SimplePlanner::SimplePlannerPrivate
{
public:
    SimplePlannerPrivate(std::shared_ptr<Parameters> _param):parameters(_param) {}
    gait_element GenerateNewGait(double aim_field_angle, gait_element last_gait_element, double error_aim_angle = 1000);
    gait_element cal_the_next_gait( gait_element last_gait ,gait_element target_gait);
    void SlowDown();
    void LittleAdjust(double start_x, double start_y, double start_angle, double end_x, double end_y, double end_angle);
    /** 
     * 用于任意坐标系下两步相对于机器人GiveAStep参数的生成，这里使用了比较输入为世界坐标系下的<x,y,yaw>起点与终点
     * 输出为相对于此时机器人的位置下一步参数,可以用于GiveAStep
     * 
     * */
    std::vector<double> OneStepTransform(double global_start_x, double global_start_y, double global_start_yaw, double global_end_x, double global_end_y, double global_end_yaw);

    std::shared_ptr<Parameters> parameters;
    int walk_to_ball_state{0};
    bool walk_start;
    double last_angle_e,error_direction;
    double shit_y_use,shit_yaw_use;  
    int shit_num;
};

Eigen::Vector3d gait2Vector(gait_element tmp_gait)
{
    return Eigen::Vector3d(tmp_gait.x,tmp_gait.y,tmp_gait.t);
}


void SimplePlanner::plan(double dest_x,double dest_y, double dest_t)
{
    this->dataPtr->parameters->stp.gait_queue.clear();
    if ((dataPtr->walk_to_ball_state <= 1 && dataPtr->parameters->stp.ball_field_distance > dataPtr->parameters->stp.stop_walk_dis) ||
        dataPtr->parameters->stp.ball_field_distance > dataPtr->parameters->stp.stop_walk_dis * 2) //如果距离过远，就按照几何约束前进
    {
        dataPtr->walk_to_ball_state = 1;
        dataPtr->parameters->stp.tmp_gait = dataPtr->GenerateNewGait(dataPtr->parameters->stp.ball_field_angle, dataPtr->parameters->stp.last_gait);
        this->dataPtr->parameters->stp.gait_queue.push_back(dataPtr->parameters->stp.tmp_gait);
    }
    else if ((dataPtr->walk_to_ball_state <= 2 && abs(dataPtr->parameters->stp.ball_field_angle) > dataPtr->parameters->stp.LAAT_for_ball) || (abs(dataPtr->parameters->stp.ball_field_angle) > dataPtr->parameters->stp.LAAT_for_ball * 3)) 
    {
        dataPtr->walk_to_ball_state = 2;//turn around;
        double error_angle = AdjustDegRange2(dataPtr->parameters->stp.ball_field_angle);
        dataPtr->parameters->stp.tmp_gait.x = 0;
        dataPtr->parameters->stp.tmp_gait.y = 0;
        dataPtr->parameters->stp.tmp_gait.t = sign(error_angle) * 8;
        dataPtr->parameters->stp.tmp_gait = dataPtr->cal_the_next_gait(dataPtr->parameters->stp.last_gait, dataPtr->parameters->stp.tmp_gait);
    }
    else if(dataPtr->walk_to_ball_state<=3&&(dataPtr->parameters->stp.ball_field_distance > dataPtr->parameters->stp.LADT_for_ball || 
    abs(AdjustDegRange2(dataPtr->parameters->stp.action_global[2] - dataPtr->parameters->stp.robot_global[2])) > dataPtr->parameters->stp.LAAT_for_ball) )
    {
        dataPtr->walk_to_ball_state=3;
        if (dataPtr->parameters->stp.gait_queue.size() == 0) 
        {
            if (dataPtr->walk_start) 
            {//第一次进入
                double angle_error = AdjustDegRange2(dataPtr->parameters->stp.action_global[2] - dataPtr->parameters->stp.robot_global[2]);
                dataPtr->last_angle_e = angle_error;
                if (abs(angle_error) > 10) 
                {
                    
                    if (angle_error > 0)  dataPtr->error_direction = 1;
                    else  dataPtr->error_direction = -1;
                    dataPtr->shit_y_use = -dataPtr->parameters->stp.one_step_y * dataPtr->error_direction;
                    dataPtr->shit_yaw_use = dataPtr->error_direction * Rad2Deg(dataPtr->parameters->stp.one_step_y / dataPtr->parameters->stp.shit_rob_radius);;
                } 
                else if(abs(angle_error) < dataPtr->parameters->stp.LAAT_for_ball && dataPtr->parameters->stp.ball_field_distance < dataPtr->parameters->stp.LADT_for_ball&& dataPtr->parameters->stp.ball_field[0]>5)
                {
                    dataPtr->SlowDown();
                    dataPtr->parameters->status_code = (StatusCode)314;
                    dataPtr->walk_start = true;
                }
                else
                {
                    dataPtr->walk_start=true;
                    dataPtr->walk_to_ball_state=1;//重新进入状态一　走向球
                }
                dataPtr->shit_num = 1;
            }
            else
            {
                double shit_y_send;
                if (dataPtr->shit_num <= 4) 
                {
                    shit_y_send = dataPtr->shit_num / 4.0 * dataPtr->shit_y_use;
                    dataPtr->shit_num++;
                }
                double shit_x_add = 0,shit_yaw_add = 0;
                if (dataPtr->parameters->stp.ball_field_distance > dataPtr->parameters->stp.shit_rob_radius)  shit_x_add += dataPtr->parameters->stp.dx_delta;
                else shit_x_add -= dataPtr->parameters->stp.dx_delta;
                if (dataPtr->parameters->stp.ball_field_angle > 0) shit_yaw_add += dataPtr->parameters->stp.dyaw_delta;
                else   shit_yaw_add -= dataPtr->parameters->stp.dyaw_delta;

                double shit_x = (1 - cos(dataPtr->parameters->stp.one_step_y / dataPtr->parameters->stp.shit_rob_radius));
                dataPtr->parameters->stp.tmp_gait.x =  shit_x + shit_x_add;
                dataPtr->parameters->stp.tmp_gait.y = shit_y_send;
                dataPtr->parameters->stp.tmp_gait.t = dataPtr->shit_yaw_use + shit_yaw_add;
                dataPtr->parameters->stp.tmp_gait.isRight =
                        !dataPtr->parameters->stp.tmp_gait.isRight;
                this->dataPtr->parameters->stp.gait_queue.push_back(dataPtr->parameters->stp.tmp_gait);
                }
                    // if (dataPtr->parameters->stp.ball_field_distance >
                    //     dataPtr->parameters->stp.stop_walk_dis) {

                    //     //dataPtr->parameters->status_code = (StatusCode)312;
                    //     dataPtr->walk_to_ball_state = 1;
                    //     walk_start = true;
                    //     ROS_FATAL("1.1.1.3");
                    // }
                double angle_e = AdjustDegRange2(dataPtr->parameters->stp.action_global[2] - dataPtr->parameters->stp.robot_global[2]);
                //当盘着盘着发现error_angle
                //小于了5度或者error_angle的符号改变了的时候，即可进入小调整了
                if (((abs(angle_e) < 5 || (sign(dataPtr->last_angle_e) != sign(angle_e) && abs(angle_e) < 40)) )&& dataPtr->parameters->stp.ball_field_distance < dataPtr->parameters->stp.stop_walk_dis&& dataPtr->parameters->stp.ball_field[0]>5) 
                {
                    dataPtr->SlowDown();
                    dataPtr->walk_to_ball_state = 4;
                    dataPtr->walk_start = true;
                }
                dataPtr->last_angle_e = angle_e;
            }
        else {  // 没看到球就进入400等着
            dataPtr->parameters->status_code = (StatusCode)400;
        }

        if(dataPtr->parameters->stp.ball_field_distance > dataPtr->parameters->stp.LADT_for_ball || (abs(dataPtr->parameters->stp.ball_field_angle) > dataPtr->parameters->stp.LAAT_for_ball * 3))  dataPtr->walk_to_ball_state = 1;
    }
    else 
    {
        dataPtr->walk_to_ball_state = 4;
        if (dataPtr->parameters->stp.gait_queue.size() == 0) 
        {
            double start_x = 0,start_y = 0,start_angle = 0;
            double end_angle = AdjustDegRange2(dataPtr->parameters->stp.action_global[2] - dataPtr->parameters->stp.robot_global[2]);
            double end_x = dataPtr->parameters->stp.ball_field[0] +
                    dataPtr->parameters->kick_param.RIGHT_KICK_X * cos(Deg2Rad(end_angle)) -
                    dataPtr->parameters->kick_param.RIGHT_KICK_Y * sin(Deg2Rad(end_angle));
            double end_y = dataPtr->parameters->stp.ball_field[1] +
                    dataPtr->parameters->kick_param.RIGHT_KICK_Y * cos(Deg2Rad(end_angle)) +
                    dataPtr->parameters->kick_param.RIGHT_KICK_X * sin(Deg2Rad(end_angle));
        // dmotion::AdjustDegRange(dataPtr->parameters->stp.ball_global[2]);
            dataPtr->LittleAdjust(start_x, start_y, start_angle, end_x, end_y, end_angle);
            dataPtr->parameters->stp.tmp_gait.isRight = !dataPtr->parameters->stp.last_gait.isRight;
            dataPtr->parameters->stp.tmp_gait.label = "before kick";
            dataPtr->parameters->stp.tmp_gait.x = 0;
            dataPtr->parameters->stp.tmp_gait.y = 0;
            dataPtr->parameters->stp.tmp_gait.t = 0;
            this->dataPtr->parameters->stp.gait_queue.push_back(dataPtr->parameters->stp.tmp_gait);
        if(dataPtr->parameters->stp.see_ball&&(dataPtr->parameters->stp.ball_field_distance <dataPtr->parameters->stp.LADT_for_ball &&
            abs(AdjustDegRange2(dataPtr->parameters->stp.action_global[2] -
                                dataPtr->parameters->stp.robot_global[2])) <
                dataPtr->parameters->stp.LAAT_for_ball &&
            dataPtr->parameters->stp.ball_field_angle <
                dataPtr->parameters->stp
                    .LAAT_for_ball)) 
        {
            dataPtr->parameters->stp.tmp_gait.isRight = !dataPtr->parameters->stp.last_gait.isRight;
            this->dataPtr->parameters->stp.gait_queue.push_back(dataPtr->parameters->stp.tmp_gait);
            dataPtr->parameters->status_code = (StatusCode) 220;
        }
        }
    }
    for(auto it : this->dataPtr->parameters->stp.gait_queue)   sol.push_back(gait2Vector(it));
}

/**
 * @brief 使用几何法构建速度限制的离散步态速度控制器
 * @param aim_field_angle 是目标点方向相对于现在自己面对的方向的角度差,需要是-180度到180度之间的值
 * @param last_gait_element 上一步的数据
 * */
gait_element SimplePlanner::SimplePlannerPrivate::GenerateNewGait(double aim_field_angle, gait_element last_gait_element, double error_aim_angle)
{
    gait_element result_gait;
    aim_field_angle = AdjustDegRange2(aim_field_angle); //调整aim_field_angle的范围

    bool this_is_right = !last_gait_element.isRight; //这一步的迈步脚应与上一步不同
    //先获得希望速度方向和最大速度四边形边界的交点
    double hope_x, hope_y;
    double x_cut, y_cut; //速度边界大菱形的目标方向的边界直线x、y轴截距
    aim_field_angle = AdjustDegRange2(aim_field_angle);
    aim_field_angle = Deg2Rad(aim_field_angle);

    // FIXME: DONT use == for double/float type(check like `abs(num-target) < .0001)
    if (aim_field_angle == 0)
    {
        hope_x = parameters->pendulum_walk_param.max_step_x;
        hope_y = 0;
    }
    else if (aim_field_angle == M_PI / 2)
    {
        hope_x = 0;
        hope_y = ((this_is_right) ? (parameters->pendulum_walk_param.max_step_y_in) : (parameters->pendulum_walk_param.max_step_y_out));
    }
    else if (aim_field_angle == -M_PI / 2)
    {
        hope_x = 0;
        hope_y = ((this_is_right) ? (-parameters->pendulum_walk_param.max_step_y_out) : (-parameters->pendulum_walk_param.max_step_y_in));
    }
    else if (aim_field_angle == M_PI || aim_field_angle == -M_PI)
    {
        hope_x = -parameters->pendulum_walk_param.max_step_x;
        hope_y = 0;
    }
    else
    {
        y_cut = ((this_is_right) ? ((sign(aim_field_angle) > 0) ? (parameters->pendulum_walk_param.max_step_y_in) : (-parameters->pendulum_walk_param.max_step_y_out)) : ((sign(aim_field_angle) > 0) ? (parameters->pendulum_walk_param.max_step_y_out) : (-parameters->pendulum_walk_param.max_step_y_in)));
        x_cut = sign(cos(aim_field_angle)) * parameters->pendulum_walk_param.max_step_x;
        hope_x = y_cut / (tan(aim_field_angle) + y_cut / x_cut);
        hope_y = tan(aim_field_angle) * hope_x;
    }

    //计算hope速度与上一步的速度差
    double error_x = hope_x - last_gait_element.x;
    double error_y = hope_y - last_gait_element.y;
    double error_direction = Atan(error_y, error_x);
    //计算速度改变引导下的未校验大小的速度
    double changed_x, changed_y;
    if (error_direction == 0)
    {
        changed_x = parameters->pendulum_walk_param.max_step_differ_x;
        changed_y = 0;
    }
    else if (error_direction == M_PI / 2 || error_direction == -M_PI / 2)
    {
        changed_x = 0;
        changed_y = parameters->pendulum_walk_param.max_step_differ_y * sign(error_direction);
    }
    else if (error_direction == M_PI || error_direction == -M_PI)
    {
        changed_x = -parameters->pendulum_walk_param.max_step_differ_x;
        changed_y = 0;
    }
    else
    {
        y_cut = sign(error_direction) * parameters->pendulum_walk_param.max_step_differ_y;
        x_cut = sign(cos(error_direction)) * parameters->pendulum_walk_param.max_step_differ_x;
        changed_x = y_cut / (tan(error_direction) + y_cut / x_cut);
        changed_y = tan(error_direction) * changed_x;
    }

    double pre_speed_x = last_gait_element.x + changed_x;
    double pre_speed_y = last_gait_element.y + changed_y;
    //判定这个速度是否超出了最大速度限制四边形
    bool is_overflow;
    double pre_speed_angle = Atan(pre_speed_y, pre_speed_x);
    double pre_dir_max_x, pre_dir_max_y;
    if (pre_speed_angle == 0)
    {
        x_cut = parameters->pendulum_walk_param.max_step_x;
        y_cut = parameters->pendulum_walk_param.max_step_y_in;
        pre_dir_max_x = parameters->pendulum_walk_param.max_step_x;
        pre_dir_max_y = 0;
    }
    else if (pre_speed_angle == M_PI / 2)
    {
        pre_dir_max_x = 0;
        pre_dir_max_y = ((this_is_right) ? (parameters->pendulum_walk_param.max_step_y_in) : (parameters->pendulum_walk_param.max_step_y_out));
        x_cut = parameters->pendulum_walk_param.max_step_x;
        y_cut = pre_dir_max_y;
    }
    else if (pre_speed_angle == -M_PI / 2)
    {
        pre_dir_max_x = 0;
        pre_dir_max_y = ((this_is_right) ? (-parameters->pendulum_walk_param.max_step_y_out) : (-parameters->pendulum_walk_param.max_step_y_in));
        x_cut = parameters->pendulum_walk_param.max_step_x;
        y_cut = pre_dir_max_y;
    }
    else if (pre_speed_angle == M_PI || pre_speed_angle == -M_PI)
    {
        pre_dir_max_x = -parameters->pendulum_walk_param.max_step_x;
        pre_dir_max_y = 0;
        x_cut = -parameters->pendulum_walk_param.max_step_x;
        y_cut = parameters->pendulum_walk_param.max_step_y_in;
    }
    else
    {
        y_cut = ((this_is_right) ? ((sign(pre_speed_angle) > 0) ? (parameters->pendulum_walk_param.max_step_y_in) : (-parameters->pendulum_walk_param.max_step_y_out)) : ((sign(pre_speed_angle) > 0) ? (parameters->pendulum_walk_param.max_step_y_out) : (-parameters->pendulum_walk_param.max_step_y_in)));
        x_cut = sign(cos(pre_speed_angle)) * parameters->pendulum_walk_param.max_step_x;
        pre_dir_max_x = y_cut / (tan(pre_speed_angle) + y_cut / x_cut);
        pre_dir_max_y = tan(pre_speed_angle) * hope_x;
    }
    is_overflow = ((pow(pre_dir_max_x, 2) + pow(pre_dir_max_y, 2)) < (pow(pre_speed_x, 2) + pow(pre_speed_y, 2)));
    double final_speed_x, final_speed_y;
    if (!is_overflow)
    {
        result_gait.x = final_speed_x = pre_speed_x;
        result_gait.y = final_speed_y = pre_speed_y;
        result_gait.isRight = this_is_right;
    }
    else
    {
        if (error_direction == 0 || error_direction == M_PI || error_direction == -M_PI)
        {
            final_speed_y = pre_speed_y;
            final_speed_x = x_cut - x_cut / y_cut * final_speed_y;
        }
        else if (error_direction == M_PI / 2 || error_direction == -M_PI / 2)
        {
            final_speed_x = pre_speed_x;
            final_speed_y = y_cut - y_cut / x_cut * final_speed_x;
        }
        else
        {
            final_speed_x = (y_cut - pre_speed_y + tan(error_direction) * pre_speed_x) / (tan(error_direction) + y_cut / x_cut);
            final_speed_y = y_cut - y_cut / x_cut * final_speed_x;
        }
        result_gait.x = final_speed_x;
        result_gait.y = final_speed_y;
        result_gait.isRight = this_is_right;
    }
    double error_angle;
    double final_yaw;

    if (error_aim_angle == 1000) //如果是默认的角度，机器人的旋转方向目标为速度方向
    {
        error_angle = Rad2Deg(Atan(final_speed_y, final_speed_x));
        result_gait.label = "speed_controller default error_angle";
    }
    else //如果指定了error_aim_angle，机器人的旋转目标为目标位置朝向的目标
    {
        error_angle = AdjustDegRange2(error_aim_angle);
        result_gait.label = "speed_controller customized error_angle";
    }
    //cout<<"error_angle"<<error_angle<<endl;
    //cout<<"error_aim_angle"<<Rad2Deg(abs(aim_field_angle))<<endl;
    //似乎是过大就只转10度
    if (abs(error_angle) > 20||Rad2Deg(abs(aim_field_angle))>50)  // TODO 修复这里的hard code
    {
        //cout<<"error_aim_angle"<<Rad2Deg(abs(aim_field_angle))<<endl;
        //这段代码可能是在判断是否是噪声干扰，第二个参数是判断上一次gait有没有做角度的变化
        if (sign(error_angle) != sign(last_gait_element.t) && last_gait_element.t != 0)
        {
            final_yaw = 0;
        }
        else
        {
            final_yaw = sign(error_angle) * 10;
        }
    }
    else
    {
        double nobrain_angle = parameters->stp.correct_k * aim_field_angle;
        nobrain_angle = (abs(nobrain_angle) > 10) ? (sign(nobrain_angle) * 10) : nobrain_angle;
        double this_step_x = parameters->stp.last_gait.x + parameters->pendulum_walk_param.max_step_differ_x / 3;
        if (this_step_x > parameters->pendulum_walk_param.max_step_x)
        {
            this_step_x = parameters->pendulum_walk_param.max_step_x;
        }
        result_gait.x = this_step_x;
        result_gait.y = nobrain_angle / parameters->stp.correct_y_k;
        final_yaw = nobrain_angle;
        result_gait.label = "nobrain";
        
    }
    //限制范围
    if (abs(final_yaw) > parameters->pendulum_walk_param.max_step_yaw)
    {
        final_yaw = sign(final_yaw) * parameters->pendulum_walk_param.max_step_yaw;
    }
    result_gait.t = final_yaw;
    return result_gait;
}

/////////////////////////////////
gait_element SimplePlanner::SimplePlannerPrivate::cal_the_next_gait( gait_element last_gait ,gait_element target_gait)
{
    gait_element gait_send;

    double last_theta = atan2(last_gait.y, last_gait.x);
    double target_theta = atan2(target_gait.y, target_gait.x);

    int last_gait_state=(last_gait.x==0 && last_gait.y==0)?1:2;//1 turn 2 forward
    int target_gait_state=(target_gait.x==0&&last_gait.y==0)?1:2;

    if(abs(last_gait.x)<2 && abs(last_gait.y)<0.3&&abs(last_gait.t)<3){
        last_gait_state=0;
    }
    if(abs(target_gait.x)<2 && abs(target_gait.y)<0.3&&abs(target_gait.t)<3){
        target_gait_state=0;//static
    }
    //cout<<"last_gait_state"<<last_gait_state<<endl;
    //cout<<"target_gait_state"<<target_gait_state<<endl;




    gait_send.isRight=target_gait.isRight;
    gait_send.label=target_gait.label;

    if((last_gait_state==2&&target_gait_state==2&&abs(last_theta - target_theta) >M_PI/6)||
    (last_gait_state==1&&target_gait_state==2)||(last_gait_state==2&&target_gait_state==1)){
        //cout<<"halt                       ----------halt-----"<<endl;

            gait_send.x = ((abs(last_gait.x) > parameters->pendulum_walk_param.slow_down_minus_x) ? last_gait.x -
                                                                                                   sign(last_gait.x) *
                                                                                                   parameters->pendulum_walk_param.slow_down_minus_y
                                                                                                 : 0);
            gait_send.y = ((abs(last_gait.y) > parameters->pendulum_walk_param.slow_down_minus_y) ? last_gait.y -
                                                                                                   sign(last_gait.y) *
                                                                                                   parameters->pendulum_walk_param.slow_down_minus_y
                                                                                                 : 0);
            gait_send.t = ((abs(last_gait.t) > parameters->pendulum_walk_param.slow_down_minus_yaw) ? last_gait.t -
                                                                                                     sign(last_gait.t) *
                                                                                                     parameters->pendulum_walk_param.slow_down_minus_yaw
                                                                                                   : 0);
        }

    else{
        //cout<<"adjust     -----adjust-------"<<endl;
        gait_send.x = ((abs(target_gait.x - last_gait.x) > parameters->pendulum_walk_param.slow_down_minus_x)
                       ?
                       last_gait.x +
                       sign(target_gait.x - last_gait.x) * parameters->pendulum_walk_param.slow_down_minus_x
                       : target_gait.x);
        gait_send.y = ((abs(target_gait.y - last_gait.y) > parameters->pendulum_walk_param.slow_down_minus_y)
                       ?
                       last_gait.y +
                       sign(target_gait.y - last_gait.y) * parameters->pendulum_walk_param.slow_down_minus_y
                       : target_gait.y);
        gait_send.t=((abs(target_gait.t - last_gait.t) > parameters->pendulum_walk_param.slow_down_minus_yaw)
                     ?
                     last_gait.t +
                     sign(target_gait.t - last_gait.t) * parameters->pendulum_walk_param.slow_down_minus_yaw
                     : target_gait.t);

    }
    return gait_send;

}

void SimplePlanner::SimplePlannerPrivate::SlowDown()//匀速减少
{
    parameters->stp.gait_queue.clear();
    double slow_init_x = parameters->stp.last_gait.x;
    double slow_init_y = parameters->stp.last_gait.y;
    double slow_init_yaw = parameters->stp.last_gait.t;
    //取次数a = max([x/dx],[y/dy],[t/dt]) + 1
    int a = abs(slow_init_x) / parameters->pendulum_walk_param.slow_down_minus_x;
    a = (abs(slow_init_y) / parameters->pendulum_walk_param.slow_down_minus_y > a) ? (abs(slow_init_y) / parameters->pendulum_walk_param.slow_down_minus_y) : (a);
    a = (abs(slow_init_yaw) / parameters->pendulum_walk_param.slow_down_minus_yaw > a) ? (abs(slow_init_yaw) / parameters->pendulum_walk_param.slow_down_minus_yaw) : (a);
    a = a + 1;

    for (int i = 0; i < a; i++)
    {
        parameters->stp.tmp_gait.x = slow_init_x * (a - i - 1) / a;
        parameters->stp.tmp_gait.y = slow_init_y * (a - i - 1) / a;
        parameters->stp.tmp_gait.t = slow_init_yaw * (a - i - 1) / a;
        parameters->stp.tmp_gait.isRight = !parameters->stp.tmp_gait.isRight;
        parameters->stp.tmp_gait.label = "SlowDown";
        parameters->stp.gait_queue.push_back(parameters->stp.tmp_gait);
    }
}

void SimplePlanner::SimplePlannerPrivate::LittleAdjust(double start_x, double start_y, double start_angle, double end_x, double end_y, double end_angle)//给定坐标系不一定是机器人坐标系
{
    std::vector<double> angle_sequence,x_sequence,y_sequence,aim_field;
    int adjust_step_num;
    double error_angle = AdjustDegRange2(end_angle - start_angle);
    start_angle = Deg2Rad(start_angle), end_angle = Deg2Rad(end_angle);
    //先转换为机器人坐标系
    aim_field.push_back((end_x  - start_x)* cos(start_angle) + (end_y - start_y)  * sin(start_angle) );
    aim_field.push_back( (end_y - start_y) * cos(start_angle) - (end_x - start_x) * sin(start_angle));
    aim_field.push_back(error_angle);
    //以下建议写成max形式
    adjust_step_num = (int)(abs(aim_field[0]) / parameters->stp.adjust_max_x + 1);
    adjust_step_num = ((int)(abs(aim_field[1]) / parameters->stp.adjust_max_y + 1) > adjust_step_num) ? ((int)(abs(aim_field[1]) / parameters->stp.adjust_max_y + 1)) : (adjust_step_num);
    adjust_step_num = ((int)(abs(aim_field[2]) / parameters->stp.adjust_max_yaw + 1) > adjust_step_num) ? ((int)(abs(aim_field[2]) / parameters->stp.adjust_max_yaw + 1)) : (adjust_step_num);
    
    angle_sequence.push_back(0);
    angle_sequence.push_back(aim_field[2] / 2.0 / adjust_step_num);
    x_sequence.push_back(0);
    x_sequence.push_back(aim_field[0] / 2.0 / adjust_step_num);
    y_sequence.push_back(0);
    y_sequence.push_back(aim_field[1] / 2.0 / adjust_step_num);
    for (int i = 0; i < adjust_step_num - 1; i++)
    {
        angle_sequence.push_back(aim_field[2] / adjust_step_num + angle_sequence[i + 1]);
        x_sequence.push_back(aim_field[0] / adjust_step_num + x_sequence[i + 1]);
        y_sequence.push_back(aim_field[1] / adjust_step_num + y_sequence[i + 1]);
    }
    angle_sequence.push_back(aim_field[2] / 2.0 / adjust_step_num + angle_sequence[adjust_step_num]);
    x_sequence.push_back(aim_field[0] / 2.0 / adjust_step_num + x_sequence[adjust_step_num]);
    y_sequence.push_back(aim_field[1] / 2.0 / adjust_step_num + y_sequence[adjust_step_num]);

    for (unsigned i = 0; i < x_sequence.size() - 1; i++)
    {
        if (i < parameters->stp.adjust_max_step_num)
        {
            std::vector<double> adjust_step = OneStepTransform(x_sequence[i], y_sequence[i], angle_sequence[i],
                                                          x_sequence[i + 1], y_sequence[i + 1], angle_sequence[i + 1]);
            parameters->stp.tmp_gait.x = adjust_step[0];
            parameters->stp.tmp_gait.y = adjust_step[1];
            parameters->stp.tmp_gait.t = adjust_step[2];
            parameters->stp.tmp_gait.isRight = !parameters->stp.tmp_gait.isRight;
            parameters->stp.tmp_gait.label = "LittleAdjust";
            parameters->stp.gait_queue.push_back(parameters->stp.tmp_gait);
        }
        else
        {
            break;
        }
    }
    parameters->stp.tmp_gait.x = 0;
    parameters->stp.tmp_gait.y = 0;
    parameters->stp.tmp_gait.t = 0;
    parameters->stp.tmp_gait.isRight = !parameters->stp.tmp_gait.isRight;
    parameters->stp.tmp_gait.label = "LittleAdjust";
    parameters->stp.gait_queue.push_back(parameters->stp.tmp_gait);
}

/** 
 * 用于任意坐标系下两步相对于机器人GiveAStep参数的生成，这里使用了比较输入为世界坐标系下的<x,y,yaw>起点与终点
 * 输出为相对于此时机器人的位置下一步参数,可以用于GiveAStep
 * 
 * */

std::vector<double> SimplePlanner::SimplePlannerPrivate::OneStepTransform(double global_start_x, double global_start_y, double global_start_yaw, double global_end_x, double global_end_y, double global_end_yaw)
{
    double start_x = global_start_x;
    double start_y = global_start_y;
    double end_x = global_end_x; //限制x范围
    double end_y = global_end_y; //限制y范围
    double start_angle = AdjustDegRange2(global_start_yaw);
    double end_angle = (abs(AdjustDegRange2(global_end_yaw - start_angle)) > 25) ? ((sign(AdjustDegRange2(global_end_yaw - start_angle))) * 25 + start_angle) : AdjustDegRange2(global_end_yaw);
    double result_angle = AdjustDegRange2(end_angle - start_angle);

    start_angle = Deg2Rad(start_angle);
    end_angle = Deg2Rad(end_angle);

    //转换成机器人坐标系
    double result_x = end_x * cos(start_angle) - start_x * cos(start_angle) + end_y * sin(start_angle) -
                      start_y * sin(start_angle);
    double result_y = end_y * cos(start_angle) - start_y * cos(start_angle) - end_x * sin(start_angle) +
                      start_x * sin(start_angle);



    result_x = (abs(result_x) > 8) ? (sign(result_x) * 8) : result_x;
    result_y = (abs(result_y) > 4) ? (sign(result_y) * 4) : result_y;
    std::vector<double> result_array = {result_x, result_y, result_angle};
    return result_array;
}

}  