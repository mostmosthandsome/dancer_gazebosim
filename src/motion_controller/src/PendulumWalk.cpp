//
// Created by zjudancer on 18-12-18.
//

#include "PendulumWalk.h"
using namespace dmotion;
namespace dmotion
{

PendulumWalk::PendulumWalk(std::shared_ptr< std::queue< std::vector<double> > > _action_list,std::shared_ptr<Parameters> param) : parameters(param),action_list(_action_list)
{
    com_ac_x = 0;
    com_ac_y = 0;
    support_is_right = false;
    hang_foot = {0, parameters->pendulum_walk_param.ANKLE_DIS, 0};
    com_pos = {0, parameters->pendulum_walk_param.ANKLE_DIS / 2.0, 0};
    com_x_changed = 0;
    com_y_changed = 0;
    Support = new OneFootLanding(false,parameters);
    last_rpy.resize(3);
}

PendulumWalk::PendulumWalk(std::shared_ptr< std::queue< std::vector<double> > > _action_list, std::shared_ptr<Parameters> param, const double &unusual_ankle_dis) : parameters(param),action_list(_action_list)
{
    com_ac_x = 0;
    com_ac_y = 0;
    support_is_right = false;
    hang_foot = {0, unusual_ankle_dis, 0};
    com_pos = {0, unusual_ankle_dis / 2.0, 0};
    com_x_changed = 0;
    com_y_changed = 0;
    Support = new OneFootLanding(false,parameters);
    last_rpy.resize(3);
}
PendulumWalk::~PendulumWalk()
{
    delete Support;
    Support = nullptr;
}

/**
     * 生成下一步的动作数据
     * @param dx 下一步质心的x变化，相对于上半身，单位是cm
     * @param dy 下一步执行的y变化，相对于上半身，单位是cm
     * @param d_yaw 下一步执行的yaw变化，相对于上半身，角度制
     */
void PendulumWalk::GiveAStep(double dx, double dy, double d_yaw)
{
    // cout << "x : " << dx << " y : " << dy << " d_yaw : " << d_yaw << " " << endl;
    cout << endl;
    delete Support;
    Support = new OneFootLanding(support_is_right,parameters);
    
    //如果是质心初始位置的话，那需要com_pos表示上一时刻质心轨迹的终点，hang_foot表示新悬荡脚作为支撑脚时偏移一半ankle——dis的位置（在哪个坐标系中随意）
    x0 = com_pos[0] * cos(hang_foot[2]) - hang_foot[0] * cos(hang_foot[2]) + com_pos[1] * sin(hang_foot[2]) - hang_foot[1] * sin(hang_foot[2]);
    //计算下一步落脚点的x坐标（在机器人质心局部坐标系下）
    xt = (dx - ((support_is_right) ? (parameters->pendulum_walk_param.ANKLE_DIS / 2.0) : (-parameters->pendulum_walk_param.ANKLE_DIS / 2.0)) * sin(Deg2Rad(d_yaw))) / 2.0;
    //        std::cout << "x0 :" << x0 << std::endl;
    //        std::cout << "xt :" << xt << std::endl;

    tao = parameters->pendulum_walk_param.TAO;

    //算出摆的周期常数，这里的com_h暂时是由机器人crouch姿态下倒挂着摆动测量得出的
    com_h = parameters->pendulum_walk_param.COM_H;
    Tc = std::sqrt(com_h / 980);
    //        std::cout << "Tc  :" << Tc << " " << std::endl;

    //算出来这个步态单元的初速度vx  
    double C = cosh(tao / Tc),S = sinh(tao / Tc);
    vx = (xt - x0 * C) / (Tc * S);
    //        std::cout << "vx :" << vx << std::endl;

    //y方向的研究

    y00 = com_pos[1] * cos(hang_foot[2]) - hang_foot[1] * cos(hang_foot[2]) - com_pos[0] * sin(hang_foot[2]) +
          hang_foot[0] * sin(hang_foot[2]);
    ytt = (dy + ((support_is_right) ? (parameters->pendulum_walk_param.ANKLE_DIS / 2.0) : (-parameters->pendulum_walk_param.ANKLE_DIS / 2.0)) +
           ((support_is_right) ? (parameters->pendulum_walk_param.ANKLE_DIS / 2.0) : (-parameters->pendulum_walk_param.ANKLE_DIS / 2.0)) * cos(Deg2Rad(d_yaw))) /
          2;

    //        std::cout << "y00 :" << y00 << std::endl;
    //        std::cout << "ytt :" << ytt << std::endl;

    //这里实际计算质心轨迹的是y0和yt，防止∆y太大。
    y0 = support_is_right ? (parameters->pendulum_walk_param.Y_HALF_AMPLITUDE) : (-parameters->pendulum_walk_param.Y_HALF_AMPLITUDE);
    yt = support_is_right ? (parameters->pendulum_walk_param.Y_HALF_AMPLITUDE) : (-parameters->pendulum_walk_param.Y_HALF_AMPLITUDE);

    //计算过程中换元得到的一个临时变量m
    m = exp(tao / Tc);
    //        std::cout << "m  :" << m << " " << std::endl;

    //步行单元的周期定了后y方向的最大速度是确定的
    vy = (yt - m * y0) / ((m + 1) * Tc);
    //        std::cout << "vy  :" << vy << " " << std::endl;

    //使用插值算法定义抬脚的时间位移曲线
    std::vector<double> akZ_t = parameters->pendulum_walk_param.foot_z_t;
    std::vector<double> akZ_p = parameters->pendulum_walk_param.foot_z_p;
    std::vector<double> akZ_s = parameters->pendulum_walk_param.foot_z_s;
    ThreeInterpolation ankle_z(parameters,akZ_t, akZ_p, akZ_s);
    // cout << "使用插值算法定义抬脚的时间位移曲线 "<< endl;
    // ankle_z.CalculatePoints(10);
    akZ = ankle_z.GetPoints();

    //TODO consider if we are going to plan the ankle pitch

    //线性规划上半身yaw
    std::vector<double> comYaw_t = {0, parameters->pendulum_walk_param.TAO};
    std::vector<double> comYaw_p = {Rad2Deg(com_pos[2] - hang_foot[2]), d_yaw / 2.0};
    double slope_comYaw = (d_yaw / 2.0 - Rad2Deg(com_pos[2] - hang_foot[2])) / parameters->pendulum_walk_param.TAO;
    std::vector<double> comYaw_s = {slope_comYaw, slope_comYaw};
    ThreeInterpolation com_yaw(parameters,comYaw_t, comYaw_p, comYaw_s);
    // cout << "线性规划上半身yaw "<< endl;
    // com_yaw.CalculatePoints(10);
    comYaw = com_yaw.GetPoints();
    //        PrintVector(comYaw);

    //线性规划质心的y基础位置
    std::vector<double> comY_t = {0, parameters->pendulum_walk_param.TAO};
    std::vector<double> comY_p = {y00, ytt};
    double slope_comY = (ytt - y00) / parameters->pendulum_walk_param.TAO;
    std::vector<double> comY_s = {slope_comY, slope_comY};
    ThreeInterpolation com_y(parameters,comY_t, comY_p, comY_s);
    // cout << "线性规划质心的y基础位置 "<< endl;
    // com_y.CalculatePoints(10);
    comY = com_y.GetPoints();
    //        std::cout << "comY :" << std::endl;
    //        PrintVector(comY);

    //线性的质心加速度x偏移
    double ac_x = (dx - com_x_changed) * parameters->pendulum_walk_param.ACC_COEF_X;
    std::vector<double> accX_t = {0, parameters->pendulum_walk_param.TAO};
    std::vector<double> accX_p = {com_ac_x, ac_x};
    double slope_accX = (ac_x - com_ac_x) / parameters->pendulum_walk_param.TAO;
    std::vector<double> accX_s = {slope_accX, slope_accX};
    ThreeInterpolation acc_x(parameters,accX_t, accX_p, accX_s);
    // cout << "线性的质心加速度x偏移 "<< endl;
    accX = acc_x.GetPoints();
    //        std::cout << "accX :" << std::endl;
    //        PrintVector(accX);

    //线性的质心加速度y偏移

    //dy的变化成上某个系数
    double ac_y = (dy - com_y_changed) * parameters->pendulum_walk_param.ACC_COEF_Y;
    std::vector<double> accY_t = {0, parameters->pendulum_walk_param.TAO};
    std::vector<double> accY_p = {com_ac_y, ac_y};
    double slope_accY = (ac_y - com_ac_y) / parameters->pendulum_walk_param.TAO;
    std::vector<double> accY_s = {slope_accY, slope_accY};
    ThreeInterpolation acc_y(parameters,accY_t, accY_p, accY_s);
    // cout << "线性的质心加速度y偏移 "<< endl;
    //        cout << "com_ac_y :" <<com_ac_y << endl;
    //        cout << "ac_y :" <<ac_y << endl;
    accY = acc_y.GetPoints();
    //        std::cout << "accY :" << std::endl;
    //        PrintVector(accY);

    //脚踝的x方向起点终点的插值
    //如果表示悬荡脚初始位置，hangfoot应当表示现在的质心在原来以悬荡脚中心的坐标系中的位置
    double ak_x_0 = -hang_foot[0] * cos(hang_foot[2]) - hang_foot[1] * sin(hang_foot[2]);
    double ak_x_t = 2 * xt;
    std::vector<double> akX_t = {0, parameters->pendulum_walk_param.TAO};
    std::vector<double> akX_p = {ak_x_0, ak_x_t};
    //        cout << "x0 :" << x0 << endl;
    //        cout << "xt :" << xt << endl;
    //        cout << "ak_x_0 :" << ak_x_0 << endl;
    //        cout << "ak_x_t :" << ak_x_t << endl;
    double slope_akX = (ak_x_t - ak_x_0) / parameters->pendulum_walk_param.TAO;
    std::vector<double> akX_s = {slope_akX, slope_akX};
    ThreeInterpolation ak_x(parameters,akX_t, akX_p, akX_s);
    // cout << "脚踝的x方向起点终点的插值 "<< endl;
    akX = ak_x.GetPoints();
    //        std::cout << "akX :" << std::endl;
    //        PrintVector(akX);

    //脚踝的y方向起点终点的插值
    double ak_y_0 = hang_foot[0] * sin(hang_foot[2]) - hang_foot[1] * cos(hang_foot[2]);
    double ak_y_t = 2 * ytt;
    std::vector<double> akY_t = {0, parameters->pendulum_walk_param.TAO};
    std::vector<double> akY_p = {ak_y_0, ak_y_t};
    double slope_akY = (ak_y_t - ak_y_0) / parameters->pendulum_walk_param.TAO;
    std::vector<double> akY_s = {slope_akY, slope_akY};
    ThreeInterpolation ak_y(parameters,akY_t, akY_p, akY_s);
    // cout << "脚踝的y方向起点终点的插值 "<< endl;
    akY = ak_y.GetPoints();
    //        std::cout << "akY :" << std::endl;
    //        PrintVector(akY);

    //脚踝的yaw起点终点的插值
    std::vector<double> akYaw_t = {0, parameters->pendulum_walk_param.TAO};
    std::vector<double> akYaw_p = {Rad2Deg(-hang_foot[2]), d_yaw};
    double slope_akYaw = (akYaw_p[1] - akYaw_p[0]) / parameters->pendulum_walk_param.TAO / 2;
    std::vector<double> akYaw_s = {slope_akYaw, slope_akYaw};
    ThreeInterpolation ak_yaw(parameters,akYaw_t, akYaw_p, akYaw_s);
    // cout << "脚踝的yaw起点终点的插值 "<< endl;
    akYaw = ak_yaw.GetPoints();

    //为下一步的数据做准备
    support_is_right = !support_is_right;
    com_x_changed = dx;
    com_y_changed = dy;

    com_pos.clear();
    com_pos.push_back(xt);
    com_pos.push_back(ytt);
    com_pos.push_back(Deg2Rad(d_yaw) / 2.0);
    //        cout << "com_pos :" << endl;
    //        PrintVector(com_pos);
    hang_foot.clear();
    hang_foot.push_back(ak_x_t);
    hang_foot.push_back(ak_y_t);
    hang_foot.push_back(2.0 * com_pos[2]); //这里使得落脚点的yaw和质心一致了
                                           //        cout << "hang_foot :" << endl;
                                           //        PrintVector(hang_foot);

    com_ac_x = ac_x;
    com_ac_y = ac_y;

    tick_num = 0;
}

void PendulumWalk::GiveATick()
{

    motion_tick tmptick;
    tmptick.time_stamp = 10000000 * (tick_num); //10毫秒
    x = x0 * std::cosh(0.01 * (tick_num) / Tc) + Tc * vx * std::sinh(0.01 * (tick_num) / Tc);
    y = y0 * std::cosh(0.01 * (tick_num) / Tc) + Tc * vy * std::sinh(0.01 * (tick_num) / Tc);
    tmptick.upbody_pose.emplace_back(0);
    tmptick.upbody_pose.emplace_back(0);
    tmptick.upbody_pose.emplace_back(comYaw[tick_num]);
    tmptick.whole_com.emplace_back(accX[tick_num] + x + parameters->pendulum_walk_param.COM_X_OFFSET); //(x * 100 +1.5);
    tmptick.whole_com.emplace_back(y + accY[tick_num]);
    tmptick.whole_com.emplace_back(parameters->pendulum_walk_param.COM_HEIGHT); //0.308637

    tmptick.hang_foot.emplace_back(akX[tick_num]);
    tmptick.hang_foot.emplace_back(akY[tick_num]);
    tmptick.hang_foot.emplace_back(akZ[tick_num]); 
    tmptick.hang_foot.emplace_back(0);
    tmptick.hang_foot.emplace_back(0);
    tmptick.hang_foot.emplace_back(akYaw[tick_num]);

    //        if(tick_num == 0)
    //        {
    //            cout << "comY[tick_num] :" << comY[tick_num] <<endl;
    //            dmotion::PrintVector(tmptick.whole_com);
    //            dmotion::PrintVector(tmptick.hang_foot);
    //        }
    tick_num++;

    parameters->support_phase = (SupportPhase)((int)support_is_right);
    auto action = Support->GetOneStep(tmptick.hang_foot, tmptick.whole_com, tmptick.upbody_pose);
    action_list->push(action);
}

void PendulumWalk::GiveAStepTick(gait_element now_gait)
{
    GiveAStep(now_gait.x, now_gait.y, now_gait.t);
    for (int j = 0; j < parameters->pendulum_walk_param.TICK_NUM; j++)     parameters->stp.cur_state.index = j,GiveATick(); // after this, cur_servo_angles update
}

bool PendulumWalk::CheckWillFall()
{
    bool will_fall = false;
    double dRoll, dPitch;
    dRoll =  parameters->stp.cur_state.rpy[0] - last_rpy[0];
    dPitch = parameters->stp.cur_state.rpy[1] - last_rpy[1];
    // ROS_INFO_STREAM("dRoll: "<<dRoll<<"  dPitch: "<<dPitch<<"  fall_tick: "<<fall_pre_tick);
    if(std::abs(dRoll) > FALL_FOR_TOLERANCE || std::abs(dPitch) > FALL_SIDE_TOLERANCE)
    {
        fall_pre_tick++;
    }
    else
        fall_pre_tick = 0;
    
    if(fall_pre_tick > FALL_TICK_NUM)
    {   
        will_fall = true;
        fall_pre_tick = 0;
    }
    last_rpy[0] = parameters->stp.cur_state.rpy[0];
    last_rpy[1] = parameters->stp.cur_state.rpy[1];

    return will_fall;
}

bool PendulumWalk::AdjustFoothold(double dx, double dy, double d_yaw)
{
    if(!PendulumWalk::CheckWillFall() || parameters->stp.cur_state.index < 5) return false;
    else RCLCPP_WARN(rclcpp::get_logger("adjust foot hold"),"Adjust Foothold!");
    robot_state cur_state, predict_state;
    std::vector<double> stable_energy={-200,-180,0}; // TODO：Get from parameters
    std::vector<double> com_new={0,0,0}, foot_new={0,0,0};
    int cur_i = parameters->stp.cur_state.index;
    std::vector<double> comp(cur_i,0); // 把调整后的vector前面补0，使得其长度和原来一样

    parameters->stp.cur_state.foot[0] = akX[cur_i]; // 先采用开环的悬荡脚位置看看
    parameters->stp.cur_state.foot[1] = akY[cur_i];
    parameters->stp.cur_state.foot[2] = akZ[cur_i];
    cur_state = parameters->stp.cur_state;

    double left_time = (parameters->pendulum_walk_param.TICK_NUM-cur_i)*0.01; // 每个点的间隔是0.01s
    Tc = std::sqrt(com_h/980); //摆动常数

    predict_state.com[0] = cur_state.com[0] * std::cosh(left_time/Tc) 
    + Tc*cur_state.dcom[0]*std::sinh(left_time/Tc); // 用线性倒立摆模型计算预测落地位置。

    predict_state.dcom[0] = cur_state.com[0]*std::sinh(left_time/Tc)/Tc   // 用线性倒立摆模型计算预测落地速度。
    + cur_state.dcom[0] * std::cosh(left_time/Tc);

    predict_state.com[1] = cur_state.com[1] * std::cosh(left_time/Tc) 
    + Tc*cur_state.dcom[1]*std::sinh(left_time/Tc);

    predict_state.dcom[1] = cur_state.com[1]*std::sinh(left_time/Tc)/Tc
    + cur_state.dcom[1] * std::cosh(left_time/Tc);

    com_new[0] = - std::sqrt((predict_state.dcom[0]*predict_state.dcom[0] - 2*stable_energy[0])) * Tc;
    foot_new[0] = predict_state.com[0] - com_new[0];

    com_new[1] = ((support_is_right) ? (-1):1) * std::sqrt((predict_state.dcom[1]*predict_state.dcom[1] - 2*stable_energy[1])) * Tc;
    foot_new[1] = predict_state.com[1] + com_new[1];
    // TODO: 脚踝y方向调整

    // foot_new[0] = (std::abs(foot_new[0]) > parameters->pendulum_walk_param.max_step_x)? parameters->pendulum_walk_param.max_step_x:foot_new[0];
    
    if(std::abs(foot_new[0]) > parameters->pendulum_walk_param.max_step_x)
        foot_new[0] = parameters->pendulum_walk_param.max_step_x * std::abs(foot_new[0])/foot_new[0];

    if(std::abs(foot_new[1]) < std::abs(hang_foot[1]))
        foot_new[1] = hang_foot[1];
        
    if(std::abs(foot_new[1]) > parameters->pendulum_walk_param.max_step_y_in)
        foot_new[1] = parameters->pendulum_walk_param.max_step_y_in * std::abs(foot_new[1])/foot_new[1];
    
    // RCLCPP_WARN(rclcpp::get_logger("adjust foot hold"),"Max Step x: %lf ,Foot New x: ",parameters->pendulum_walk_param.max_step_x, << ""<<<<"  Foot New y: "
    // <<foot_new[1] << "support is right?  "<< support_is_right << "predict comY: "<<predict_state.com[1] << "comY new: "<<com_new[1]);

// 摆动腿和身体质心插值

    //使用插值算法定义抬脚的高度时间位移曲线
    std::vector<double> my_comY, my_comYaw;
    std::vector<double> my_akX, my_akY, my_akYaw, my_akZ;

    //线性规划质心的y基础位置
    std::vector<double> comY_t = {0, left_time};
    std::vector<double> comY_p = {cur_state.com[1], com_new[1]};
    double slope_comY = (com_new[1] - cur_state.com[1]) / left_time;
    std::vector<double> comY_s = {slope_comY, slope_comY};
    ThreeInterpolation com_y(parameters,comY_t, comY_p, comY_s);
    // cout << "线性规划质心的y基础位置 "<< endl;
    // com_y.CalculatePoints(10);
    my_comY = com_y.GetPoints();
    my_comY.insert(my_comY.begin(), comp.data(), comp.data() + cur_i);
    
    // std::cout << "-------mycomY------size: " << my_comY.size() <<std::endl;
    // PrintVector(my_comY);
    // std::cout << "-------comY--------size: " << comY.size() <<std::endl;
    // PrintVector(comY);


    //脚踝的x方向起点终点的插值
    std::vector<double> akX_t = {0, left_time};
    std::vector<double> akX_p = {cur_state.foot[0], foot_new[0]};
    double slope_akX = (foot_new[0]-cur_state.foot[0])/left_time;
    std::vector<double> akX_s = {slope_akX, slope_akX};  //可以测试末端斜率设成0的效果
    ThreeInterpolation ak_x(parameters,akX_t, akX_p, akX_s);
    // cout << "脚踝的x方向起点终点的插值 "<< endl;
    my_akX = ak_x.GetPoints();
    my_akX.insert(my_akX.begin(), comp.data(), comp.data() + cur_i);
    std::cout << "-------myakX------size: " << my_akX.size() <<std::endl;
    PrintVector(my_akX);
    std::cout << "-------akX------size: " << akX.size() <<std::endl;
    PrintVector(akX);
    akX = my_akX; //实机测试


    //脚踝的y方向起点终点的插值
    std::vector<double> akY_t = {0, left_time};
    std::vector<double> akY_p = {cur_state.foot[1], foot_new[1]};
    double slope_akY = (foot_new[1]-cur_state.foot[1])/left_time;
    std::vector<double> akY_s = {slope_akY, slope_akY};  //可以测试末端斜率设成0的效果
    ThreeInterpolation ak_y(parameters,akY_t, akY_p, akY_s);
    // cout << "脚踝的y方向起点终点的插值 "<< endl;
    my_akY = ak_y.GetPoints();
    my_akY.insert(my_akY.begin(), comp.data(), comp.data() + cur_i);
    std::cout << "-------myakY-------size: " <<my_akY.size()<< std::endl;
    PrintVector(my_akY);
    std::cout << "-------akY-------size: " <<akY.size()<< std::endl;
    PrintVector(akY);
    akY = my_akY;

    // Z，脚踝Yaw, 以及身体Yaw 先别管

    #ifdef MODIFY_ACC_OFFSET
    //线性的质心加速度x偏移
    double ac_x = (dx - com_x_changed) * parameters->pendulum_walk_param.ACC_COEF_X;
    std::vector<double> accX_t = {0, parameters->pendulum_walk_param.TAO};
    std::vector<double> accX_p = {com_ac_x, ac_x};
    double slope_accX = (ac_x - com_ac_x) / parameters->pendulum_walk_param.TAO;
    std::vector<double> accX_s = {slope_accX, slope_accX};
    ThreeInterpolation acc_x(accX_t, accX_p, accX_s);
    // cout << "线性的质心加速度x偏移 "<< endl;
    accX = acc_x.GetPoints();
    //        std::cout << "accX :" << std::endl;
    //        PrintVector(accX);

    //线性的质心加速度y偏移
    double ac_y = (dy - com_y_changed) * parameters->pendulum_walk_param.ACC_COEF_Y;
    std::vector<double> accY_t = {0, parameters->pendulum_walk_param.TAO};
    std::vector<double> accY_p = {com_ac_y, ac_y};
    double slope_accY = (ac_y - com_ac_y) / parameters->pendulum_walk_param.TAO;
    std::vector<double> accY_s = {slope_accY, slope_accY};
    ThreeInterpolation acc_y(accY_t, accY_p, accY_s);
    // cout << "线性的质心加速度y偏移 "<< endl;
    //        cout << "com_ac_y :" <<com_ac_y << endl;
    //        cout << "ac_y :" <<ac_y << endl;
    accY = acc_y.GetPoints();
    //        std::cout << "accY :" << std::endl;
    //        PrintVector(accY);

    #endif

#ifdef UPDATE_CUR_PARA
    com_x_changed = com_new[0];
    com_y_changed = com_new[1];

    com_pos.clear();
    com_pos.push_back(com_new[0]);
    com_pos.push_back(com_new[1]);
    com_pos.push_back(Deg2Rad(d_yaw) / 2.0);
    hang_foot.clear();
    hang_foot.push_back(ak_x_t);
    hang_foot.push_back(ak_y_t);
    hang_foot.push_back(2.0 * com_pos[2]); //这里使得落脚点的yaw和质心一致了
                                           //        cout << "hang_foot :" << endl;
                                           //        PrintVector(hang_foot);
    // com_ac_x = ac_x;
    // com_ac_y = ac_y;
    tick_num = 0;
#endif

    return true; 

}




void PendulumWalk::TurnAround(double angle)
{
    vector<double> turn_sequence = CalculateTurnSequence(angle);

    parameters->stp.tmp_gait.x = 0;
    parameters->stp.tmp_gait.y = 0;
    parameters->stp.tmp_gait.t = turn_sequence[0];
    parameters->stp.tmp_gait.isRight = !parameters->stp.tmp_gait.isRight;
    parameters->stp.tmp_gait.label = "Turning";
    parameters->stp.gait_queue.push_back(parameters->stp.tmp_gait);

}

vector<double> PendulumWalk::CalculateTurnSequence(double turn_angle)
{
    if (turn_angle == 0)
    {
        return {0};
    }
    double angle_sign = dmotion::sign(turn_angle);
    turn_angle = turn_angle / angle_sign;
    //选取最省事的旋转方向
    if (turn_angle > 180)
    {
        turn_angle = 360 - turn_angle;
        angle_sign = -angle_sign;
    }

    vector<double> tmp_sequence;
    if (turn_angle / 2 <= 8)
    {
        tmp_sequence.push_back(turn_angle / 2);
    }
    else if (turn_angle / 2 > 8 && turn_angle / 2 <= 24)
    {
        tmp_sequence.push_back(turn_angle / 6);
        tmp_sequence.push_back(turn_angle * 2 / 6);
    }
    else if (turn_angle / 2 > 24 && turn_angle / 2 <= 48)
    {
        tmp_sequence.push_back(turn_angle / 12);
        tmp_sequence.push_back(turn_angle * 2 / 12);
        tmp_sequence.push_back(turn_angle * 3 / 12);
    }
    else if (turn_angle / 2 > 48 && turn_angle / 2 <= 72)
    {
        tmp_sequence.push_back(turn_angle / 18);
        tmp_sequence.push_back(turn_angle * 2 / 18);
        tmp_sequence.push_back(turn_angle * 3 / 18);
        tmp_sequence.push_back(turn_angle * 3 / 18);
    }
    else if (turn_angle / 2 > 72 && turn_angle / 2 <= 90)
    {
        tmp_sequence.push_back(turn_angle / 24);
        tmp_sequence.push_back(turn_angle * 2 / 24);
        tmp_sequence.push_back(turn_angle * 3 / 24);
        tmp_sequence.push_back(turn_angle * 3 / 24);
        tmp_sequence.push_back(turn_angle * 3 / 24);
    }

    unsigned long tmp_vector_size = tmp_sequence.size();
    vector<double> result_sequence;
    for (unsigned i = 0; i < tmp_vector_size; i++)
    {
        result_sequence.push_back(angle_sign * (tmp_sequence[i] + parameters->pendulum_walk_param.TURNING_ERROR));
    }
    for (unsigned i = 0; i < tmp_vector_size; i++)
    {
        result_sequence.push_back(angle_sign * (tmp_sequence[tmp_vector_size - i - 1] + parameters->pendulum_walk_param.TURNING_ERROR));
    }
    return result_sequence;
}


} // namespace dmotion


