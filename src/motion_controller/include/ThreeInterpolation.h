//
// Created by fw on 18/10/17.
// E-mail: zjufanwu@zju.edu.cn
//TODO:有时间实现实例化对象时输入参数斜率改为倾斜角的工作

#ifndef THREEINTERPOLATION_H
#define THREEINTERPOLATION_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <memory>
#include "Polynomial/Polynomial.hpp"
#include "Parameters.h"
#include <vector>

// #define DEFAULT_POINT_INTERVAL 10   //默认的发值时间间隔
// #define DEFAULT_BOUNDARY_SLOPE 1.0

namespace dmotion {

    class ThreeInterpolation {
    private:
        std::shared_ptr<Parameters> parameters;
        int piece_num_;
        double time_interval_;
        std::vector<dmotion::Polynomial<3>> poly_;
        std::vector<double> x_array_;
        std::vector<double> y_array_;
        std::vector<double> s_angle_;
        std::vector<double> x_samples_;
        std::vector<double> y_samples_;

    public:
        bool is_order = false;
        //参数健全的构造函数
        ThreeInterpolation(std::shared_ptr<Parameters> param,
                            const std::vector<double> &x_array,
                           const std::vector<double> &y_array,
                           const std::vector<double> &s_angle);

        //缺省了斜率的构造函数
        ThreeInterpolation(std::shared_ptr<Parameters> param,const std::vector<double> &x_array,
                           const std::vector<double> &y_array);


        //析构函数
        ~ThreeInterpolation();

        //获得曲线所对应的函数的定义域中的某一x点的函数值
        double EvalHere(double x0) const;

        //获得固定时间间隔下样本点的Eval序列，时间t0是这个间隔时间，单位是ms,有默认毫秒数DEFAULT_POINT_INTERVAL
        void CalculatePoints(int t0_in);
        void CalculatePoints();
        

        //获取t0时间间隔下的分段函数值点序列
        std::vector<double> &GetPoints();

        //获取和GetPoints相对应的时间序列。
        std::vector<double> &GetTimes();

        //添加一个点，并重新进行calculate这一步
        void AddPoint(double x_a, double y_a, double s_a = 0);

        //查看一个分段的三次多项式系数
        Eigen::Matrix<double, 4, 1> GetCoef(int piece_num) const;

    protected:
        //构造对象的时候要先判断输入的参数是否有效，x_array必须是顺序的
        bool isInOrder(std::vector<double> &x_ar);

        //经过这一步计算，算出这些分段多项式的Polynomial对象
        void Calculate();

        //单独一个piece的计算过程
        Eigen::Matrix<double, 4, 1> OnePiece(const double &x_r, const double &y_r, const double &s_r,
                                              const double &x_l, const double &y_l, const double &s_l);

    private:
    };

    inline std::vector<double> Planning(std::shared_ptr<Parameters> parameters, double start_time, double end_time, double y_0, double y_1, double s_0, double s_1)
    {
        std::vector<double> x, y, s, result;
        y.push_back(y_0);
        y.push_back(y_1);
        s.push_back(s_0);
        s.push_back(s_1);
        x.push_back(start_time / 100.0);
        x.push_back(end_time / 100.0);
        //        cout << "tttttt" << endl;
        //        PrintVector(x);
        //        PrintVector(y);
        //        PrintVector(s);

        ThreeInterpolation hello(parameters, x, y, s);
        result = hello.GetPoints();
        //        PrintVector(hello.GetTimes());
        return result;
    }

    inline std::vector< std::vector<double> > ServoTransition(std::shared_ptr<Parameters> parameters, std::vector<double> start_vector, std::vector<double> end_vector)
    {
        int tick_num = 40; //50个tick
        std::vector<std::vector<double>> result_matrix;
        std::vector<double> ttmp = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        for (int k = 0; k < tick_num; k++)
        {
            result_matrix.push_back(ttmp);
        }
        for (int i = 0; i < 12; i++)
        {
            std::vector<double> tmp = Planning(parameters, 0, (double)tick_num, start_vector[i], end_vector[i], 0, 0);

            for (int j = 0; j < tick_num; j++)
            {
                result_matrix[j][i] = tmp[j+1];
            }
        }
        return result_matrix;
    }

}


#endif // THREEINTERPOLATION_H
