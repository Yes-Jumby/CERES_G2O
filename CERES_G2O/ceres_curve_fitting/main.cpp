#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include <fstream>
using namespace std;

// 代价函数的计算模型
struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST ( double x, double y ,double xw,double zw) : _x ( x ), _y ( y ),_xw ( xw ), _zw ( zw ) , _f ( 10000 ) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const abc,     // 模型参数，有3维
        T* residual ) const     // 残差2
    {
        T R11,R12,R13,R21,R22,R23,R31,R32;
        T u0,v0;
        T k1,k2;
        R11 = abc[0];
        R12 = abc[1];
        R13 = abc[2];
        R21 = abc[3];
        R22 = abc[4];
        R23 = abc[5];
        R31 = abc[6];
        R32 = abc[7];

        u0 = abc[8];
        v0 = abc[9];
        k1 = abc[10];
        k2 = abc[11];

        T devided_ = R31*_xw + R32*_zw+1.0;
        T Xc = (R11*_xw + R12*_zw+R13)/devided_;
        T Yc = (R21*_xw + R22*_zw+R23)/devided_;

        T r2 = Xc*Xc + Yc*Yc;
        T undistort_ = 1.0 + k1*r2 + k2*r2*r2;
        T Xc_undistort = Xc*undistort_;
        T Yc_undistort = Yc*undistort_;


        residual[0] = T ( _x ) - (_f*Xc_undistort + u0);
        residual[1] = T ( _y ) - (_f*Yc_undistort + v0);
        return true;
    }
    const double _x, _y,_xw, _zw,_f;    // x,y数据
};

int main ( int argc, char** argv )
{

    double param[12] = {0};            // abc参数的估计值

    vector<double> x_data, y_data,xw_data, zw_data;      // 数据

    //2D
    std::string fileName = "/media/ubuntu/1523870f-0fee-4896-9c94-398be1101136/Data/interSec.txt";
    std::ifstream fileInput(fileName.c_str());   // ´ò¿ªÎÄ¼þ£¬½¨Á¢Êý¾ÝÁ÷
    std::string lineStr;
    std::stringstream sstr;
    float x;
    float y;
    int i = 1;
    while (std::getline(fileInput, lineStr))    // ¶ÁÈ¡Ò»ÐÐ
    {
        sstr << lineStr;                          // ½«¶ÁÈ¡µÄÊý¾ÝÑ¹Èëµ½ sstr
        sstr >> x >> y;        //  Ê¹ÓÃsringsream ²ð·ÖÊý¾Ý
        std::cout << "line2d " << i << " " << x << " " << y << std::endl;
        i++;
        sstr.clear();          // this is important. ×¢Òâ½«stringsream¶ÔÏóÇå¿Õ£¬·ñÔòÒ»Ö±ÎªµÚÒ»ÐÐ
        x_data.push_back(x);
        y_data.push_back(y);
    }
    fileInput.close(); //¹Ø±ÕÊý¾ÝÁ÷
    std::cout << "line2D size:" << x_data.size()<< std::endl;

    //3D
    std::string fileName1 = "/media/ubuntu/1523870f-0fee-4896-9c94-398be1101136/Data/interSec3D.txt";
    std::ifstream fileInput1(fileName1.c_str());   // ´ò¿ªÎÄ¼þ£¬½¨Á¢Êý¾ÝÁ÷
    float z;
    while (std::getline(fileInput1, lineStr))    // ¶ÁÈ¡Ò»ÐÐ
    {
        sstr << lineStr;                          // ½«¶ÁÈ¡µÄÊý¾ÝÑ¹Èëµ½ sstr
        sstr >> x >> y>> z;        //  Ê¹ÓÃsringsream ²ð·ÖÊý¾Ý
        std::cout << "line3D " << i << " " << x << " " << y << std::endl;
        sstr.clear();          // this is important. ×¢Òâ½«stringsream¶ÔÏóÇå¿Õ£¬·ñÔòÒ»Ö±ÎªµÚÒ»ÐÐ
        xw_data.push_back(x);
        zw_data.push_back(z);
    }
    fileInput1.close(); //¹Ø±ÕÊý¾ÝÁ÷
    std::cout << "line3D size:" << xw_data.size()<< std::endl;

    // 构建最小二乘问题
    ceres::Problem problem;
    for ( int i=0; i<12; i++ )
    {
        problem.AddResidualBlock (     // 向问题中添加误差项
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 2, 12> (
                new CURVE_FITTING_COST ( x_data[i], y_data[i],xw_data[i], zw_data[i] )
            ),
            nullptr,            // 核函数，这里不使用，为空
            param                 // 待估计参数
        );
    }

    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve ( options, &problem, &summary );  // 开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    // 输出结果
    cout<<summary.BriefReport() <<endl;
    cout<<"estimated a,b,c = ";
    for ( auto a:param ) cout<<a<<" ";
    cout<<endl;

    return 0;
}

