//#include <iostream>
//#include <opencv2/core/core.hpp>
//#include <ceres/ceres.h>
//#include <chrono>
//#include<Eigen/Core>
//#include<Eigen/Dense>
//#include<fstream>
//#include<math.h>
//using namespace std;
//double sumVector(vector<double> x)
//{
//    double sum = 0.0;
//    for (int i = 0; i<x.size(); ++i)
//    {
//        sum += x[i];
//    }
//    return sum / x.size();
//}
//// 代价函数的计算模型
//struct Resection
//{
//    Resection(double X, double Y, double Z, double x, double y, 
//             double fx, double fy, double cx, double cy, 
//             double *lensDistort) :_X(X), _Y(Y), _Z(Z), observed_x_(x), observed_y_(y)
//            , fx_(fx), fy_(fy), cx_(cx), cy_(cy), lensDistort_(lensDistort) {}
//    // 残差的计算
//    template <typename T>
//    bool operator() (const T* const camPose, T* residual) const     // 残差
//    {
//        T AngleAxis[3] = { camPose[0],camPose[1],camPose[2]};
//        T P_W_[3] = {_X,_Y,_Z};
//        T P_C_[3];
//        ceres::AngleAxisRotatePoint(AngleAxis, P_W_, P_C_);
//        P_C_[0] = P_C_[0] + camPose[3];
//        P_C_[1] = P_C_[1] + camPose[4];
//        P_C_[2] = P_C_[2] + camPose[5];
//
//        // Compute the center of distortion. The sign change comes from
//        // the camera model that Noah Snavely's Bundler assumes, whereby
//        // the camera coordinate system has a negative z axis.
//
//        T xp = P_C_[0] / P_C_[2];
//        T yp = P_C_[1] / P_C_[2];
//
//        // Apply second and fourth order radial distortion.
//        const T& l1 = lensDistort_[0];
//        const T& l2 = lensDistort_[1];
//        T r2 = xp*xp + yp*yp;
//        T distortion = 1.0 + r2  * (l1 + l2  * r2);
//
//        // Compute final projected point position.
//        T xp_undistort_ = distortion * xp;
//        T yp_undistort_ = distortion * yp;
//
//        T predicted_x = fx_ * xp_undistort_ + cx_;
//        T predicted_y = fy_ * yp_undistort_ + cy_;
//
//        // The error is the difference between the predicted and observed position.
//        residuals[0] = predicted_x - T(observed_x_);
//        residuals[1] = predicted_y - T(observed_y_);
//
//        return true; //千万不要写成return 0,要写成return true
//    }
//private:
//    const double _X, _Y, _Z;
//    const double observed_x_,observed_y_;
//    const double   fx_, fy_, cx_,cy_;
//    const double *lensDistort_;
//};
//
//int main1221211(int argc, char** argv)
//{
//    google::InitGoogleLogging(argv[0]);
//    //read file
//    string filename = argv[1];
//    ifstream fin(filename.c_str());
//    string line;
//    vector<double> x;
//    vector<double> y;
//    vector<double> X;
//    vector<double> Y;
//    vector<double> Z;
//    while (getline(fin, line))
//    {
//        char* pEnd;
//        double a, b, c, d, e;
//        a = strtod(line.c_str(), &pEnd);
//        b = strtod(pEnd, &pEnd);
//        c = strtod(pEnd, &pEnd);
//        d = strtod(pEnd, &pEnd);
//        e = strtod(pEnd, nullptr);
//        x.push_back(a);
//        y.push_back(b);
//        X.push_back(c);
//        Y.push_back(d);
//        Z.push_back(e);
//    }
//    //初始化参数
//    double camPose[6] = { 0 };
//    camPose[0] = sumVector(X);
//    camPose[1] = sumVector(Y);
//    camPose[2] = sumVector(Z);
//    double f = 153.24; //mm
//                       //    camPose[2]=50*f;
//                       //构建最小二乘
//    ceres::Problem problem;
//    try
//    {
//        for (int i = 0; i<x.size(); ++i)
//        {
//            ceres::CostFunction *costfunction = new ceres::AutoDiffCostFunction<Resection, 2, 6>(new Resection(X[i], Y[i], Z[i], x[i] / 1000, y[i] / 1000, f / 1000));
//            //将残差方程和观测值加入到problem,nullptr表示核函数为无，
//            problem.AddResidualBlock(costfunction, nullptr, camPose);
//        }
//    }
//    catch (...)
//    {
//        cout << "costFunction error" << endl;
//    }
//
//    // 配置求解器
//    ceres::Solver::Options options;     // 这里有很多配置项可以填
//    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
//    options.minimizer_progress_to_stdout = true;   // 输出到cout
//                                                   //    options.max_num_iterations=25;
//    ceres::Solver::Summary summary;                // 优化信息
//    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//    ceres::Solve(options, &problem, &summary);  // 开始优化
//    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
//    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
//
//    // 输出结果
//    cout << summary.BriefReport() << endl;
//    cout << "*************结果****************" << endl;
//    cout << "estimated Xs,Ys,Zs,omega,pho,kappa = ";
//    for (auto p : camPose) cout << p << " ";
//    cout << endl;
//
//    return 0;
//}
//
