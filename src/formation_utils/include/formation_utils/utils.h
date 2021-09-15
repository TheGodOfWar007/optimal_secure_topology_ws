#ifndef UTIL_H
#define UTIL_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <XmlRpcValue.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>

#define DEG2RAD(x)              (x * M_PI/180) // x*pi/180
#define RAD2DEG(x)              (x * 180/M_PI) // x*180/pi

#define pass                    (void)0 // Similar to python's pass. Defined for any possible future use. You are advised against using this unless it is very very necessary/you are out of options.

#define BURGER_MAX_VEL          0.22 // m/s
#define BURGER_MAX_ANG_VEL      2.84 // rad/s
#define WAFFLEPI_MAX_VEL        0.26 // m/s
#define WAFFLEPI_MAX_ANG_VEL    1.82 // rad/s

namespace Eigen {
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfRowMajor;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdRowMajor;
}

namespace FormationUtils{
    
    template <class T>
    int findVectorElement(std::vector<T> &vec, T find_element) {
        typename std::vector<T>::iterator it1 = std::find(vec.begin(), vec.end(), find_element);
        int idx = std::distance(vec.begin(), it1);
        if (!(idx < vec.size())) {
            ROS_WARN("Request element not found in the vector provided.");
        }
        return idx;
    }


    Eigen::Quaterniond euler_to_quaternion(float r, float p, float y);

    Eigen::Vector3d quaternion_to_euler(Eigen::Quaterniond q);
    
    void saturation_function(double &e, double &max, double &min);

    template <class T>
    void xmlrpc_to_matrix(int rows, int cols, XmlRpc::XmlRpcValue& XmlConfig, T& mat) {
        mat.resize(1, rows*cols);
        for (int i = 0; i < rows*cols; i++) {
            std::ostringstream ostr;
            ostr << XmlConfig[i];
            std::istringstream istr(ostr.str());
            istr >> mat(0,i);
        }
        mat.resize(rows, cols);
    }


    template <class T>
    void matrixEigentoFloatMsg(Eigen::MatrixBase<T> &mat, std_msgs::Float64MultiArray &msg) {
        if (msg.layout.dim.size() != 2){
            msg.layout.dim.resize(2);
        }
        msg.layout.dim[0].stride = mat.rows() * mat.cols();
        msg.layout.dim[0].size = mat.rows();
        msg.layout.dim[1].stride = mat.cols();
        msg.layout.dim[1].size = mat.cols();
        if ((int)msg.data.size() != mat.size()){
            msg.data.resize(mat.size());
        }
        int ii = 0;
        for (int i = 0; i < mat.rows(); ++i) {
            for (int j = 0; j < mat.cols(); ++j) {
            msg.data[ii++] = mat.coeff(i, j);
            }
        }
    }

    template <class T1, class T2>
    void matrixEigentoArrayMsg(Eigen::MatrixBase<T1> &mat, T2 &msg) {
        if (msg.layout.dim.size() != 2){
            msg.layout.dim.resize(2);
        }
        msg.layout.dim[0].stride = mat.rows() * mat.cols();
        msg.layout.dim[0].size = mat.rows();
        msg.layout.dim[1].stride = mat.cols();
        msg.layout.dim[1].size = mat.cols();
        if ((int)msg.data.size() != mat.size()){
            msg.data.resize(mat.size());
        }
        int ii = 0;
        for (int i = 0; i < mat.rows(); ++i) {
            for (int j = 0; j < mat.cols(); ++j) {
            msg.data[ii++] = mat.coeff(i, j);
            }
        }
    }

    template <class T>
    void floatMsgtoMatrixEigen(std_msgs::Float64MultiArray &msg, Eigen::MatrixBase<T> &mat) {
        ROS_ASSERT(msg.layout.dim.size() == 2);
        mat.resize(1, msg.layout.dim[0].stride);
        for(int i = 0; i < msg.layout.dim[0].stride; i++) {
            mat(0, i) = msg.data[i];
        }
        mat.resize(msg.layout.dim[0].size, msg.layout.dim[1].size);
    }

    template <class T1, class T2>
    void arrayMsgtoMatrixEigen(T1 &msg, Eigen::MatrixBase<T2> &mat) {

        ROS_ASSERT(msg.layout.dim.size() == 2);
        mat.resize(1, msg.layout.dim[0].stride);
        for(int i = 0; i < msg.layout.dim[0].stride; i++) {
            mat(0, i) = msg.data[i];
        }
        mat.resize(msg.layout.dim[0].size, msg.layout.dim[1].size);
    }

    template<class T>
    T calculateLaplacianMatrix(T A_) {
        T D_out = T::Zero(A_.rows(), A_.cols());
        D_out.diagonal() = A_.rowwise().sum();
        T L_ = D_out - A_;
        return L_;
    }

    template<class T>
    T calculateOutDegreeMatrix(T A_) {
        T D_out = T::Zero(A_.rows(), A_.cols());
        D_out.diagonal() = A_.rowwise().sum();
        return D_out;
    }

    template<class T>
    T calculateInDegreeMatrix(T A_) {
        T D_in = T::Zero(A_.rows(), A_.cols());
        D_in.diagonal() = A_.colwise().sum();
        return D_in;
    }

}

#endif //UTIL_H