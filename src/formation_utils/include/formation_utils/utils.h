#ifndef UTIL_H
#define UTIL_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <XmlRpcValue.h>
#include <std_msgs/Float64MultiArray.h>

namespace Eigen {
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfRowMajor;
}

namespace FormationUtils{
    
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

    Eigen::Quaternionf euler_to_quaternion(float r, float p, float y);

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

    template <class T>
    void floatMsgtoMatrixEigen(std_msgs::Float64MultiArray &msg, Eigen::MatrixBase<T> &mat) {
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