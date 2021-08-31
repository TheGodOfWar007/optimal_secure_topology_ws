#include <ros/ros.h>
#include <ros/console.h>
#include <formation_msgs/FormationGraphParams.h>
#include <formation_utils/formation_handle.h>
#include <formation_utils/formation_graph_util.h>

namespace FormationUtils {
    GraphInterface::GraphInterface(FormationHandle* fh_) : fh(fh_) { 
        ROS_INFO("Initializing Graph Interface.");
    }

    void GraphInterface::InterfaceServiceAdvertise() {
        interface_srv_server = fh->nh.advertiseService(interface_srv_name, &GraphInterface::InterfaceServiceCallback, this);
    }

    bool GraphInterface::InterfaceServiceCallback(formation_msgs::FormationGraphParams::Request &req,
                                formation_msgs::FormationGraphParams::Response &res) {
        
        if(req.UPDATING_ADJACENCY_MATRIX) {
            Eigen::MatrixXfRowMajor A_;
            floatMsgtoMatrixEigen<Eigen::MatrixXfRowMajor>(req.A_, A_);
            if(A_.size() == 0) {
                ROS_ERROR("Empty update matrix received. Invalid request.");
                return false;
            }
            fh->setAdjacencyMatrix(A_);
            Eigen::MatrixXfRowMajor L_;
            L_ = calculateLaplacianMatrix<Eigen::MatrixXfRowMajor>(A_);
            fh->setLaplacianMatrix(L_);
        }
        if(req.REQUESTING_PARAMS) {
            Eigen::MatrixXfRowMajor A_;
            Eigen::MatrixXfRowMajor L_;
            Eigen::MatrixXfRowMajor D_out;
            Eigen::MatrixXfRowMajor D_in;
            A_ = fh->getAdjacencyMatrix();
            L_ = calculateLaplacianMatrix(A_);
            D_out = calculateOutDegreeMatrix(A_);
            D_in = calculateInDegreeMatrix(A_);
            matrixEigentoFloatMsg<Eigen::MatrixXfRowMajor>(A_, res.A_);
            matrixEigentoFloatMsg<Eigen::MatrixXfRowMajor>(D_out, res.D_out);
            matrixEigentoFloatMsg<Eigen::MatrixXfRowMajor>(D_in, res.D_in);
            matrixEigentoFloatMsg<Eigen::MatrixXfRowMajor>(L_, res.L_);
        }

        if(!(req.REQUESTING_PARAMS || req.UPDATING_ADJACENCY_MATRIX)) {
            ROS_WARN("Nothing requested from the graph parameter service.");
        }

        return true;
    }

    GraphInterface::~GraphInterface() { };
}