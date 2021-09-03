#include <formation_utils/utils.h>

namespace FormationUtils{
    Eigen::Quaterniond euler_to_quaternion(float r, float p, float y) {
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
        
        return q;
    }

    Eigen::Vector3d quaternion_to_euler(Eigen::Quaterniond q) {
        return q.toRotationMatrix().eulerAngles(0,1,2);
    }

    void saturation_function(double &e, double &max, double &min) {
        if (e > max) {
            e = max;
        }
        if (e < min) {
            e = min;
        }   
    }
}