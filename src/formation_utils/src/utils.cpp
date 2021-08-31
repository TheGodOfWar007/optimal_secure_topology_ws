#include <formation_utils/utils.h>

namespace FormationUtils{
    Eigen::Quaternionf euler_to_quaternion(float r, float p, float y) {
        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(p, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(y, Eigen::Vector3f::UnitZ());
        
        return q;
    }
}