#ifndef FORMATION_STATE_PUBLISHER_H
#define FORMATION_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <robot_state_publisher_modified/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <formation_utils/formation_handle.h>
#include <urdf/model.h>

namespace FormationUtils {
    class FormationStatePublisher {
        public:
            FormationStatePublisher(FormationHandle* _fh) 
            : fh(_fh) { };

            virtual ~FormationStatePublisher() { };

            virtual bool init();

            virtual bool publishFixedTransforms();

            virtual bool publishTransforms(const std::vector<std::map<std::string, double>>& joint_positions, const ros::Time& time);

        protected:
            robot_state_publisher::RobotStatePublisher robotStatePublisher;
            const std::vector<std::string>* uid_list_ptr; // deref vector_ptr->at() to check range bounds.
            FormationHandle* fh; 
            KDL::Tree robot_desc_KDL_tree;
            int num_bots;
        
        protected:
            /**
             * @brief : Flag: Decides whether to use the latched static transform for fixed segments. 
             * 
             */
            bool USE_STATIC_TF;
    };
}

#endif // FORMATION_STATE_PUBLISHER_H