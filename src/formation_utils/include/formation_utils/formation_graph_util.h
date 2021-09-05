#ifndef FORMATION_GRAPH_UTIL_H
#define FORMATION_GRAPH_UTIL_H

#include <ros/ros.h>
#include <formation_msgs/FormationGraphParams.h>
#include <formation_utils/formation_handle.h>

namespace FormationUtils {
    class GraphInterface {

        public:
            GraphInterface(FormationHandle* fh_);

            virtual ~GraphInterface();

            virtual void InterfaceServiceAdvertise();

        protected:
            virtual bool InterfaceServiceCallback(formation_msgs::FormationGraphParams::Request &req,
                                formation_msgs::FormationGraphParams::Response &res);

        protected:
            ros::ServiceServer interface_srv_server;
            const std::string interface_srv_name= "/formation_graph/interface";
            FormationHandle* fh;
    };
}

#endif //FORMATION_GRAPH_UTIL_H