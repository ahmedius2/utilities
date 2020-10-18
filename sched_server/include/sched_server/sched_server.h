// header
//

#ifndef PROJECT_SCHED_SERVER_H
#define PROJECT_SCHED_SERVER_H

#define __APP_NAME__ "sched_server"

#include <string>
#include <unordered_map>
#include <ros/ros.h>
#include "sched_server/Sched.h"

class ROSSchedServerApp
{
    ros::NodeHandle node_handle_;
    ros::ServiceServer ss_;
    std::string sched_conf_path_;
    std::unordered_map<std::string, sched_server::SchedResponse> config_map;

    bool ServiceCallback(sched_server::SchedRequest& request, sched_server::SchedResponse& response);

public:
    ROSSchedServerApp();

    void Run();
};

#endif //PROJECT_SCHED_SERVER_H
