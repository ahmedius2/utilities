// header
//

#ifndef SCHED_CLIENT_H
#define SCHED_CLIENT_H

#include <string>
#include <sstream>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <time.h>
#include <ros/ros.h>
#include "sched_server/Sched.h"

namespace SchedClient
{
bool ConfigureSchedOfCallingThread(){
    sched_server::Sched sched_srv_msg;

    sched_srv_msg.request.node_name = ros::this_node::getName() ;
    ros::service::waitForService("sched_service", 3); // 3 seconds timout
    if (!ros::service::call("sched_service", sched_srv_msg))
    {
        std::cerr << "Coudln't get response from server for node "
                  << sched_srv_msg.request.node_name << std::endl;
        return false;
    }

    sched_server::SchedResponse& sr = sched_srv_msg.response;
    if(!sr.execute){
        ros::shutdown();
        return false;
    }

    struct sched_param sp;
    sp.sched_priority= (sr.policy == SCHED_OTHER) ? 0 : sr.priority;

    auto thread_id=pthread_self();
    int ret = pthread_setschedparam(thread_id, sr.policy, &sp);
    if(ret){
        std::cerr << "pthread_setschedparam returned error: "
                  << std::strerror(errno) << std::endl;
        std::cerr << "Arguments are policy:" << sr.policy
                  << " priority:" << sp.sched_priority << std::endl;
        return false;
    }

    if(sr.policy == SCHED_OTHER){
        ret = setpriority(PRIO_PROCESS, getpid(), sr.priority);
        if(ret == -1){
            std::cerr << "setpriority returned error: "
                      << std::strerror(errno) << std::endl;
            std::cerr << "Argument is priority:" << sr.priority << std::endl;
            return false;
        }
    }

    if(sr.set_affinity){
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        auto mask = sr.cpu_affinity_mask;
        for(unsigned i=0; mask != 0; ++i, mask = mask>>1)
            if(mask & 0x1)
                CPU_SET(i,&cpuset);
        ret = pthread_setaffinity_np(thread_id,sizeof(cpuset),&cpuset);
        if(ret){
            std::cerr << "pthread_setaffinity_np returned error: "
                      << std::strerror(errno) << std::endl;
            return false;
        }
    }
    return true;
}
}

#endif //SCHED_CLIENT_H
