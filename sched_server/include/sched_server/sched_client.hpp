// header
//

#ifndef SCHED_CLIENT_H
#define SCHED_CLIENT_H

#include <string>
#include <sstream>
#include <cstdlib>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <time.h>
#include <ros/ros.h>
#include "sched_server/Sched.h"
#include "sched_server/rtg_lib.h"

namespace SchedClient
{
bool ConfigureSchedOfCallingThread(std::string thread_name=""){
    sched_server::Sched sched_srv_msg;

    sched_srv_msg.request.node_name = (thread_name.compare("") == 0) ? ros::this_node::getName() : thread_name;
    ros::service::waitForService("sched_service", 3); // 3 seconds timout
    while (!ros::service::call("sched_service", sched_srv_msg))
    {
        std::cerr << "Coudln't get response from server for node "
                  << sched_srv_msg.request.node_name << std::endl;
        std::cerr << "Going to try again connecting after 1 second...\n";
        sleep(1);
        //return false;
    }

    sched_server::SchedResponse& sr = sched_srv_msg.response;
    if(!sr.execute){
        std::cerr << "Node " << sched_srv_msg.request.node_name
                  << " is not configured to execute, exiting...\n";
        ros::shutdown();
        exit(0);
        // won't reach here
        return false;
    }

    struct sched_param sp;
    sp.sched_priority= (sr.policy == SCHED_OTHER) ? 0 : sr.priority;

#ifdef USE_PTHREAD
    auto thread_id=pthread_self();
    int ret = pthread_setschedparam(thread_id, sr.policy, &sp);
#else
    int ret = sched_setscheduler(0, sr.policy,&sp);
#endif
    if(ret){
        std::cerr << "pthread_setschedparam or sched_setscheduler returned error: "
                  << std::strerror(errno) << std::endl;
        std::cerr << "Arguments are policy:" << sr.policy
                  << " priority:" << sp.sched_priority << std::endl;
        return false;
    }

    if(sr.policy == SCHED_OTHER){
        ret = setpriority(PRIO_PROCESS, 0, sr.priority);
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
#ifdef USE_PTHREAD
        ret = pthread_setaffinity_np(thread_id,sizeof(cpuset),&cpuset);
#else
        ret = sched_setaffinity(0,sizeof(cpuset),&cpuset);
#endif
        if(ret){
            std::cerr << "pthread_setaffinity_np or sched_setaffinity returned error: "
                      << std::strerror(errno) << std::endl;
            return false;
        }
    }

    if(sr.rt_gang_id > 0){
    	int ret = sched_getscheduler(0);
    	ROS_INFO("Node %s is being configured to use rt gang id: %ld with sched policy %d", sched_srv_msg.request.node_name.c_str(), sr.rt_gang_id, ret);
        pthread_barrier_t* barrier = rtg_member_setup(sr.rt_gang_id, 0, 0);
        rtg_member_sync(barrier);
    }

    ROS_INFO("Node %s is configured and ready to execute", sched_srv_msg.request.node_name.c_str());
    std::cerr << "Node " << sched_srv_msg.request.node_name 
	      << " is configured and ready to execute with conf:" << sr;


    return true;
}
}

#endif //SCHED_CLIENT_H
