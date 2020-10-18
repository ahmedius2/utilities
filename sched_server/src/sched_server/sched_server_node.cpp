/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * lidar_tracklet_compare_node.cpp
 *
 *  Created on: July, 05th, 2018
 */

#define __APP_NAME__ "sched_server"

#include <iostream>
#include <string>
#include <unordered_map>
#include <chrono>

#include <ros/ros.h>
#include "sched_server/sched_server.h"

bool ROSSchedServerApp::ServiceCallback(sched_server::SchedRequest& request, sched_server::SchedResponse& response){

	std::cout << "A sched request arrived from node: " << request.node_name << std::endl;

	if(config_map.count(request.node_name)){
	    response = config_map[request.node_name];
	    std::cout << "Responded with: " << response << std::endl;
	    return true;
	}
	else{
	    std::cout << "Couldn't find sched configuration for client node!" << std::endl;
	    return false;
	}
}


ROSSchedServerApp::ROSSchedServerApp(){
	ss_ = node_handle_.advertiseService("sched_service", &ROSSchedServerApp::ServiceCallback, this);

	ros::NodeHandle nhPrivate("~"); // private node handle
	nhPrivate.param<std::string>("sched_conf_path", sched_conf_path_, "sched_conf.csv");
	ROS_INFO("sched_conf_path: %s", sched_conf_path_.c_str());

	// Parse the file and put it inside unordered map
	std::ifstream infile(sched_conf_path_);

	std::string line;
	while (std::getline(infile, line))
	{
	    if(line[0] == '#') // comment
		continue;

	    std::istringstream iss(line);
	    std::vector<std::string> tokens;
	    std::string token;

	    while(std::getline(iss, token, ','))
	    {
		tokens.push_back(token);
	    }

	    std::string node_name = tokens[0];
	    sched_server::SchedResponse node_sched_conf;
	    node_sched_conf.execute = (tokens[1].compare("TRUE") == 0);
	    if(tokens[2].compare("SCHED_OTHER") == 0)
		node_sched_conf.policy = SCHED_OTHER;
	    else if(tokens[2].compare("SCHED_FIFO") == 0)
		node_sched_conf.policy = SCHED_FIFO;
	    else if(tokens[2].compare("SCHED_RR") == 0)
		node_sched_conf.policy = SCHED_RR;
	    else{
		std::cerr << "Unkown sched policy: " << tokens[2] << std::endl;
		std::cerr << "Using SCHED_OTHER instead" << std::endl;
		node_sched_conf.policy = SCHED_OTHER;
	    }

	    node_sched_conf.priority = std::stoi(tokens[3]);
	    node_sched_conf.set_affinity = (tokens[4].compare("TRUE") == 0);
	    std::stringstream ss_hex;
	    ss_hex << std::hex << tokens[5];
	    ss_hex >> node_sched_conf.cpu_affinity_mask;
	    node_sched_conf.rt_gang_id= std::stoi(tokens[6]);

	    config_map[node_name] = node_sched_conf;
	    //std::cout << "Registered node: " << node_name << " Conf:" << node_sched_conf << std::endl;
	}

}

void ROSSchedServerApp::Run(){
	ros::Rate r(10); // 10 hz
	while (ros::ok())
	{
	    // wait for clients
	    //std::cout << "I am a sched server...\n";
	    ros::spinOnce();
	    r.sleep();
	}
	ss_.shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, __APP_NAME__);
  ROSSchedServerApp app;

  app.Run();

  return 0;
}
