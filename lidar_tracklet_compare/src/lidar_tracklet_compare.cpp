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
 *  v1.0: amc-nu
 *
 * lidar_tracklet_compare_node.cpp
 *
 *  Created on: July, 05th, 2018
 */

#include "lidar_tracklet_compare/lidar_tracklet_compare.h"

void ROSLidarTrackletCompareApp::DetectionsCallback(
        const autoware_msgs::DetectedObjectArray::ConstPtr &in_detections_msg)
{
    const std::lock_guard<std::mutex> lock(manual_sync_mutex_);

    if (tracklets_queue_.empty()){
        detections_queue_.push(in_detections_msg);
    }
    else{
        SyncedDetectionsTrackletsCallback(in_detections_msg, tracklets_queue_.front());
        tracklets_queue_.pop();
    }
}

void ROSLidarTrackletCompareApp::TrackletsCallback(
        const autoware_msgs::DetectedObjectArray::ConstPtr &in_tracklets_msg)
{
    const std::lock_guard<std::mutex> lock(manual_sync_mutex_);

    if (detections_queue_.empty()){
        tracklets_queue_.push(in_tracklets_msg);
    }
    else{
        SyncedDetectionsTrackletsCallback(detections_queue_.front(), in_tracklets_msg);
        detections_queue_.pop();
    }
}

void
ROSLidarTrackletCompareApp::SyncedDetectionsTrackletsCallback(
        const autoware_msgs::DetectedObjectArray::ConstPtr &in_detections_msg,
        const autoware_msgs::DetectedObjectArray::ConstPtr &in_tracklets_msg)
{
    autoware_msgs::DetectedObjectArray matched_objects, missed_objects
            , mispredicted_objects;
    matched_objects.header = in_detections_msg->header;
    mispredicted_objects.header = in_detections_msg->header;
    missed_objects.header = in_tracklets_msg->header;

    MatchDetectionsAndTracklets(in_detections_msg, in_tracklets_msg,
                                matched_objects, missed_objects, mispredicted_objects);

    matched_objects_publisher_.publish(matched_objects);
    missed_objects_publisher_.publish(missed_objects);
    mispredicted_objects_publisher_.publish(mispredicted_objects);
}

void
ROSLidarTrackletCompareApp::MatchDetectionsAndTracklets(
        const autoware_msgs::DetectedObjectArray::ConstPtr &in_detections_msg,
        const autoware_msgs::DetectedObjectArray::ConstPtr &in_tracklets_msg,
        autoware_msgs::DetectedObjectArray &out_matched_objects,
        autoware_msgs::DetectedObjectArray &out_missed_objects,
        autoware_msgs::DetectedObjectArray &out_mispredicted_objects)
{
    std::vector<bool> is_tracklet_detected(in_tracklets_msg->objects.size(), false);
    auto& tracklets = in_tracklets_msg->objects;

    for (auto obj : in_detections_msg->objects){
        bool matched = false;
        for(auto i=0; i<tracklets.size(); ++i ){
            if(is_tracklet_detected[i])
                continue;

            auto tracklet = tracklets[i];
            // this is inneficient but lets stick like this for now
            double uio = Get2DIoUOfObjects(obj, tracklet);
            if(uio > overlap_threshold_){
                obj.color = matched_color_;
                out_matched_objects.objects.push_back(obj);
                is_tracklet_detected[i] = matched = true;
                break;
            }

            //            double d = Get2DDistanceBetweenObjects(obj, tracklet);
            //            if(d <= 2){ // Distance is smaller than 2 meters, let's check
            //                out_matched_objects.objects.push_back(obj);
            //                missed  = false;
            //                break;
            //            }
        }

        if(!matched){
            obj.color = mispredicted_color_;
            out_mispredicted_objects.objects.push_back(obj);
        }
    }

    // If there are tracklets that were not matched with anything,
    // they are missed
    for(auto i=0; i < is_tracklet_detected.size(); ++i ){
        if(!is_tracklet_detected[i]){
            auto obj = tracklets[i];
            obj.color = missed_color_;
            out_missed_objects.objects.push_back(obj);
        }
    }

}

double ROSLidarTrackletCompareApp::Get2DIoUOfObjects(
        const autoware_msgs::DetectedObject &in_object1,
        const autoware_msgs::DetectedObject &in_object2)
{
    auto obj1_vertices = getVerticesOf3DObject(in_object1);
    // Just XY dimension is enough, remove half of the points in Z
    obj1_vertices.erase(obj1_vertices.begin(), obj1_vertices.begin()+4);
    auto obj2_vertices = getVerticesOf3DObject(in_object2);
    // Just XY dimension is enough, remove half of the points in Z
    obj2_vertices.erase(obj2_vertices.begin(), obj2_vertices.begin()+4);

    auto sum_of_areas = polygonArea(obj1_vertices) + polygonArea(obj2_vertices);

    // calculate the points in each others area
    std::array<float,4> obj1_limits, obj2_limits;

    // maybe these can be done faster! because the points are sorted as:
    // 1 1, -1 1, -1 -1, 1 -1
    getXYLimitsOfVertices(obj1_vertices, obj1_limits);
    getXYLimitsOfVertices(obj2_vertices, obj2_limits);

    //populate points
    const auto neighboorPointsDistance = 0.01; //meters
    populateVertices(obj1_vertices, neighboorPointsDistance);
    populateVertices(obj2_vertices, neighboorPointsDistance);

    jsk_recognition_utils::Vertices intersect_points1, intersect_points2;
    for(auto& v : obj1_vertices){
        if(is2DPointInsideLimits(v, obj2_limits)){
            intersect_points1.push_back(v);
        }
    }
    for(auto &v : obj2_vertices){
        if(is2DPointInsideLimits(v, obj1_limits)){
            intersect_points2.push_back(v);
        }
    }

    if(intersect_points1.size() < 2 || intersect_points2.size() < 2)
        return 0; // not enough intersection!

    // I HAVE TO MAKE SURE THAT POINTS ARE ORDERED CLOCKWISE OR COUNTER CLOCKWISE
    auto d1 = (intersect_points1.back() - intersect_points2[0]).squaredNorm();
    auto d2 = (intersect_points1.back() - intersect_points2.back()).squaredNorm();

    if(d1 < d2){
        auto it = intersect_points2.cbegin();
        auto end_it = intersect_points2.cend();
        while(it != end_it)
            intersect_points1.push_back(*(it++));
    }
    else{
        auto it = intersect_points2.crbegin();
        auto end_it = intersect_points2.crend();
        while(it != end_it)
            intersect_points1.push_back(*(it++));
    }

    // debug
    std::cout << "Intersection points:\n";
    for(auto& v : intersect_points1)
        std::cout << v << std::endl;

    // calculate area and IoU
    auto intersect_area = polygonArea(intersect_points1);

    return intersect_area / (sum_of_areas - intersect_area);
}

// Shoelace formula
// thank you geeksforgeeks!
double ROSLidarTrackletCompareApp::polygonArea(
        jsk_recognition_utils::Vertices &vertices)
{
    // Initialze area
    double area = 0.0;

    // Calculate value of shoelace formula
    int j = vertices.size() - 1;
    for (int i = 0; i < vertices.size(); i++)
    {
        area += (vertices[j][0] + vertices[i][0])
                * (vertices[j][1] - vertices[i][1]);
        j = i;  // j is previous vertex to i
    }

    // Return absolute value
    return std::abs(area / 2.0);
}

void ROSLidarTrackletCompareApp::populateVertices(
        jsk_recognition_utils::Vertices &vertices,
        float distBetweenNeighboorPointsInMeters)
{
    jsk_recognition_utils::Vertices new_vertices;

    vertices.push_back(vertices[0]);
    for(auto i=0; i<vertices.size()-1; ++i){
        float x_diff = vertices[i+1][0] - vertices[i][0];
        float y_diff = vertices[i+1][1] - vertices[i][1];
        unsigned pointsToPopulate =
                std::floor(std::sqrt(std::pow(x_diff,2) + std::pow(y_diff,2))
                           / distBetweenNeighboorPointsInMeters) + 1;

        float x_step = x_diff / pointsToPopulate;
        float y_step = y_diff / pointsToPopulate;
        auto& v = vertices[i];

        for(auto j = 0; j < pointsToPopulate ; ++j)
            new_vertices.push_back(jsk_recognition_utils::Point(
                                       v[0]+x_step*j,
                                   v[1]+y_step*j,
                    v[2]));
    }
    vertices = new_vertices;
}

//out: min x min y max x max y
void ROSLidarTrackletCompareApp::getXYLimitsOfVertices(
        const jsk_recognition_utils::Vertices &vertices,
        std::array<float,4> &out_limits)
{
    out_limits[0]=FLT_MAX, out_limits[1]=FLT_MAX;
    out_limits[2]=FLT_MIN, out_limits[3]=FLT_MIN;

    for(auto &v : vertices){
        out_limits[0] = std::min(out_limits[0],v(0));
        out_limits[1] = std::min(out_limits[1],v(1));
        out_limits[2] = std::max(out_limits[2],v(0));
        out_limits[3] = std::max(out_limits[3],v(1));
    }
}

inline bool ROSLidarTrackletCompareApp::is2DPointInsideLimits(
        const jsk_recognition_utils::Point &p, const std::array<float,4> &in_limits)
{
    if(p(0) > in_limits[0] && p(1) > in_limits[1]
            && p(0) < in_limits[2] && p(1) < in_limits[3])
        return true;
    return false;
}

jsk_recognition_utils::Vertices ROSLidarTrackletCompareApp::getVerticesOf3DObject(
        const autoware_msgs::DetectedObject &in_object)
{
    Eigen::Vector3f pos;
    pos << in_object.pose.position.x,
            in_object.pose.position.y,
            in_object.pose.position.z;

    Eigen::Quaternionf rot(in_object.pose.orientation.w,
                           in_object.pose.orientation.x,
                           in_object.pose.orientation.y,
                           in_object.pose.orientation.z);

    std::vector<double> dims = {
        in_object.dimensions.x,
        in_object.dimensions.y,
        in_object.dimensions.z
    };

    jsk_recognition_utils::Cube cube(pos, rot, dims);

    return cube.vertices();
}

double ROSLidarTrackletCompareApp::Get2DDistanceBetweenObjects(
        const autoware_msgs::DetectedObject &in_object1,
        const autoware_msgs::DetectedObject &in_object2)
{
    auto x_square = in_object1.pose.position.x - in_object2.pose.position.x;
    x_square *= x_square;
    auto y_square = in_object1.pose.position.y - in_object2.pose.position.y;
    y_square *= y_square ;

    return sqrt(x_square + y_square); // return meters
}

void
ROSLidarTrackletCompareApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
    //get params
    //  std::string min_car_dimensions, min_person_dimensions, min_truck_dimensions;
    std::string lidar_detections_topic, lidar_tracklets_topic;
    std::string matched_objects_topic, missed_objects_topic, mispredicted_objects_topic;
    //std::string name_space_str = ros::this_node::getNamespace();
    bool sync_topics = false;

    ROS_INFO(
                "[%s] This node requires: Range detections and tracklets being published.",
                __APP_NAME__);
    in_private_handle.param<std::string>("lidar_detections_topic", lidar_detections_topic,
                                         "/detection/lidar_detector/objects");
    ROS_INFO("[%s] lidar_detections_topic: %s", __APP_NAME__,
             lidar_detections_topic.c_str());

    in_private_handle.param<std::string>("lidar_tracklets_topic", lidar_tracklets_topic,
                                         "/detection/tracklets/objects");
    ROS_INFO("[%s] lidar_tracklets_topic: %s", __APP_NAME__,
             lidar_tracklets_topic.c_str());

    in_private_handle.param<std::string>("matched_objects_topic", matched_objects_topic,
                                         "/detection/lidar_matched/objects");
    ROS_INFO("[%s] matched_objects_topic: %s", __APP_NAME__,
             matched_objects_topic.c_str());

    in_private_handle.param<std::string>("missed_objects_topic", missed_objects_topic,
                                         "/detection/lidar_missed/objects");
    ROS_INFO("[%s] missed_objects_topic: %s", __APP_NAME__,
             missed_objects_topic.c_str());

    in_private_handle.param<std::string>("mispredicted_objects_topic", mispredicted_objects_topic,
                                         "/detection/lidar_mispredicted/objects");
    ROS_INFO("[%s] mispredicted_objects_topic: %s", __APP_NAME__,
             mispredicted_objects_topic.c_str());

    in_private_handle.param<double>("overlap_threshold", overlap_threshold_, 0.6);
    ROS_INFO("[%s] overlap_threshold: %f", __APP_NAME__, overlap_threshold_);

    in_private_handle.param<bool>("sync_topics", sync_topics, false);
    ROS_INFO("[%s] sync_topics: %d", __APP_NAME__, sync_topics);

    //generate subscribers and sychronizers
    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, lidar_detections_topic.c_str());
    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, lidar_tracklets_topic.c_str());

    if (!sync_topics)
    {
        detections_subscriber_ = in_private_handle.subscribe(lidar_detections_topic,
                                                             10,
                                                             &ROSLidarTrackletCompareApp::DetectionsCallback,
                                                             this);

        tracklets_subscriber_ = in_private_handle.subscribe(lidar_tracklets_topic,
                                                            10,
                                                            &ROSLidarTrackletCompareApp::TrackletsCallback,
                                                            this);
    }
    else
    {
        detections_filter_subscriber_ =
                new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>
                (node_handle_,
                 lidar_detections_topic,
                 10);
        tracklets_filter_subscriber_ =
                new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>
                (node_handle_,
                 lidar_tracklets_topic,
                 10);
        detection_tracklet_synchronizer_ =
                new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
                                                               *detections_filter_subscriber_,
                                                               *tracklets_filter_subscriber_);

        detection_tracklet_synchronizer_->registerCallback(
                    boost::bind(&ROSLidarTrackletCompareApp::SyncedDetectionsTrackletsCallback,
                                this, _1, _2));
    }

    matched_objects_publisher_ =
            node_handle_.advertise<autoware_msgs::DetectedObjectArray>(
                matched_objects_topic, 10);
    missed_objects_publisher_ =
            node_handle_.advertise<autoware_msgs::DetectedObjectArray>(
                missed_objects_topic, 10);
    mispredicted_objects_publisher_ =
            node_handle_.advertise<autoware_msgs::DetectedObjectArray>(
                mispredicted_objects_topic, 10);


    ROS_INFO("[%s] Publishing matched objects in %s", __APP_NAME__, matched_objects_topic.c_str());
    ROS_INFO("[%s] Publishing missed objects in %s", __APP_NAME__, missed_objects_topic.c_str());
    ROS_INFO("[%s] Publishing mispredicted objects in %s", __APP_NAME__, mispredicted_objects_topic.c_str());
}


void
ROSLidarTrackletCompareApp::Run()
{
    ros::NodeHandle private_node_handle("~");

    InitializeROSIo(private_node_handle);

    ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

    ros::spin();

    ROS_INFO("[%s] END", __APP_NAME__);
}

ROSLidarTrackletCompareApp::ROSLidarTrackletCompareApp()
{
    overlap_threshold_ = 0.5;
    // green (Salem)
    matched_color_ .r = 30; matched_color_ .g = 130;
    matched_color_ .b = 76; matched_color_ .a = 1;
    // red (Monza)
    missed_color_ .r = 207; missed_color_ .g = 0;
    missed_color_ .b = 15; missed_color_ .a = 1;
    // purple (Seance)
    mispredicted_color_ .r = 154; mispredicted_color_ .g = 18;
    mispredicted_color_ .b = 179; mispredicted_color_ .a = 1;
}
