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

    unsigned detected_cars_in_this_frame=0;

    for (auto obj : in_detections_msg->objects){
        unsigned i;
        for(i=0; i<tracklets.size(); ++i ){
           if(is_tracklet_detected[i])
              continue;

           auto tracklet = tracklets[i];

           double d = Get2DDistanceBetweenObjects(obj, tracklet);
           if(d > 2){ // skip this obj
               continue;
           }

            double iou = Get2DIoUOfObjects(obj, tracklet);
            if(iou > overlap_threshold_){
                obj.color = matched_color_;
                out_matched_objects.objects.push_back(obj);
                is_tracklet_detected[i] = true;

                if(tracklet.label == "Car"){
                  ++detected_cars_in_this_frame;
                  double n = std::sqrt(std::pow(obj.pose.position.x,2)
                    +std::pow(obj.pose.position.y,2));
                  distance_histogram_of_detected_[(int)n/(100/arr_size)]++;
                  iou_histogram_of_detected_[int(100*iou)/(100/arr_size)]++;
                }

                break;
            }

        }

        if(!is_tracklet_detected[i]){
            obj.color = mispredicted_color_;
            out_mispredicted_objects.objects.push_back(obj);
        }
    }

    // If there are tracklets that were not matched with anything,
    // they are missed
    unsigned total_cars_in_this_frame=0;
    for(auto i=0; i < is_tracklet_detected.size(); ++i ){
        if(tracklets[i].label == "Car"){
          ++total_cars_in_this_frame;
        }

        if(!is_tracklet_detected[i]){
            auto obj = tracklets[i];
            obj.color = missed_color_;
            out_missed_objects.objects.push_back(obj);
            double n = std::sqrt(std::pow(obj.pose.position.x,2)
              +std::pow(obj.pose.position.y,2));
            distance_histogram_of_missed_[(int)n/(100/arr_size)]++;
        }

    }

    detected_car_stats_per_frame_.emplace_back(
      detected_cars_in_this_frame, total_cars_in_this_frame);

    std::cout << "IoU histogram (%0 to %99):\n";
    for(auto i=0; i<arr_size ; ++i)
      std::cout << std::setw(3) << i*(100/arr_size);
    std::cout << std::endl;
    for(auto i=0; i<arr_size ; ++i)
      std::cout << std::setw(3) << iou_histogram_of_detected_[i];
    std::cout << std::endl;

    std::cout << "Detected object distance histogram (meters):\n";
    for(auto i=0; i<arr_size ; ++i)
      std::cout << std::setw(3) << i*(100/arr_size);
    std::cout << std::endl;
    for(auto i=0; i<arr_size ; ++i)
      std::cout << std::setw(3) << distance_histogram_of_detected_[i];
    std::cout << std::endl;

    std::cout << "Missed object distance histogram (meters):\n";
    for(auto i=0; i<arr_size ; ++i)
      std::cout << std::setw(3) << i*(100/arr_size);
    std::cout << std::endl;
    for(auto i=0; i<arr_size ; ++i)
      std::cout << std::setw(3) << distance_histogram_of_missed_[i];
    std::cout << std::endl;

    std::cout << "Total detected car \n";


    std::cout << "Detected car stats:\n";
    // for(auto i=0; i<detected_car_stats_per_frame_.size() ; ++i)
      // std::cout << std::setw(3) << i;
    // std::cout << std::endl;
    auto t=0;
    for(auto i=0; i<detected_car_stats_per_frame_.size() ; ++i){
      // std::cout << std::setw(3) << detected_car_stats_per_frame_[i].second;
      t += detected_car_stats_per_frame_[i].second;
    }
    std::cout << std::endl;
    auto td=0;
    for(auto i=0; i<detected_car_stats_per_frame_.size() ; ++i){
      // std::cout << std::setw(3) << detected_car_stats_per_frame_[i].first;
      td += detected_car_stats_per_frame_[i].first;
    }
    std::cout << "Total detected:" << td << "/" << t;
    std::cout << std::endl;
}

double ROSLidarTrackletCompareApp::Get2DIoUOfObjects(
        const autoware_msgs::DetectedObject &in_object1,
        const autoware_msgs::DetectedObject &in_object2)
{
    // the points are sorted as:
    // 1 1, -1 1, -1 -1, 1 -1
    auto obj1_vertices = getVerticesOf3DObject(in_object1);
    // Just XY dimension is enough, remove half of the points in Z
    obj1_vertices.erase(obj1_vertices.begin(), obj1_vertices.begin()+4);
    for(auto &v : obj1_vertices) v[2] = 0;
    auto obj2_vertices = getVerticesOf3DObject(in_object2);
    // Just XY dimension is enough, remove half of the points in Z
    obj2_vertices.erase(obj2_vertices.begin(), obj2_vertices.begin()+4);
    for(auto &v : obj2_vertices) v[2] = 0;

    jsk_recognition_utils::Vertices intersect_points;

    GetIntersectionPoints(intersect_points, obj1_vertices, obj2_vertices);
    if(intersect_points.size() < 4) return 0;

    // calculate area and IoU
    auto intersect_area = polygonArea(intersect_points);

    auto rect1_area = polygonArea(obj1_vertices);
    auto rect2_area = polygonArea(obj2_vertices);
    auto sum_of_obj_areas = rect1_area + rect2_area;

    auto iou =  intersect_area / (sum_of_obj_areas - intersect_area);

    std::cout << "obj1 position:" << in_object1.pose.position;
    std::cout << "obj2 position:" << in_object2.pose.position;
    std::cout << "distance between objects:"
              << Get2DDistanceBetweenObjects(in_object1, in_object2)
              << "m\n";
    std::cout << "IoU: " << iou << std::endl << std::endl;

    return iou;
}

void ROSLidarTrackletCompareApp::GetIntersectionPoints(
  jsk_recognition_utils::Vertices &out_intersect_points,
  const jsk_recognition_utils::Vertices &rect1_v,
  const jsk_recognition_utils::Vertices &rect2_v)
{
  std::list<jsk_recognition_utils::Vertices> ip_v_l;

  GetEncapsulatedPoints(ip_v_l, rect1_v, rect2_v);
  auto sz = ip_v_l.size();
  if(sz == 0) return;

  GetEncapsulatedPoints(ip_v_l, rect2_v, rect1_v);
  if(ip_v_l.size() == sz) return;

  out_intersect_points.insert(out_intersect_points.end(),
    ip_v_l.front().cbegin(), ip_v_l.front().cend());
  ip_v_l.pop_front();

  while(!ip_v_l.empty()){
    auto &v = out_intersect_points.back();
    double d = DBL_MAX, d_temp;
    std::list<jsk_recognition_utils::Vertices>::iterator target;
    bool reverse;
    for(auto it=ip_v_l.begin(); it != ip_v_l.end() ; ++it){
      d_temp = (v-it->front()).squaredNorm();
      if(d_temp < d){
        target=it; reverse=false; d = d_temp;
      }
      d_temp = (v-it->back()).squaredNorm();
      if(d_temp < d){
        target=it; reverse=true; d = d_temp;
      }
    }

    if(reverse){
      out_intersect_points.insert(out_intersect_points.end(),
        target->crbegin(), target->crend());
    }
    else{
      out_intersect_points.insert(out_intersect_points.end(),
        target->cbegin(), target->cend());
    }

    ip_v_l.erase(target);
  }

}

void ROSLidarTrackletCompareApp::GetEncapsulatedPoints(
  std::list<jsk_recognition_utils::Vertices> &out_encapsulated_rect2_v_l,
  const jsk_recognition_utils::Vertices &rect1_v,
  const jsk_recognition_utils::Vertices &rect2_v,
  const double rect1_area)
{
  double r_area = (rect1_area == 0) ? polygonArea(rect1_v) : rect1_area;

  //populate points
  jsk_recognition_utils::Vertices populated_rect2_v;
  const auto neighboorPointsDistance = 0.1; //meters
  populateVertices(rect2_v, neighboorPointsDistance, populated_rect2_v);

  for(auto i =0 ; i < populated_rect2_v.size() ; ++i){
    out_encapsulated_rect2_v_l.emplace_back();
    auto &vertices = out_encapsulated_rect2_v_l.back();

    while(is2DPointInsideRectangle(populated_rect2_v[i], rect1_v, r_area)
        &&  i < populated_rect2_v.size())
    {
        vertices.push_back(populated_rect2_v[i]);
        ++i;
    }

    while(!is2DPointInsideRectangle(populated_rect2_v[i], rect1_v, r_area)
        && i < populated_rect2_v.size())
    {
        ++i;
    }

    if(vertices.empty())
      out_encapsulated_rect2_v_l.pop_back();
  }
}

// Shoelace formula
// thank you geeksforgeeks!
double ROSLidarTrackletCompareApp::polygonArea(
        const jsk_recognition_utils::Vertices &vertices)
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
        const jsk_recognition_utils::Vertices &vertices,
        const float distBetweenNeighboorPointsInMeters,
        jsk_recognition_utils::Vertices &out_populated_vertices)
{
    out_populated_vertices.clear();

    auto vrts(vertices);
    vrts.push_back(vrts[0]);
    for(auto i=0; i<vrts.size()-1; ++i){
        float x_diff = vrts[i+1][0] - vrts[i][0];
        float y_diff = vrts[i+1][1] - vrts[i][1];
	      auto d = std::sqrt(std::pow(x_diff,2) + std::pow(y_diff,2));
        unsigned pointsToPopulate =
                std::floor(d) / distBetweenNeighboorPointsInMeters + 1;

        float x_step = x_diff / pointsToPopulate;
        float y_step = y_diff / pointsToPopulate;
        auto step = std::sqrt(std::pow(x_step,2) + std::pow(y_step,2));

        auto& v = vrts[i];

        for(auto j = 0; j < pointsToPopulate ; ++j)
            out_populated_vertices.push_back(jsk_recognition_utils::Point(
                                                 v[0]+x_step*j,
                                                 v[1]+y_step*j, v[2]));


    }

}

// https://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
inline bool ROSLidarTrackletCompareApp::is2DPointInsideRectangle(
        const jsk_recognition_utils::Point &p,
        const jsk_recognition_utils::Vertices &vertices,
        const double rect_area)
{
    double r_area = (rect_area == 0) ? polygonArea(vertices) : rect_area;

    jsk_recognition_utils::Vertices triangle;
    triangle.push_back(p);
    triangle.push_back(vertices[0]);
    triangle.push_back(vertices[1]);
    auto area = polygonArea(triangle);
    triangle[1] = vertices[1];
    triangle[2] = vertices[2];
    area += polygonArea(triangle);
    triangle[1] = vertices[2];
    triangle[2] = vertices[3];
    area += polygonArea(triangle);
    triangle[1] = vertices[3];
    triangle[2] = vertices[0];
    area += polygonArea(triangle);

    const double epsilon = 0.0001;

    auto sub = area - r_area;
    bool b = sub < epsilon;
    return b;
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
    // green
    matched_color_ .r = 0; matched_color_ .g = 255;
    matched_color_ .b = 0; matched_color_ .a = 1;
    // red
    missed_color_ .r = 255; missed_color_ .g = 0;
    missed_color_ .b = 0;   missed_color_ .a = 1;
    // blue
    mispredicted_color_ .r = 0;   mispredicted_color_ .g = 0;
    mispredicted_color_ .b = 255; mispredicted_color_ .a = 1;
}
