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
 * lidar_tracklet_compare_node.h
 *
 *  Created on: July, 05th, 2018
 */

#ifndef PROJECT_RANGE_VISION_FUSION_H
#define PROJECT_RANGE_VISION_FUSION_H

#define __APP_NAME__ "lidar_tracklet_compare"

#include <string>
#include <vector>
#include <queue>
#include <unordered_map>
#include <chrono>
#include <mutex>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Point.h>

#include <jsk_recognition_utils/geo/cube.h>
#include <jsk_recognition_utils/geo/convex_polygon.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "autoware_msgs/DetectedObjectArray.h"

#define arr_size 50

class ROSLidarTrackletCompareApp
{
  ros::NodeHandle node_handle_;
  ros::Publisher matched_objects_publisher_, missed_objects_publisher_,
    mispredicted_objects_publisher_;
  ros::Subscriber tracklets_subscriber_, detections_subscriber_;
  message_filters::Subscriber<autoware_msgs::DetectedObjectArray>
    *tracklets_filter_subscriber_, *detections_filter_subscriber_;

  std_msgs::ColorRGBA matched_color_, missed_color_, mispredicted_color_;

  std::queue<autoware_msgs::DetectedObjectArray::ConstPtr>
    detections_queue_, tracklets_queue_;

  std::mutex manual_sync_mutex_;
  double overlap_threshold_;

  std::vector<std::pair<unsigned,unsigned>> detected_car_stats_per_frame_;
  std::array<unsigned,arr_size> iou_histogram_of_detected_ = {0};
  std::array<unsigned,arr_size> distance_histogram_of_detected_ = {0};
  std::array<unsigned,arr_size> distance_histogram_of_missed_ = {0};

  typedef
  message_filters::sync_policies::ApproximateTime<autoware_msgs::DetectedObjectArray,
    autoware_msgs::DetectedObjectArray> SyncPolicyT;

  message_filters::Synchronizer<SyncPolicyT>
    *detection_tracklet_synchronizer_;

  void DetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_detections_msg);
  void TrackletsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_tracklets_msg);

  void SyncedDetectionsTrackletsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_detections_msg,
                                const autoware_msgs::DetectedObjectArray::ConstPtr &in_tracklets_msg);

  void
  MatchDetectionsAndTracklets(const autoware_msgs::DetectedObjectArray::ConstPtr &in_detections_msg,
                              const autoware_msgs::DetectedObjectArray::ConstPtr &in_tracklets_msg,
                              autoware_msgs::DetectedObjectArray &out_matched_objects,
                              autoware_msgs::DetectedObjectArray &out_missed_objects,
                              autoware_msgs::DetectedObjectArray &out_mispredicted_objects);

  double Get2DIoUOfObjects(
              const autoware_msgs::DetectedObject &in_object1,
              const autoware_msgs::DetectedObject &in_object2);

  void GetIntersectionPoints(
    jsk_recognition_utils::Vertices &out_intersect_points,
    const jsk_recognition_utils::Vertices &rect1_v,
    const jsk_recognition_utils::Vertices &rect2_v
  );

  void GetEncapsulatedPoints(
    std::list<jsk_recognition_utils::Vertices> &out_encapsulated_rect2_v_l,
    const jsk_recognition_utils::Vertices &rect1_v,
    const jsk_recognition_utils::Vertices &rect2_v,
    const double rect1_area=0
  );

  double polygonArea(const jsk_recognition_utils::Vertices &vertices);

  void populateVertices(
          const jsk_recognition_utils::Vertices &vertices,
          const float distBetweenNeighboorPointsInMeters,
          jsk_recognition_utils::Vertices &out_populated_vertices);


  inline bool is2DPointInsideRectangle(
          const jsk_recognition_utils::Point &p,
          const jsk_recognition_utils::Vertices &vertices,
          const double rect_area=0);

  jsk_recognition_utils::Vertices getVerticesOf3DObject(
          const autoware_msgs::DetectedObject &in_object);

  double Get2DDistanceBetweenObjects(const autoware_msgs::DetectedObject &in_object1,
                                   const autoware_msgs::DetectedObject &in_object2);

  /*!
   * Reads the config params from the command line
   * @param in_private_handle
   */
  void InitializeROSIo(ros::NodeHandle &in_private_handle);

public:
  void Run();

  ROSLidarTrackletCompareApp();
};


#endif //PROJECT_RANGE_VISION_FUSION_H
