#ifndef TOPIC_BARRIER_H
#define TOPIC_BARRIER_H

#include <queue>
#include <mutex>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/node_handle.h>

class TopicBarrier
{
public:
    TopicBarrier();

    void run();
    void run_barrier_mode(std::chrono::system_clock::time_point target,
                          std::chrono::milliseconds period);
    void run_lb_barrier_mode(std::chrono::system_clock::time_point target,
                                   std::chrono::milliseconds period);

    const double rosbag_freq=10.0;
private:
    ros::NodeHandle nh_;
//    std::mutex data_lock_;
    double publish_freq_hz_;
    // Modes:
    // 1- lb + relay
    // 2- lb + barrier
    // 3- relay
    // 4- barrier
    bool lb_relay_mode_, lb_barrier_mode_, relay_mode_, barrier_mode_;
    std::vector<ros::Subscriber> subscriptions_;

    ros::Publisher pc2_pub_, img_pub_, imu_pub_, fix_pub_, caminfo_pub_, ts_pub_;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pc2_queue_;
    std::queue<sensor_msgs::ImageConstPtr> img_queue_;
    std::queue<sensor_msgs::ImuConstPtr> imu_queue_;
    std::queue<sensor_msgs::NavSatFixConstPtr> fix_queue_;
    std::queue<sensor_msgs::CameraInfoConstPtr> caminfo_queue_;
    std::queue<geometry_msgs::TwistStampedConstPtr> ts_queue_;

    bool all_queues_non_empty();

    void pc2_callback(const sensor_msgs::PointCloud2ConstPtr& pc2_ptr);
    void img_callback(const sensor_msgs::ImageConstPtr& img_ptr);
    void imu_callback(const sensor_msgs::ImuConstPtr& imu_ptr);
    void fix_callback(const sensor_msgs::NavSatFixConstPtr& fix_ptr);
    void caminfo_callback(const sensor_msgs::CameraInfoConstPtr& caminfo_ptr);
    void twist_callback(const geometry_msgs::TwistStampedConstPtr& ts_ptr);
};

#endif
