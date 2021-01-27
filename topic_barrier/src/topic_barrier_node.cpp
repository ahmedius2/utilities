#include "topic_barrier/topic_barrier.h"
#include <thread>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/UInt64.h>
#include <ros/init.h>
#include <ros/topic.h>
#include <assert.h>     /* assert */

TopicBarrier::TopicBarrier()
{
    {
        ros::NodeHandle private_nh("~");
        bool relay;
        private_nh.param<bool>("relay", relay, false);
        bool data_loopback;
        private_nh.param<bool>("data_loopback", data_loopback, false);

        lb_relay_mode_ = data_loopback && relay;
        if(lb_relay_mode_)
            ROS_INFO("Topic barrier is going to run in lb_relay_mode");
        lb_barrier_mode_ = data_loopback && !relay;
        if(lb_barrier_mode_)
            ROS_INFO("Topic barrier is going to run in lb_barrier_mode");
        relay_mode_ = !data_loopback && relay;
        if(relay_mode_)
            ROS_INFO("Topic barrier is going to run in relay_mode");
        barrier_mode_ = !data_loopback && !relay;
        if(barrier_mode_)
            ROS_INFO("Topic barrier is going to run in barrier_mode");
    }

    subscriptions_.push_back(nh_.subscribe("/kitti/velo/pointcloud", 10,
                   &TopicBarrier::pc2_callback, this, ros::TransportHints().tcpNoDelay()));
    subscriptions_.push_back(nh_.subscribe("/kitti/oxts/imu", 10,
                   &TopicBarrier::imu_callback, this, ros::TransportHints().tcpNoDelay()));
    subscriptions_.push_back(nh_.subscribe("/kitti/oxts/gps/fix", 10,
                   &TopicBarrier::fix_callback, this, ros::TransportHints().tcpNoDelay()));
    subscriptions_.push_back(nh_.subscribe("/kitti/oxts/gps/vel", 10,
                   &TopicBarrier::twist_callback, this, ros::TransportHints().tcpNoDelay()));
    subscriptions_.push_back(nh_.subscribe("/kitti/camera_color_right/image_raw", 10,
                   &TopicBarrier::img_callback, this, ros::TransportHints().tcpNoDelay()));
    subscriptions_.push_back(nh_.subscribe("/kitti/camera_color_right/camera_info", 10,
                   &TopicBarrier::caminfo_callback, this, ros::TransportHints().tcpNoDelay()));

    pc2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_raw", 1);
    img_pub_ = nh_.advertise<sensor_msgs::Image>("/image_raw", 1);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu_raw", 1);
    fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/fix", 1);
    caminfo_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera_info", 1);
    ts_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/vel", 1);
}

void TopicBarrier::run()
{
    if(relay_mode_ || lb_relay_mode_){
        ros::spin();
    }
    else{
        rosgraph_msgs::ClockConstPtr clk_cptr;
        {
            ros::NodeHandle timing_nh("timing");
            bool ret = timing_nh.getParam("default_period_hz", publish_freq_hz_);
            assert(ret);
            clk_cptr = ros::topic::waitForMessage<rosgraph_msgs::Clock>(
                        "period_init_time",timing_nh);
            assert(clk_cptr != NULL);
        }

        auto target = std::chrono::system_clock::time_point(
          std::chrono::milliseconds(static_cast<unsigned long>(
          clk_cptr->clock.toSec()*1000 + 5000 - (1000.0/publish_freq_hz_)/2)));
        auto period = std::chrono::milliseconds(
                    static_cast<unsigned long>(1000/publish_freq_hz_));

        std::this_thread::sleep_until(target);

        if(lb_barrier_mode_)
            run_lb_barrier_mode(target, period);
        else
            run_barrier_mode(target, period);
    }

}

void TopicBarrier::run_barrier_mode(std::chrono::system_clock::time_point target,
                                    std::chrono::milliseconds period)
{
    unsigned publish_round = rosbag_freq / publish_freq_hz_, cur_round=1;

    while(ros::ok()){
        ros::spinOnce();

        // this will ensure correct data is published depending on frequency
        while(cur_round < publish_round && all_queues_non_empty()){
            pc2_queue_.pop(); img_queue_.pop(); imu_queue_.pop();
            fix_queue_.pop(); caminfo_queue_.pop(); ts_queue_.pop();
            ++cur_round;
        }

        if(cur_round == publish_round && all_queues_non_empty()){
            ts_pub_.publish(ts_queue_.front()); ts_queue_.pop();
            pc2_pub_.publish(pc2_queue_.front()); pc2_queue_.pop();
            img_pub_.publish(img_queue_.front()); img_queue_.pop();
            imu_pub_.publish(imu_queue_.front()); imu_queue_.pop();
            fix_pub_.publish(fix_queue_.front()); fix_queue_.pop();
            caminfo_pub_.publish(caminfo_queue_.front()); caminfo_queue_.pop();
            cur_round=1;
        }

        std::this_thread::sleep_until(target += period);
    }

}

void TopicBarrier::run_lb_barrier_mode(std::chrono::system_clock::time_point target,
                                             std::chrono::milliseconds period)
{
    while(ros::ok()){
        ros::spinOnce();

        if(all_queues_non_empty())
            break;

        std::this_thread::sleep_until(target += period);
    }

    // This loop will continue even after rosbag player finishes
    while(ros::ok()){
        ts_pub_.publish(ts_queue_.front());
        pc2_pub_.publish(pc2_queue_.front());
        img_pub_.publish(img_queue_.front());
        imu_pub_.publish(imu_queue_.front());
        fix_pub_.publish(fix_queue_.front());
        caminfo_pub_.publish(caminfo_queue_.front());

        std::this_thread::sleep_until(target += period);
    }
}

bool TopicBarrier::all_queues_non_empty(){
    return !(pc2_queue_.empty()   || img_queue_.empty() ||
           imu_queue_.empty()     || fix_queue_.empty() ||
           caminfo_queue_.empty() || ts_queue_.empty());
}

void TopicBarrier::pc2_callback(const sensor_msgs::PointCloud2ConstPtr &pc2_ptr)
{
    if(relay_mode_){
        pc2_pub_.publish(pc2_ptr);
    }
    else if(lb_relay_mode_){
        if(pc2_queue_.empty())
            pc2_queue_.push(pc2_ptr);
        pc2_pub_.publish(pc2_queue_.front());
    }
    else if(barrier_mode_){
        pc2_queue_.push(pc2_ptr);
    }
    else{ // lb_barrier_mode_
        if(pc2_queue_.empty())
            pc2_queue_.push(pc2_ptr);
    }
}

void TopicBarrier::img_callback(const sensor_msgs::ImageConstPtr &img_ptr)
{
    if(relay_mode_){
        img_pub_.publish(img_ptr);
    }
    else if(lb_relay_mode_){
        if(img_queue_.empty())
            img_queue_.push(img_ptr);
        img_pub_.publish(img_queue_.front());
    }
    else if(barrier_mode_){
        img_queue_.push(img_ptr);
    }
    else{ // lb_barrier_mode_
        if(img_queue_.empty())
            img_queue_.push(img_ptr);
    }
}

void TopicBarrier::imu_callback(const sensor_msgs::ImuConstPtr &imu_ptr)
{    
    if(relay_mode_){
        imu_pub_.publish(imu_ptr);
    }
    else if(lb_relay_mode_){
        if(imu_queue_.empty())
            imu_queue_.push(imu_ptr);
        imu_pub_.publish(imu_queue_.front());
    }
    else if(barrier_mode_){
        imu_queue_.push(imu_ptr);
    }
    else{ // lb_barrier_mode_
        if(imu_queue_.empty())
            imu_queue_.push(imu_ptr);
    }
}

void TopicBarrier::fix_callback(const sensor_msgs::NavSatFixConstPtr &fix_ptr)
{    
    if(relay_mode_){
        fix_pub_.publish(fix_ptr);
    }
    else if(lb_relay_mode_){
        if(fix_queue_.empty())
            fix_queue_.push(fix_ptr);
        fix_pub_.publish(fix_queue_.front());
    }
    else if(barrier_mode_){
        fix_queue_.push(fix_ptr);
    }
    else{ // lb_barrier_mode_
        if(fix_queue_.empty())
            fix_queue_.push(fix_ptr);
    }
}

void TopicBarrier::caminfo_callback(const sensor_msgs::CameraInfoConstPtr &caminfo_ptr)
{
    if(relay_mode_){
        caminfo_pub_.publish(caminfo_ptr);
    }
    else if(lb_relay_mode_){
        if(caminfo_queue_.empty())
            caminfo_queue_.push(caminfo_ptr);
        caminfo_pub_.publish(caminfo_queue_.front());
    }
    else if(barrier_mode_){
        caminfo_queue_.push(caminfo_ptr);
    }
    else{ // lb_barrier_mode_
        if(caminfo_queue_.empty())
            caminfo_queue_.push(caminfo_ptr);
    }
}

void TopicBarrier::twist_callback(const geometry_msgs::TwistStampedConstPtr &ts_ptr)
{
    if(relay_mode_){
        ts_pub_.publish(ts_ptr);
    }
    else if(lb_relay_mode_){
        if(ts_queue_.empty())
            ts_queue_.push(ts_ptr);
        ts_pub_.publish(ts_queue_.front());
    }
    else if(barrier_mode_){
        ts_queue_.push(ts_ptr);
    }
    else{ // lb_barrier_mode_
        if(ts_queue_.empty())
            ts_queue_.push(ts_ptr);
    }
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "topic_barrier");
    TopicBarrier tb;
    tb.run();
	return 0;
}
