#include <string>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <time.h>
#include <fstream>

#include <ros/ros.h>

#include "time_profiling_spinner/time_profiling_spinner.h"

TimeProfilingSpinner::TimeProfilingSpinner(double callbackCheckFrequency, int execLifetimeMinutes)
{
    if (callbackCheckFrequency_ < 0)
        callbackCheckFrequency_=10;
    else
        callbackCheckFrequency_ = callbackCheckFrequency;
    bufIndex_=0;
    bufSize_=callbackCheckFrequency_ * 60 * execLifetimeMinutes;
    m_time_start_buf_ = new double[bufSize_];
    m_time_end_buf_ = new double[bufSize_];
    t_cpu_time_diff_buf_ = new double[bufSize_];
    flipped_=false;
}

void TimeProfilingSpinner::measureStartTime(){
    get_monotonic_time(m_time_start_buf_[bufIndex_]);
    get_thread_cputime(cpu_time1_);
}

void TimeProfilingSpinner::measureAndSaveEndTime(){
    get_monotonic_time(m_time_end_buf_[bufIndex_]);
    double cpu_time2_;
    get_thread_cputime(cpu_time2_);
    t_cpu_time_diff_buf_[bufIndex_] = cpu_time2_ - cpu_time1_;

    // reset buf index if node continues to live
    if(++bufIndex_ >= bufSize_){
        bufIndex_ = 0;
        flipped_ = true;
    }
}

void TimeProfilingSpinner::spinAndProfileUntilShutdown(){
    ros::Rate r = ros::Rate(callbackCheckFrequency_);
    ROS_INFO("Starting to spin.");
    while (ros::ok())
    {
        measureStartTime();
        ros::spinOnce();
        measureAndSaveEndTime();
        r.sleep();
        ROS_INFO("Spinning...");
    }
    ROS_INFO("Stoppped spinning.");

}

void TimeProfilingSpinner::saveProfilingData(){
    //Save timing info to file
    ROS_INFO("Starting to write profiling data to file.");

    std::ofstream outfile("~/.ros" + ros::this_node::getName() + "timing.csv");

    if(flipped_){
        for(int i=bufIndex_; i<bufSize_; ++i){
            outfile << m_time_start_buf_[i] << ","
                    << m_time_end_buf_[i] << ","
                    << t_cpu_time_diff_buf_[i] << std::endl;
        }

    }
    for(int i=0; i<bufIndex_; ++i){
        outfile << m_time_start_buf_[i] << ","
                << m_time_end_buf_[i] << ","
                << t_cpu_time_diff_buf_[i] << std::endl;
    }

    ROS_INFO("Done writing profile data.");

}

TimeProfilingSpinner::~TimeProfilingSpinner(){
    delete m_time_start_buf_;
    delete m_time_end_buf_;
    delete t_cpu_time_diff_buf_;
}


inline void TimeProfilingSpinner::get_thread_cputime(double& seconds){
    struct timespec ts;

    clock_gettime(CLOCK_THREAD_CPUTIME_ID,&ts);
    seconds=static_cast<double>(ts.tv_sec) +
            static_cast<double>(ts.tv_nsec)/1000000000;
}

inline void TimeProfilingSpinner::get_monotonic_time(double& seconds){
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC,&ts);
    seconds=static_cast<double>(ts.tv_sec) +
            static_cast<double>(ts.tv_nsec)/1000000000;
}

