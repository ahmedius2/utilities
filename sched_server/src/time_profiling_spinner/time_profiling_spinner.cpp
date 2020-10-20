#include <string>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <time.h>
#include <fstream>
#include <thread>
#include <list>
#include <mutex>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "time_profiling_spinner/time_profiling_spinner.h"

std::list<TimeProfilingSpinner*> TimeProfilingSpinner::createdObjects;
std::mutex TimeProfilingSpinner::cObjsMtx;
std::mutex TimeProfilingSpinner::writeMtx;

TimeProfilingSpinner::TimeProfilingSpinner(
        double callbackCheckFrequency, int execLifetimeMinutes,
        std::string fname_post)
{
    if (callbackCheckFrequency_ < 0)
        callbackCheckFrequency_=10;
    else
        callbackCheckFrequency_ = callbackCheckFrequency;
    bufIndex_=0;
    bufSize_=callbackCheckFrequency_ * 60 * execLifetimeMinutes;
    m_time_start_buf_ = new long[bufSize_];
    m_time_end_buf_ = new long[bufSize_];
    t_cpu_time_diff_buf_ = new long[bufSize_];
    callback_called_buf_ = new char[bufSize_];
    flipped_=false;
    file_saved_=false;
//    if(TimeProfilingSpinner::createdObjects == nullptr){
//        TimeProfilingSpinner::createdObjects =
//                new std::forward_list<TimeProfilingSpinner*>();
//    }
    fname_post_ = fname_post;

    const std::lock_guard<std::mutex> lock(TimeProfilingSpinner::cObjsMtx);
    TimeProfilingSpinner::createdObjects.push_front(this);
}

void TimeProfilingSpinner::measureStartTime(){
    get_monotonic_time(m_time_start_buf_[bufIndex_]);
    get_thread_cputime(cpu_time1_);
}

void TimeProfilingSpinner::measureAndSaveEndTime(int num_callbacks_called){
    get_monotonic_time(m_time_end_buf_[bufIndex_]);
    long cpu_time2_;
    get_thread_cputime(cpu_time2_);
    t_cpu_time_diff_buf_[bufIndex_] = cpu_time2_ - cpu_time1_;
    callback_called_buf_[bufIndex_] = num_callbacks_called;

    // reset buf index if node continues to live
    if(++bufIndex_ >= bufSize_){
        bufIndex_ = 0;
        flipped_ = true;
    }
}

void TimeProfilingSpinner::spinAndProfileUntilShutdown(){
    ros::Rate r = ros::Rate(callbackCheckFrequency_);
    ROS_INFO("Starting to spin.");
    ros::CallbackQueue* cq = ros::getGlobalCallbackQueue();
    while (ros::ok())
    {
        measureStartTime();
        int num_called_callbacks=0;
        ros::CallbackQueue::CallOneResult cor;
        while(!cq->empty()){
            cor = cq->callOne();
            if(cor == ros::CallbackQueue::CallOneResult::Called)
                ++num_called_callbacks;
            else if(cor == ros::CallbackQueue::CallOneResult::TryAgain)
                ROS_INFO("Couldn't call callback, gonna try again...");
            else
                break; // disabled or empty
        }
        measureAndSaveEndTime(num_called_callbacks);
        r.sleep();
    }
    ROS_INFO("Stoppped spinning.");

}

void TimeProfilingSpinner::saveProfilingData(){
    const std::lock_guard<std::mutex> lock(TimeProfilingSpinner::writeMtx);
    //Save timing info to file
    if(file_saved_)
        return;

    ROS_INFO("Starting to write profiling data to file.");

    std::ofstream outfile("./" + ros::this_node::getName()
                          + fname_post_ + "_timing.csv");

    if(flipped_){
        for(int i=bufIndex_; i<bufSize_; ++i){
            outfile << m_time_start_buf_[i] << ","
                    << m_time_end_buf_[i] << ","
                    << static_cast<long>(callback_called_buf_[i])  << ","
                    << t_cpu_time_diff_buf_[i] << std::endl;
        }
    }
    for(int i=0; i<bufIndex_; ++i){
        outfile << m_time_start_buf_[i] << ","
                << m_time_end_buf_[i] << ","
                << static_cast<long>(callback_called_buf_[i])  << ","
                << t_cpu_time_diff_buf_[i] << std::endl;
    }
    file_saved_=true;
    ROS_INFO("Done writing profile data.");
}

void TimeProfilingSpinner::saveProfilingDataOfAllCreatedObjects()
{
    {
        const std::lock_guard<std::mutex> lock(TimeProfilingSpinner::cObjsMtx);
        for(auto obj : TimeProfilingSpinner::createdObjects)
            obj->saveProfilingData();
    }

}

void TimeProfilingSpinner::signalHandler(int sig)
{
    ROS_INFO("Signal has come.");
    TimeProfilingSpinner::saveProfilingDataOfAllCreatedObjects();
    ros::shutdown();
}


TimeProfilingSpinner::~TimeProfilingSpinner(){
    if(!file_saved_){
        saveProfilingData();
    }

    delete m_time_start_buf_;
    delete m_time_end_buf_;
    delete t_cpu_time_diff_buf_;
    delete callback_called_buf_;
}


//inline void TimeProfilingSpinner::get_thread_cputime(double& seconds){
inline void TimeProfilingSpinner::get_thread_cputime(long& microseconds){
    struct timespec ts;

    clock_gettime(CLOCK_THREAD_CPUTIME_ID,&ts);
//    seconds=static_cast<double>(ts.tv_sec) +
//            static_cast<double>(ts.tv_nsec)/1000000000;
    microseconds = ts.tv_sec*1000000 + ts.tv_nsec/1000;
}

//inline void TimeProfilingSpinner::get_monotonic_time(double& seconds){
inline void TimeProfilingSpinner::get_monotonic_time(long& microseconds){

    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC,&ts);
//    seconds=static_cast<double>(ts.tv_sec) +
//            static_cast<double>(ts.tv_nsec)/1000000000;
    microseconds = ts.tv_sec*1000000 + ts.tv_nsec/1000;

}


