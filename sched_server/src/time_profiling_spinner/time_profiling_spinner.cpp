#include <string>
#include <sstream>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <time.h>
#include <stdlib.h>
#include <fstream>
#include <thread>
#include <chrono>
#include <list>
#include <mutex>
#include <atomic>
#include <malloc.h>
#include <sys/mman.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <rosgraph_msgs/Clock.h>
#include <assert.h>
#include <sched_server/sched_client.hpp>

#include "time_profiling_spinner/time_profiling_spinner.h"

std::list<TimeProfilingSpinner*> TimeProfilingSpinner::createdObjects;
std::mutex TimeProfilingSpinner::cObjsMtx;
std::mutex TimeProfilingSpinner::writeMtx;
std::mutex TimeProfilingSpinner::wakeup_mtx;
std::mutex TimeProfilingSpinner::cb_mtx;
std::condition_variable TimeProfilingSpinner::wakeup_cv;
std::condition_variable TimeProfilingSpinner::cb_cv;
std::atomic_flag TimeProfilingSpinner::flag = ATOMIC_FLAG_INIT;
volatile bool TimeProfilingSpinner::compThrReady = false;

TimeProfilingSpinner::TimeProfilingSpinner(
        OperationMode op_mode,
        double callbackCheckFrequency,
        bool useCompanionThread,
        std::function<void()> funcToCall,
        std::string fname_post)
{
    ros::NodeHandle timing_nh("timing");

    bool startSynchronized;
    bool ret = timing_nh.getParam("sync_start", startSynchronized);
    if(!ret) startSynchronized = false; // couldn't get the param
    synchronizedStart_ = startSynchronized;

    op_mode_ = op_mode;
    if(op_mode_ == OperationMode::CHAIN_HEAD){
        op_mode_ = synchronizedStart_ ?
          OperationMode::PERIODIC : OperationMode::RUN_CB_ON_ARRIVAL;
    }

    double def_cb_chk_freq;
    ret = timing_nh.getParam("default_period_hz", def_cb_chk_freq);
    if(!ret) def_cb_chk_freq = 10.0; // couldn't get the param, use 10 as default

    bool udef = (callbackCheckFrequency == USE_DEFAULT_CALLBACK_FREQ);
    callbackCheckFrequency_ = udef ? def_cb_chk_freq : callbackCheckFrequency;

    ROS_INFO("Using callback frequency: %f", callbackCheckFrequency_);

//    useCompanionThread_=useCompanionThread;
    useCompanionThread_=false;

    funcToCall_=funcToCall;
    bufIndex_=0;
    bufSize_= 1024;//callbackCheckFrequency_ * 60 * execLifetimeMinutes;
    m_time_start_buf_ = new long[bufSize_];
    m_time_end_buf_ = new long[bufSize_];
    t_cpu_time_diff_buf_ = new long[bufSize_];
    callback_called_buf_ = new char[bufSize_];
    start_cpuid_buf_ = new char[bufSize_];
    end_cpuid_buf_ = new char[bufSize_];
    flipped_=false;
    file_saved_=false;
    //    if(TimeProfilingSpinner::createdObjects == nullptr){
    //        TimeProfilingSpinner::createdObjects =
    //                new std::forward_list<TimeProfilingSpinner*>();
    //    }
    fname_post_ = fname_post;

    {
        const std::lock_guard<std::mutex> lock(TimeProfilingSpinner::cObjsMtx);
        TimeProfilingSpinner::createdObjects.push_front(this);
    }
}

// handle the case that this thread does not execute at all!
void* TimeProfilingSpinner::companionSpinner(void* ignored){
    SchedClient::ConfigureSchedOfCallingThread(); // necessary if virtual gang is used
    ROS_INFO("Scheduling of companion thread was configured.");

    while(ros::ok()){
        std::unique_lock<std::mutex> lk(wakeup_mtx);
        wakeup_cv.wait(lk, []{return compThrReady;});
        lk.unlock();
        if(!ros::ok())
            break;
        while(flag.test_and_set())
            pthread_yield();
    }
    return NULL;
}


void TimeProfilingSpinner::measureStartTime(){
    get_monotonic_time(m_time_start_buf_[bufIndex_]);
    get_thread_cputime(cpu_time1_);
    start_cpuid_buf_[bufIndex_] = (char)sched_getcpu();
}

void TimeProfilingSpinner::measureAndSaveEndTime(int num_callbacks_called){
    get_monotonic_time(m_time_end_buf_[bufIndex_]);
    long cpu_time2_;
    get_thread_cputime(cpu_time2_);
    t_cpu_time_diff_buf_[bufIndex_] = cpu_time2_ - cpu_time1_;
    callback_called_buf_[bufIndex_] = num_callbacks_called;
    end_cpuid_buf_[bufIndex_] = (char)sched_getcpu();

    // reset buf index if node continues to live
    if(++bufIndex_ >= bufSize_){
        bufIndex_ = 0;
        flipped_ = true;
    }
}

std::chrono::system_clock::time_point TimeProfilingSpinner::getInitTargetTime()
{
    ros::NodeHandle nh("timing");
    boost::shared_ptr<rosgraph_msgs::Clock const> clk_cptr;
    clk_cptr = ros::topic::waitForMessage<rosgraph_msgs::Clock>("period_init_time",nh);
    assert(clk_cptr != NULL);

//    struct sched_param prio;
//    sched_getparam(0, &prio);
    // they can all agree on this time point
    // Tasks with higher prioriy will be activated earlier than the tasks with lower priority
    return std::chrono::system_clock::time_point(
                std::chrono::milliseconds(
                    static_cast<unsigned long>(
                        clk_cptr->clock.toSec()*1000 + 5000)));
    //; - prio.sched_priority)));

}

void TimeProfilingSpinner::spinAndProfileUntilShutdown(){
    ROS_INFO("Starting to initialize spinner.");

    SchedClient::ConfigureSchedOfCallingThread();

    int policy = sched_getscheduler(0);
    bool privileged = (policy != SCHED_OTHER);
    if(privileged){
        // real-time system memory settings
        mallopt(M_MMAP_MAX, 0);
        mallopt(M_TRIM_THRESHOLD, -1);
        mlockall(MCL_CURRENT | MCL_FUTURE);
    }
    // Only priviledged tasks can use companion thread
//    useCompanionThread_ = useCompanionThread_ && privileged;

//    sched_getparam(0, &spinner_sched_param_); // store for later use

    if(useCompanionThread_)
        startCompanionThread();

    ros::CallbackQueue* cq = ros::getGlobalCallbackQueue();

    auto period = std::chrono::milliseconds(
                static_cast<unsigned long>(1000.0/callbackCheckFrequency_));

    std::chrono::system_clock::time_point target;
    if(synchronizedStart_){
        target = getInitTargetTime();
        std::this_thread::sleep_until(target);
    }
    else{
        target = std::chrono::system_clock::now() + period;
    }

    int calls;
    // priorities should be descending through chains
    if(op_mode_ == OperationMode::RUN_CB_ON_ARRIVAL){
        ros::WallDuration cbWait = ros::WallDuration(1.0/callbackCheckFrequency_);
        while(ros::ok()){
//            bool prioDecreased=false;
//            if(privileged && cq->empty()){
                // Decrease is required to become lowest priority
                // gang task, so we don't interrupt other gangs
                // frequently while waiting for the callback to be available
                // This continues until callback is available
//                decreasePriority();
//                prioDecreased=true;
//            }

            //Wait until callback becomes available
//            auto rate = ros::Rate(200.0);
//            while(ros::ok() && cq->empty())
//                rate.sleep();

//            if(privileged && prioDecreased){
                // Retreive back the gang priority
//                regainPriority();
//            }

//            if(useCompanionThread_)
//                wakeupCompanionThread();

            measureStartTime();
            calls=callAvailableOneByOne(cq, target, cbWait);
            if(calls > 0)
                measureAndSaveEndTime(calls);

            if(target < std::chrono::system_clock::now())
                target += period;

//            if(useCompanionThread_)
//                suspendCompanionThread();

        }
    }
    else{ // periodic
        bool func_available = (funcToCall_ ? true : false);

        while(ros::ok())
        {
            if(useCompanionThread_)
                wakeupCompanionThread();

            measureStartTime();
            calls=callAvailableOneByOne(cq, target += period);
            if(ros::ok() && func_available){
                funcToCall_();
                calls+=1;
            }
            if(calls > 0)
                measureAndSaveEndTime(calls);

            if(useCompanionThread_)
                suspendCompanionThread();

            std::this_thread::sleep_until(target);
        }
    }

    joinThreads();

    ROS_INFO("Stoppped spinning.");
}

int TimeProfilingSpinner::callAvailableOneByOne(
        ros::CallbackQueue *cq,
        std::chrono::system_clock::time_point timeout,
        ros::WallDuration cbWaitTime)
{
    int calls=0;
    ros::CallbackQueue::CallOneResult cor = ros::CallbackQueue::Called;
    while(ros::ok() &&
          std::chrono::system_clock::now() < timeout &&
          cor != ros::CallbackQueue::Disabled &&
          cor != ros::CallbackQueue::Empty)
    {
        cor = cq->callOne(cbWaitTime);
        if(cor == ros::CallbackQueue::Called)
            ++calls;
    }

    return calls;
}

void TimeProfilingSpinner::startCompanionThread()
{
//    pthread_attr_t comp_thr_attr;
//    pthread_attr_init(&comp_thr_attr);

//    int policy = sched_getscheduler(0);
//    pthread_attr_setschedpolicy(&comp_thr_attr, policy);

//    sched_param sp;
//    sched_getparam(0, &sp);
    // This is a problem when this task is inside a virtual gang with
    // other tasks having lower priority
    // other than that, it is ok
//    sp.sched_priority--; // use one less priority for the companion
//    pthread_attr_setschedparam(&comp_thr_attr,&sp);
//    pthread_attr_setinheritsched(&comp_thr_attr, PTHREAD_EXPLICIT_SCHED);

//    cpu_set_t cpuset;
//    sched_getaffinity(0,sizeof(cpuset),&cpuset);
//    pthread_attr_setaffinity_np(&comp_thr_attr,sizeof(cpuset),&cpuset);

//    int ret = pthread_create(&comp_thr, &comp_thr_attr,
//                             TimeProfilingSpinner::companionSpinner, NULL);
    int ret = pthread_create(&comp_thr, NULL,
                             TimeProfilingSpinner::companionSpinner, NULL);
    pthread_yield(); // Let companion to execute until it comes to condition variable.
    if(ret != 0){
        std::cerr << "Companion thread creation failed\n" << strerror(errno) << std ::endl;
    }
    else
        ROS_INFO("Companion thread created.");

    //    pthread_attr_destroy(&comp_thr_attr);
}

void TimeProfilingSpinner::wakeupCompanionThread()
{
    flag.test_and_set();
    {
        std::lock_guard<std::mutex> lk(wakeup_mtx);
        compThrReady=true;
    }
    wakeup_cv.notify_one();
    pthread_yield();
}

void TimeProfilingSpinner::suspendCompanionThread()
{
    {
        std::lock_guard<std::mutex> lk(wakeup_mtx);
        compThrReady=false;
    }
    flag.clear();
    pthread_yield();
}

void TimeProfilingSpinner::saveProfilingData(){
    const std::lock_guard<std::mutex> lock(TimeProfilingSpinner::writeMtx);
    //Save timing info to file
    if(file_saved_)
        return;

    if(bufIndex_==0 && !flipped_){
        ROS_INFO("Not writing profile data since it is empty.");
        return;
    }

    ROS_INFO("Starting to write profiling data to file.");

    std::stringstream ss;
    auto num_lines=0;

    if(flipped_){
        for(int i=bufIndex_; i<bufSize_; ++i){
            ss << m_time_start_buf_[i] << ","
               << m_time_end_buf_[i] << ","
               << t_cpu_time_diff_buf_[i] << ","
               << static_cast<long>(callback_called_buf_[i]) <<","
               << static_cast<long>(start_cpuid_buf_[i]) <<","
               << static_cast<long>(end_cpuid_buf_[i]) << std::endl;
            ++num_lines;
        }
    }
    
    for(int i=0; i<bufIndex_; ++i){
        ss << m_time_start_buf_[i] << ","
           << m_time_end_buf_[i] << ","
           << t_cpu_time_diff_buf_[i] << ","
           << static_cast<long>(callback_called_buf_[i]) <<","
           << static_cast<long>(start_cpuid_buf_[i]) <<","
           << static_cast<long>(end_cpuid_buf_[i]) << std::endl;
        ++num_lines;
    }

    std::ofstream outfile("./" + ros::this_node::getName()
                          + fname_post_ + "_timing.csv");
    
    // Skip first 32 and last 16 samples
    auto firstToSkip=32, lastToSkip=16;
    // read from string stream and dump to file
    std::string line;
    for(auto l=0; l<num_lines-lastToSkip; ++l){
        std::getline(ss, line);
        if(l > firstToSkip)
            outfile << line << std::endl;
    }

    file_saved_=true;
    ROS_INFO("Done writing profile data.");
}

void TimeProfilingSpinner::joinThreads()
{
    if(useCompanionThread_){
        {
            std::lock_guard<std::mutex> lk(wakeup_mtx);
            compThrReady=true;
        }
        flag.clear();
        wakeup_cv.notify_one();
        pthread_join(comp_thr, NULL);
    }

}

void TimeProfilingSpinner::decreasePriority()
{
    static const sched_param lowest_rt_prio_sp = {
        .sched_priority=1
    };
    if(sched_setparam(0, &lowest_rt_prio_sp) < 0)
        perror("decreasePriority, sched_setparam error");
}

void TimeProfilingSpinner::regainPriority()
{
    if(sched_setparam(0, &spinner_sched_param_) < 0)
        perror("regainPriority, sched_setparam error");
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
    delete start_cpuid_buf_;
    delete end_cpuid_buf_;
}


//inline void TimeProfilingSpinner::get_thread_cputime(double& seconds){
inline void TimeProfilingSpinner::get_thread_cputime(long& microseconds){
    struct timespec ts;

    clock_gettime(CLOCK_THREAD_CPUTIME_ID,&ts);
    microseconds = ts.tv_sec*1000000 + ts.tv_nsec/1000;
}

//inline void TimeProfilingSpinner::get_monotonic_time(double& seconds){
inline void TimeProfilingSpinner::get_monotonic_time(long& microseconds){

    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC,&ts);
    microseconds = ts.tv_sec*1000000 + ts.tv_nsec/1000;
}


