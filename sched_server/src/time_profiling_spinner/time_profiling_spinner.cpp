#include <string>
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

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <rosgraph_msgs/Clock.h>

#include "time_profiling_spinner/time_profiling_spinner.h"

std::list<TimeProfilingSpinner*> TimeProfilingSpinner::createdObjects;
std::mutex TimeProfilingSpinner::cObjsMtx;
std::mutex TimeProfilingSpinner::writeMtx;
sem_t TimeProfilingSpinner::cb_checker_sem;
sem_t TimeProfilingSpinner::cb_ready_sem;
std::mutex TimeProfilingSpinner::wakeup_mtx;
std::condition_variable TimeProfilingSpinner::wakeup_cv;
std::atomic_flag TimeProfilingSpinner::flag = ATOMIC_FLAG_INIT;
volatile bool TimeProfilingSpinner::compThrReady = false;

TimeProfilingSpinner::TimeProfilingSpinner(
        double callbackCheckFrequency,
        bool useCompanionThread,
        std::function<void()> funcToCall,
        std::string fname_post)
{
    if (callbackCheckFrequency == USE_DEFAULT_CALLBACK_FREQ ||  callbackCheckFrequency <= 0)
        callbackCheckFrequency_=DEFAULT_CALLBACK_FREQ_HZ;
    else
        callbackCheckFrequency_ = callbackCheckFrequency;
    //useCompanionThread_=useCompanionThread;
    useCompanionThread_=false; //For now, disable companion thread for all tasks
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

    const std::lock_guard<std::mutex> lock(TimeProfilingSpinner::cObjsMtx);
    TimeProfilingSpinner::createdObjects.push_front(this);
}

// handle the case that this thread does not execute at all!
void* TimeProfilingSpinner::companionSpinner(void* ignored){
    while(ros::ok()){
        std::unique_lock<std::mutex> lk(wakeup_mtx);
        wakeup_cv.wait(lk, []{return compThrReady;});
        lk.unlock();
        if(!ros::ok())
            break;
        while(flag.test_and_set());
    }
    return NULL;
}

void* TimeProfilingSpinner::cbChecker(void* cb_queue){
    ros::CallbackQueue* cq = (ros::CallbackQueue*)cb_queue;

    while(ros::ok()){
        sem_wait(&TimeProfilingSpinner::cb_checker_sem);
        while(cq->empty() && ros::ok())
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        sem_post(&TimeProfilingSpinner::cb_ready_sem);
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
//    std::cerr << "Starting to wait for message from topic /period_init_time \n";
//    ROS_INFO("Starting to wait for message from topic /period_init_time ");

    auto trynum=1u;
    ros::NodeHandle nh("timing");
try_again:
    boost::shared_ptr<rosgraph_msgs::Clock const> clk_cptr;
    clk_cptr = ros::topic::waitForMessage<rosgraph_msgs::Clock>("period_init_time",nh);

    if(clk_cptr == NULL){
        //ROS_INFO("Message arrived from topic /period_init_time but it is NULL! try number: %d", trynum);
        std::cerr << "Message arrived from topic /period_init_time but it is NULL! try number:" << trynum << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ++trynum;
        goto try_again;
    }
    else{
        //ROS_INFO("Message arrived from topic /period_init_time ");
        std::cerr << "Message arrived from topic /period_init_time, message is:" << clk_cptr->clock.toSec() << std::endl;
    }

//    struct sched_param prio;
//    sched_getparam(0, &prio);
    // they can all agree on this time point
    // Tasks with higher prioriy will be activated earlier than the tasks with lower priority
    return std::chrono::system_clock::time_point(
                std::chrono::milliseconds(static_cast<unsigned long>(clk_cptr->clock.toSec()*1000 + 3000)));//; - prio.sched_priority)));

}

void TimeProfilingSpinner::spinAndProfileUntilShutdown(){
    ROS_INFO("Starting to initialize spinner.");

    if(useCompanionThread_)
        startCompanionThread();

    ros::CallbackQueue* cq = ros::getGlobalCallbackQueue();
    auto period = std::chrono::milliseconds(
                static_cast<int>(1000/callbackCheckFrequency_));

    //    startCbCheckerThread(cq);

    auto target = getInitTargetTime();
    std::this_thread::sleep_until(target);
    target += period;

    for(; ros::ok(); target += period)
    {

//        if(!funcToCall_ && cq->empty()){
//            sem_post(&TimeProfilingSpinner::cb_checker_sem);
//            sem_wait(&TimeProfilingSpinner::cb_ready_sem);
//        }

//        if(!funcToCall_){
//             we have to wait for callback
//            while(ros::ok() && cq->empty()){
//                std::this_thread::sleep_for(std::chrono::milliseconds(1));
//                if(std::chrono::system_clock::now() >= target)
//                    target += period;
//            }
//        }

        if(useCompanionThread_){
            flag.test_and_set();
            {
                std::lock_guard<std::mutex> lk(wakeup_mtx);
                compThrReady=true;
            }
            wakeup_cv.notify_one();
        }

        //auto thr = std::thread([this, cq](){
        int calls=0;
        measureStartTime();
        calls += static_cast<int>(!cq->empty());
	    cq->callAvailable();
        //do{
        //    ros::CallbackQueue::CallOneResult cor;
        //    cor = cq->callOne();
        //    if(cor == ros::CallbackQueue::CallOneResult::TryAgain)
        //        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        //    else if(cor == ros::CallbackQueue::CallOneResult::Called)
        //        ++calls;
        //}while(ros::ok() &&
        //       cor != ros::CallbackQueue::CallOneResult::Empty &&
        //       cor != ros::CallbackQueue::CallOneResult::Disabled);

        if(ros::ok() && funcToCall_){
            funcToCall_();
            ++calls;
        }

        measureAndSaveEndTime(calls);
        //    return;
        //});
        //thr.join();

        if(useCompanionThread_){
            {
                std::lock_guard<std::mutex> lk(wakeup_mtx);
                compThrReady=false;
            }
            flag.clear();
        }

        // We don't want this part to cut execution of others
        // when we are using rtg-sync
        std::this_thread::sleep_until(target);
    }

    joinThreads();

    ROS_INFO("Stoppped spinning.");
}

void TimeProfilingSpinner::startCompanionThread()
{

    pthread_attr_t comp_thr_attr;
    pthread_attr_init(&comp_thr_attr);

    int policy = sched_getscheduler(0);
    pthread_attr_setschedpolicy(&comp_thr_attr, policy);

    sched_param sp;
    sched_getparam(0, &sp);
    // This is a problem when this task is inside a virtual gang with
    // other tasks having lower priority
    // other than that, it is ok
    sp.sched_priority--; // use one less priority for the companion
    // task priorities must be assigned by as even numbers, like 2 4 6...
    pthread_attr_setschedparam(&comp_thr_attr,&sp);
    pthread_attr_setinheritsched(&comp_thr_attr, PTHREAD_EXPLICIT_SCHED);

    cpu_set_t cpuset;
    sched_getaffinity(0,sizeof(cpuset),&cpuset);
    pthread_attr_setaffinity_np(&comp_thr_attr,sizeof(cpuset),&cpuset);

    int ret = pthread_create(&comp_thr, &comp_thr_attr, TimeProfilingSpinner::companionSpinner, NULL);
    if(ret != 0){
        std::cerr << "Companion thread creation failed\n" << strerror(errno) << std ::endl;
    }
    else
        ROS_INFO("Companion thread created.");

    pthread_attr_destroy(&comp_thr_attr);

}

void TimeProfilingSpinner::startCbCheckerThread(ros::CallbackQueue* cq)
{
    sem_init(&TimeProfilingSpinner::cb_checker_sem, 0, 0);
    sem_init(&TimeProfilingSpinner::cb_ready_sem, 0, 0);

    pthread_attr_t cb_chk_thr_attr;
    pthread_attr_init(&cb_chk_thr_attr);

    int policy = sched_getscheduler(getppid());
    pthread_attr_setschedpolicy(&cb_chk_thr_attr, policy);

    sched_param sp;
    sched_getparam(getppid(), &sp);
    pthread_attr_setschedparam(&cb_chk_thr_attr,&sp);
    pthread_attr_setinheritsched(&cb_chk_thr_attr, PTHREAD_EXPLICIT_SCHED);

    cpu_set_t cpuset;
    sched_getaffinity(getppid(),sizeof(cpuset),&cpuset);
    pthread_attr_setaffinity_np(&cb_chk_thr_attr,sizeof(cpuset),&cpuset);

    int ret = pthread_create(&cb_chk_thr, &cb_chk_thr_attr, TimeProfilingSpinner::cbChecker, cq);
    if(ret != 0){
        std::cerr << "Cb checker thread creation failed\n" << strerror(errno) << std ::endl;
    }
    else
        ROS_INFO("Cb checker thread created.");

    pthread_attr_destroy(&cb_chk_thr_attr);
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

    std::ofstream outfile("./" + ros::this_node::getName()
                          + fname_post_ + "_timing.csv");

    if(flipped_){
        for(int i=bufIndex_; i<bufSize_; ++i){
            outfile << m_time_start_buf_[i] << ","
                    << m_time_end_buf_[i] << ","
                    << t_cpu_time_diff_buf_[i] << ","
                    << static_cast<long>(callback_called_buf_[i]) <<","
                    << static_cast<long>(start_cpuid_buf_[i]) <<","
                    << static_cast<long>(end_cpuid_buf_[i]) << std::endl;
        }
    }
    for(int i=0; i<bufIndex_; ++i){
        outfile << m_time_start_buf_[i] << ","
                << m_time_end_buf_[i] << ","
                << t_cpu_time_diff_buf_[i] << ","
                << static_cast<long>(callback_called_buf_[i]) <<","
                << static_cast<long>(start_cpuid_buf_[i]) <<","
                << static_cast<long>(end_cpuid_buf_[i]) << std::endl;
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

    //    sem_post(&TimeProfilingSpinner::cb_checker_sem);
    //    pthread_join(cb_chk_thr, NULL);
    //    sem_destroy(&TimeProfilingSpinner::cb_ready_sem);
    //    sem_destroy(&TimeProfilingSpinner::cb_checker_sem);
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


