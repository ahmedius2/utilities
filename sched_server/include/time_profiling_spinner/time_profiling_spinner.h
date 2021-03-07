// header
//

#ifndef SCHED_SERVICE_H
#define SCHED_SERVICE_H

// How to config schedule of a node:
// add: 
/*
//#include "sched_server/sched_client.hpp"
#include "sched_server/time_profiling_spinner.h"
*/

// How to profile a node:

// Also in the cmake, add sched_server to find_package(catkin ...
// And in the package.xml, add: 
/*
  <depend>sched_server</depend>
*/

// If the node has ros::shutdown problem, install a new singint handler at where
/*
#include "sched_server/time_profiling_spinner.h"
#include <signal.h>

  ros::init is called: like this:
  ros::init(argc, argv, "node_name", ros::init_options::NoSigintHandler);
  signal(SIGINT, TimeProfilingSpinner::signalHandler);
*/
// After that, replace ros::spin() with this code snippet:
/*
  //SchedClient::ConfigureSchedOfCallingThread();
  TimeProfilingSpinner spinner(DEFAULT_CALLBACK_FREQ_HZ,false, func);
  spinner.spinAndProfileUntilShutdown();
  spinner.saveProfilingData();
*/


#include <list>
#include <chrono>
#include <functional>
#include <atomic>
#include <condition_variable>
#include <semaphore.h>

namespace std {
class mutex;
}

namespace ros {
class CallbackQueue;
class WallDuration;
}

#define USE_DEFAULT_CALLBACK_FREQ 0
//#define DEFAULT_EXEC_TIME_MINUTES 2

class TimeProfilingSpinner
{
public:
    enum class OperationMode {
        CHAIN_HEAD, // periodic if synchronized start else arrival otherwise
        RUN_CB_ON_ARRIVAL,
        PERIODIC
    };

    TimeProfilingSpinner(OperationMode op_mode,
        double callbackCheckFrequency = USE_DEFAULT_CALLBACK_FREQ,
        bool useCompanionThread = false,
        std::function<void()> funcToCall = std::function<void()>(),
        std::string fname_post = "");

    void measureStartTime();

    void measureAndSaveEndTime(int num_callbacks_called = -1);

    std::chrono::system_clock::time_point getInitTargetTime();

    void spinAndProfileUntilShutdown();

    int callAvailableOneByOne(
            ros::CallbackQueue* cq,
            std::chrono::system_clock::time_point timeout,
            ros::WallDuration cbWaitTime = ros::WallDuration());

    void startCompanionThread();

    void wakeupCompanionThread();

    void suspendCompanionThread();

    void saveProfilingData();

    void joinThreads();

    void decreasePriority();

    void regainPriority();

    // Most likely, there will be only one object.
    // This function is added to the class due to ros::shutdown problem.
    static void saveProfilingDataOfAllCreatedObjects();

    static void signalHandler(int sig);

    static void* companionSpinner(void *ignored);

    ~TimeProfilingSpinner();

private:
    OperationMode op_mode_;
    std::function<void()> funcToCall_;
    long cpu_time1_;
    double callbackCheckFrequency_;
    int bufIndex_, bufSize_;
    char *callback_called_buf_, *start_cpuid_buf_, *end_cpuid_buf_;
    long *m_time_start_buf_, *m_time_end_buf_, *t_cpu_time_diff_buf_;
    bool flipped_, file_saved_;
    std::string fname_post_;
    static std::list<TimeProfilingSpinner*> createdObjects;
    static std::mutex cObjsMtx, writeMtx;

    pthread_t comp_thr, cb_chk_thr;
//    static sem_t cb_checker_sem, cb_ready_sem;
    static std::mutex wakeup_mtx, cb_mtx;
    static std::condition_variable wakeup_cv, cb_cv;
    static std::atomic_flag flag;
    static volatile bool compThrReady;
    bool synchronizedStart_;
    bool useCompanionThread_;

    sched_param spinner_sched_param_;

//    inline void get_thread_cputime(double& seconds);
    inline void get_thread_cputime(long& microseconds);

//    inline void get_monotonic_time(double& seconds);
    inline void get_monotonic_time(long& microseconds);

};


#endif //SCHED_SERVICE_H
