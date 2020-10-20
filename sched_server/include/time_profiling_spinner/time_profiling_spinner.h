// header
//

#ifndef SCHED_SERVICE_H
#define SCHED_SERVICE_H

// How to config schedule of a node:
// add: 
/*
#include "sched_server/sched_client.hpp"
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
  SchedClient::ConfigureSchedOfCallingThread();
  TimeProfilingSpinner spinner(DEFAULT_CALLBACK_FREQ_HZ,
  DEFAULT_EXEC_TIME_MINUTES);
  spinner.spinAndProfileUntilShutdown();
  spinner.saveProfilingData();
*/


#include<list>

namespace std {
class mutex;
}

#define DEFAULT_CALLBACK_FREQ_HZ 10
#define DEFAULT_EXEC_TIME_MINUTES 5

class TimeProfilingSpinner
{
public:
    TimeProfilingSpinner(double callbackCheckFrequency,
                         int execLifetimeMinutes,
                         std::string fname_post = "");

    void measureStartTime();

    void measureAndSaveEndTime(int num_callbacks_called = -1);

    void spinAndProfileUntilShutdown();

    void saveProfilingData();

    // Most likely, there will be only one object.
    // This function is added to the class due to ros::shutdown problem.
    static void saveProfilingDataOfAllCreatedObjects();

    static void signalHandler(int sig);

    ~TimeProfilingSpinner();

private:
    long cpu_time1_;
    double callbackCheckFrequency_;
    int bufIndex_, bufSize_;
    char *callback_called_buf_;
    long *m_time_start_buf_, *m_time_end_buf_, *t_cpu_time_diff_buf_;
    bool flipped_, file_saved_;
    std::string fname_post_;
    static std::list<TimeProfilingSpinner*> createdObjects;
    static std::mutex cObjsMtx, writeMtx;

//    inline void get_thread_cputime(double& seconds);
    inline void get_thread_cputime(long& microseconds);

//    inline void get_monotonic_time(double& seconds);
    inline void get_monotonic_time(long& microseconds);

};


#endif //SCHED_SERVICE_H
