// header
//

#ifndef SCHED_SERVICE_H
#define SCHED_SERVICE_H


class TimeProfilingSpinner
{
public:
    TimeProfilingSpinner(double callbackCheckFrequency, int execLifetimeMinutes);

    void measureStartTime();

    void measureAndSaveEndTime();

    void spinAndProfileUntilShutdown();

    void saveProfilingData();

    ~TimeProfilingSpinner();

private:
    double cpu_time1_;
    double callbackCheckFrequency_;
    int bufIndex_, bufSize_;
    double *m_time_start_buf_, *m_time_end_buf_, *t_cpu_time_diff_buf_;
    bool flipped_;

    inline void get_thread_cputime(double& seconds);

    inline void get_monotonic_time(double& seconds);
};


#endif //SCHED_SERVICE_H
