#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <math.h>

class c3_algorithm
{
private:
    //vel
    double pos, vel, acc, jeck;
    double t_pos, t_vel, t_acc;
    double max_vel, min_vel, max_acc, min_acc;
    double ek, ek1, ek2;
    double e1_min, e1_max, e2_min, e2_max;
    double qk2, qk1, qk, uk;

    double sign(double x);
    double power(double x, int n);
    double cv_uvv(double v);
    double cv_ucv(double v);
    double cv_ua(double v);
    double cv_uv(double v);
    void c3_function();

public:
    int frequency;
    double last_pos, last_vel, last_acc;
    c3_algorithm();
    ~c3_algorithm();
    void get_qk(double l_acc, double l_vel, double l_pos, double &acc, double &vel, double &pos);
    void start(double c_pos, double target_pos, double c_vel, double target_vel, double c_acc, double target_acc);
    void init(double min_vel_, double max_vel_, double min_acc_, double max_acc_, double jeck_, int frequency_);
    void prepare();
};