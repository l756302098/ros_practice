#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <math.h>
#include <yd_obstacle_avoid/c2_algorithm.h>

using namespace std;

c2_algorithm::c2_algorithm()
{
    pos = 0;
    t_pos = 0;
    vel = 0;
    t_vel = 0;
    acc = 0;
    t_acc = 0;
    last_vel = 0;
    last_acc = 0;
}
c2_algorithm ::~c2_algorithm()
{
}

double c2_algorithm::sign(double x)
{
    if (x > 0)
    {
        return 1;
    }
    else if (x == 0)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

double c2_algorithm::power(double x, int n)
{
    double val = 1.0;
    while (n--)
        val *= x;
    return val;
}

void c2_algorithm::prepare()
{
    ek = (pos - t_pos) / jeck;
    ek1 = (vel - t_vel) / jeck;
}

double c2_algorithm::sat(double x)
{
    if (x < -1)
    {
        return -1;
    }
    else if (x >= -1 && x <= 1)
    {
        return x;
    }
    else
    {
        return 1;
    }
}

void c2_algorithm::c2_function()
{
    double ts = (double)1 / frequency;
    double zk = 1 / ts * (ek / ts + ek1 / 2);
    double zk1 = ek1 / ts;
    double m1 = 1 + 8 * abs(zk);
    double m = floor((1 + sqrt(m1)) / 2);
    double omk = zk1 + zk / m + (m - 1) / 2 * sign(zk);
    double uk1 = vel * sign(omk) + max_vel - ts * jeck;
    double uk2 = 1 + sign(uk1);
    uk = -1 * jeck * sat(omk) * uk2 / 2;
}

void c2_algorithm::get_qk(double l_vel, double l_pos, double &vel_, double &pos_)
{
    prepare();
    c2_function();
    double ts = (double)1 / frequency;
    qk1 = l_vel + ts * uk;
    qk = l_pos + ts / 2 * (qk1 + l_vel);

    vel = qk1;
    pos = qk;

    vel_ = qk1;
    pos_ = qk;
}

void c2_algorithm::start(double c_pos, double target_pos, double c_vel, double target_vel)
{
    pos = c_pos;
    t_pos = target_pos;
    vel = c_vel;
    t_vel = target_vel;
}

void c2_algorithm::init(double min_vel_, double max_vel_, double min_acc_, double max_acc_, double jeck_, int frequency_ = 100)
{
    min_vel = min_vel_;
    max_vel = max_vel_;
    min_acc = min_acc_;
    max_acc = max_acc_;
    jeck = jeck_;
    frequency = frequency_;
}
