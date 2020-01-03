#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <math.h>

#include "c3_algorithm/c3_algorithm.h"

using namespace std;

c3_algorithm::c3_algorithm()
{
    pos = 0;
    t_pos = 0;
    vel = 0;
    t_vel = 0;
    acc = 0;
    t_acc = 0;
    last_pos = 0;
    last_vel = 0;
    last_acc = 0;
}
c3_algorithm ::~c3_algorithm()
{
}

double c3_algorithm::sign(double x)
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

double c3_algorithm::power(double x, int n)
{
    double val = 1.0;
    while (n--)
        val *= x;
    return val;
}

//δ v (v) = ë k |ë k | + 2( ė k − v)
double c3_algorithm::cv_uvv(double v)
{
    return ek2 * abs(ek2) + 2 * (ek1 - v);
}
//u cv (v) = −U sign δ v (v) + (1 − |sign(δ v (v))|)ë k
double c3_algorithm::cv_ucv(double v)
{
    double v1 = cv_uvv(v);
    double v2 = 1 - abs(sign(cv_uvv(v)));
    return -1 * jeck * sign(v1 + v2);
}
//u a (a) = −U sign(ë k − a).
double c3_algorithm::cv_ua(double v)
{
    return -1 * jeck * sign(ek2 - v);
}
//u v (v) = max u a (ë min ), min{u cv (v), u a (ë max )}
double c3_algorithm::cv_uv(double v)
{
    double v1 = cv_ua(e2_min);
    double v2 = min(cv_ucv(v), cv_ua(e2_max));
    return max(v1, v2);
}

void c3_algorithm::c3_function()
{
    double c31 = ek1 + ek2 * abs(ek2) / 2;
    double s_c31 = sign(c31);
    double p1 = 2 * power(ek2 * ek2 + 2 * ek1 * s_c31, 3);
    double c32 = ek + ek1 * ek2 * s_c31 - ek2 * ek2 * ek2 / 6 * (1 - 3 * abs(s_c31)) + s_c31 / 4 * sqrt(p1);
    double v_add = ek - e2_max * (ek2 * ek2 - 2 * ek1) / 4 - (ek2 * ek2 - 2 * ek1) * (ek2 * ek2 - 2 * ek1) / e2_max / 8 - ek2 * (3 * ek1 - ek2 * ek2) / 3;
    double v_sub = ek - e2_min * (ek2 * ek2 + 2 * ek1) / 4 - (ek2 * ek2 + 2 * ek1) * (ek2 * ek2 + 2 * ek1) / e2_min / 8 + ek2 * (3 * ek1 + ek2 * ek2) / 3;
    double c35 = 0;
    if (ek2 <= e2_max && ek1 <= (ek2 * ek2 / 2 - e2_max * e2_max))
    {
        c35 = v_add;
    }
    else if (ek2 >= e2_min && ek1 >= (e2_min * e2_min - ek2 * ek2 / 2))
    {
        c35 = v_sub;
    }
    else
    {
        c35 = c32;
    }
    double uc_1 = 1 - abs(sign(c35));
    double uc_2 = c31 + (1 - abs(sign(c31))) * ek2;
    double uc = -1 * jeck * sign(c35 + uc_1 * uc_2);
    double uk_1 = cv_uv(e1_min);
    double uk_2 = min(uc, cv_uv(e1_max));
    uk = max(uk_1, uk_2);

    // double uk = max(uk_1, uk_2);
    // return uk;
}
//q̈ k = q̈ k−1 + T s u k−1
//前一次时刻的加速度/速度/位置
void c3_algorithm::get_qk(double l_acc, double l_vel, double l_pos, double &acc_, double &vel_, double &pos_)
{
    //update uk
    prepare();
    c3_function();

    //cout << "frequency:" << frequency << " uk:" << uk << endl;
    double qk2 = l_acc + (double)1 / frequency * uk;
    double qk1 = l_vel + (double)1 / frequency / 2 * (qk2 + l_acc);
    double qk = l_pos + (double)1 / frequency / 2 * (qk1 + l_vel);
    //cout << "qk:" << qk << " qk1:" << qk1 << " qk2:" << qk2 << endl;
    //update
    pos = qk;
    vel = qk1;
    acc = qk2;

    pos_ = qk;
    vel_ = qk1;
    acc_ = qk2;
}

void c3_algorithm::start(double c_pos, double target_pos, double c_vel, double target_vel, double c_acc, double target_acc)
{
    pos = c_pos;
    t_pos = target_pos;
    vel = c_vel;
    t_vel = target_vel;
    acc = c_acc;
    t_acc = target_acc;
}

void c3_algorithm::init(double min_vel_, double max_vel_, double min_acc_, double max_acc_, double jeck_, int frequency_ = 100)
{
    min_vel = min_vel_;
    max_vel = max_vel_;
    min_acc = min_acc_;
    max_acc = max_acc_;
    jeck = jeck_;
    frequency = frequency_;
}

void c3_algorithm::prepare()
{
    //1
    ek = (pos - t_pos) / jeck;
    ek1 = (vel - t_vel) / jeck;
    ek2 = (acc - t_acc) / jeck;
    //2
    e1_min = (min_vel - t_vel) / jeck;
    e1_max = (max_vel - t_vel) / jeck;
    e2_min = (min_acc - t_acc) / jeck;
    e2_max = (max_acc - t_acc) / jeck;
}
