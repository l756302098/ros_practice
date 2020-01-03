#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>

#include <iostream>
#include <set> //使用multiset须包含此头文件
#include <map>

using namespace std;

struct Cmp
{
    bool operator()(const float &f1, const float &f2) const
    {
        return f1 < f2;
    }
};

class Student
{
private:
public:
    string name;
    int level;
    int n;
    float score;
    Student(float s_, int level_, string name_)
    {
        score = s_;
        name = name_;
    }
};

int main(int argc, char **argv)
{
    Student s3(0.333, 3, "张三");
    Student s1(0.111, 1, "张一");
    Student s2(0.222, 2, "张二");

    Student s4(0.4, 2, "张四");

    multimap<float, Student, Cmp> stu_map;
    //stu_map[0.4] = s4;
    stu_map.insert(make_pair(0.333, s3));
    stu_map.insert(make_pair(0.222, s2));
    stu_map.insert(make_pair(0.111, s1));
    stu_map.insert(make_pair(0.4, s4));

    multimap<float, Student>::iterator iter;
    for (iter = stu_map.begin(); iter != stu_map.end(); ++iter)
    {
        cout << (*iter).first << endl;
        Student ss = (*iter).second;
        cout << ss.name << endl;
    }

    stu_map.erase(0.222);

    cout << "删除后" << endl;
    Student s5(0.001, 5, "张五");
    stu_map.insert(make_pair(0.001, s5));
    multimap<float, Student>::iterator iter2;
    for (iter2 = stu_map.begin(); iter2 != stu_map.end(); ++iter2)
    {
        cout << (*iter2).first << endl;
        Student ss = (*iter2).second;
        cout << ss.name << endl;
    }

    sleep(2);

    return 0;
}