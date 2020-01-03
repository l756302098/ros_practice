#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>

#include <iostream>
#include <set> //使用multiset须包含此头文件

using namespace std;

class Student
{
private:
public:
    string name;
    int level;
    int n;
    Student(int n_, int level_, string name_) { n = n_; }
    friend bool operator<(const Student &a1, const Student &a2)
    {
        return a1.n < a2.n;
    }
};

typedef multiset<Student> stu_set;

int main(int argc, char **argv)
{
    Student s3(3, 3, "张三");
    Student s1(1, 1, "张一");
    Student s2(2, 2, "张二");

    stu_set set1;
    set1.insert(s1);
    set1.insert(s2);
    set1.insert(s3);

    Student s4(3, 3, "张三");
    Student s5(4, 4, "张四");
    set1.insert(s4);
    set1.insert(s5);

    cout << "size" << set1.size() << endl;

    //查找前3个元素
    cout << "前三个元素：" << endl;
    int count = 0;
    stu_set::iterator iter;
    for (iter = set1.begin(); iter != set1.end(); ++iter)
    {
        Student ss = *iter;
        cout << ss.n << endl;
        count++;
        if (count >= 3)
        {
            break;
        }
    }

    //查找后三个元素
    cout << "后三个元素：" << endl;
    count = 0;
    stu_set::reverse_iterator iter2;
    for (iter2 = set1.rbegin(); iter2 != set1.rend(); ++iter2)
    {
        Student ss = *iter2;
        cout << ss.n << endl;
        count++;
        if (count >= 3)
        {
            break;
        }
    }

    return 0;
}