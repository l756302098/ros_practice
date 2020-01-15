#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int64MultiArray.h"
//#include <geometry_msgs/Twist.h>

#include "std_msgs/String.h"
#include <sstream>

#include <string>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

using std::cerr;
using std::cout;
using std::endl;
using std::exception;
using std::string;
using std::vector;

char *strCom = "AAAA12131213BBBBAAAA12131213BBBBAAAA12131213BBBBAAAA12131213BBBB";

class robot_odometer
{
private:
    ros::NodeHandle n;
    ros::Publisher wheel_pub;
    void enumerate_ports();
    void publish(int left_wheel, int right_wheel);
    std::string port_name;

public:
    robot_odometer();
    ~robot_odometer();
    int run(int argc, char **argv);
    int string_to_hex(char *str, unsigned char *out, unsigned int *outlen);
    char *cut(char *s, int m, int n);
    void deal_data(char *data, int &speed_left, int &speed_right);
    void update();
};

robot_odometer::robot_odometer()
{
    n.param<std::string>("/robot_odometer/port", port_name, "/dev/ttyUSB0");
    std::cout << "port name:" << port_name << std::endl;
    //ROS_INFO("port name:%s",port_name);
    wheel_pub = n.advertise<std_msgs::Int64MultiArray>("/yida/robot_odometer/wheel_speed", 1);
}

robot_odometer::~robot_odometer()
{
}

int robot_odometer::string_to_hex(char *str, unsigned char *out, unsigned int *outlen)
{
    char *p = str;
    char high = 0, low = 0;
    int tmplen = strlen(p), cnt = 0;
    tmplen = strlen(p);
    while (cnt < (tmplen / 2))
    {
        high = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;
        low = (*(++p) > '9' && ((*p <= 'F') || (*p <= 'f'))) ? *(p)-48 - 7 : *(p)-48;
        out[cnt] = ((high & 0x0f) << 4 | (low & 0x0f));
        p++;
        cnt++;
    }
    if (tmplen % 2 != 0)
        out[cnt] = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;

    if (outlen != NULL)
        *outlen = tmplen / 2 + tmplen % 2;
    return tmplen / 2 + tmplen % 2;
}

char *robot_odometer::cut(char *s, int m, int n)
{
    char *r = (char *)malloc(n + 1);
    int i;
    for (i = m; i < m + n; i++)
        r[i - m] = s[i];
    r[n] = 0;
    return r;
}

void robot_odometer::deal_data(char *data, int &speed_left, int &speed_right)
{
    int length = strlen(strCom);
    //printf("%i ",length);
    int offset = 4, total = 0;
    int com_length = 16;
    int read_position = 0;
    char *p = strCom;
    while (read_position + offset < length)
    {
        total++;
        p += 1;
        read_position += 1;
        //find last
        char *temp = cut(strCom, read_position, offset);

        //printf("%s", temp);
        if (strcmp(temp, "BBBB") == 0)
        {
            //printf(" %i a line end！ \n", total);
            //check num  if complete deal data
            if (total == com_length)
            {
                temp = cut(strCom, read_position - 12, com_length);
                printf("deal %s \n", temp);
                //calc 4-8 8-12
                unsigned char out[64];
                unsigned int outlen = 0;
                string_to_hex(strCom, out, &outlen);
                //printf(" length: %i \n", out);
                printf(" out: %s \n", out);
            }
            else
            {
                printf("warinng:data is not complete: %s \n", temp);
            }
            total = 0;
        }
    }
}

void robot_odometer::enumerate_ports()
{
    vector<serial::PortInfo> devices_found = serial::list_ports();

    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end())
    {
        serial::PortInfo device = *iter++;

        printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
               device.hardware_id.c_str());
    }
}

int robot_odometer::run(int argc, char **argv)
{
    try
    {
        serial::Serial my_serial(port_name, 115200, serial::Timeout::simpleTimeout(1000));
        cout << "Is the serial port open?";
        if (my_serial.isOpen())
            cout << " Yes." << endl;
        else
            cout << " No." << endl;
        my_serial.flushInput();

        const int buf_size = 9999;
        int iret;
        char *resultfirst;
        size_t read_size = buf_size;
        int counter = 0;
        int nmatch = 0;
        int left_speed=0, right_speed=0;
        while (1)
        {
            string line_info;
            size_t ret = 0;
            try
            {
                ret = my_serial.readline(line_info, read_size);
            }
            catch (exception ex)
            {
                printf("Something Weird %s \n", ex.what());
                return -1;
            }
            if (ret == 0)
                continue;
            try
            {
                resultfirst = (char *)line_info.c_str();
                std::cout << "resultfirst:" << resultfirst << std::endl;
                //
                //deal_data(resultfirst,left_speed,right_speed);
            }
            catch (exception ex)
            {
                printf("error: %s \n", ex.what());
                return -1;
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << "open port :" << port_name;
        std::cerr << " exception:" << e.what() << '\n';
    }
                                                                                                                                             return 0;
}
void robot_odometer::publish(int left_wheel, int right_wheel)
{
    //publish left/right
    std_msgs::Int64MultiArray array;
    //push left
    array.data.push_back(left_wheel);
    array.data.push_back(right_wheel);
    //Publish array
    wheel_pub.publish(array);
}

void robot_odometer::update()
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_wheel_speed");

    robot_odometer rod;
    ROS_INFO("robot_odometer node started ...");

    unsigned char out[64];
    unsigned int outlen = 0;
    rod.string_to_hex(strCom, out, &outlen);
    for (int i = 0; i < outlen; i++)
    {
        printf("%02X ", out[i]);
    }

    return rod.run(argc,argv);
    /*
    ros::Rate rate(10);
    while (ros::ok())
    {
        rod.update();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
    */
}