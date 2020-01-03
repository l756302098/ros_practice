#include "ros/ros.h"

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

char *strCom = "AAAA12131213BBBBAAAA1213123BBBBAAAA12131213BBBBAAAA12131213BBBB";

int StringToHex(char *str, unsigned char *out, unsigned int *outlen)
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

char *cut(char *s, int m, int n)
{
    char *r = (char *)malloc(n + 1);
    int i;
    for (i = m; i < m + n; i++)
        r[i - m] = s[i];
    r[n] = 0;
    return r;
}

void my_sleep(unsigned long milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds); // 100 ms
#else
    usleep(milliseconds * 1000); // 100 ms
#endif
}

void enumerate_ports()
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

int run(int argc, char **argv)
{
    try
    {
        serial::Serial my_serial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000));
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
        std::cerr << "open port exception:" << e.what() << '\n';
    }
                                                                                                                                             return 0;
}
void publish(int left_wheel,int right_wheel){
    //publish left/right

    //publish odom
    
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    /*
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    return 0;
*/
    try
    {
        // int cnt;
        // unsigned char out[33];

        // unsigned int outlen = 0;
        // StringToHex(strCom, out, &outlen);
        // for (cnt = 0; cnt < outlen; cnt++)
        // {
        //     printf("%02X ", out[cnt]);
        // }
        // putchar(10);

        //split string by end "BBBB"
        int length = strlen(strCom);
        //printf("%i ",length);
        int offset = 4,total = 0;
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
                printf(" %i a line end！ \n",total);
                //check num  if complete deal data
                if (total == com_length)
                {
                    temp = cut(strCom, read_position - 12, com_length);
                    printf("deal %s \n", temp);
                }else{
                    printf("warinng:data is not complete: %s \n", temp);
                }
                total = 0;
            }
        }

        //return run(argc, argv);
    }
    catch (exception &e)
    {
        cerr << "Unhandled Exception: " << e.what() << endl;
    }
    return 0;
}