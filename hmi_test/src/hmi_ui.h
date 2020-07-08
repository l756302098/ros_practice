#ifndef HMI_TEST_HMI_HPP
#define HMI_TEST_HMI_HPP
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <Uart.h>

#include <vector>
#include <queue>
#include <map>

using namespace std;

struct MonNodeS
{
    std::string mon;
    std::string node;
    std::string state;
    std::string res;
    std::string cpu;
    std::string mem;
};

class HmiUI
{
public:
    HmiUI()
    {
        openDevice();
    }
    ~HmiUI()
    {
        ut.close();
    }

    void update(int level, string message)
    {
    }
    void openDevice(string name = "/dev/ttyUSB0")
    {
        isOpen = ut.open(name, 9600);
        if (!isOpen)
        {
            ROS_ERROR("open serial failed!");
        }else{
            ROS_INFO("open serial success!");
        }
    }

    void reopen(){
        ut.close();
        sleep(0.5);
        openDevice();
    }
    void updateData(diagnostic_msgs::DiagnosticArray &diagnostics){
        robotData = diagnostics;
        double diff = ros::Time::now().toSec() - timestamps;
        if(diff>1){
            freshUI();
            timestamps = ros::Time::now().toSec();
        }
    }
    //ui logic
    int getPage();
    bool openStatus();
    void freshPageDevice();
    void freshPageLog();
    void freshPageNode();
    void freshPageOther();
    void freshUI();
    //deal string
    void comCmd(string &cmd, vector<string> vec);
    bool sendCmd(string &cmd);
    void deleteAllMark(string &s, const string &mark);

private:
    diagnostic_msgs::DiagnosticArray robotData;
    std::map<std::string, diagnostic_msgs::DiagnosticStatus> hwStatus;
    vector<MonNodeS> monStatus;
    vector<diagnostic_msgs::DiagnosticStatus> logStatus;
    bool isOpen;
    wincomm::Uart ut;
    int page,pageSub,sys0;
    vector<vector<char>> hmiBack;
    double timestamps;
};

#endif
