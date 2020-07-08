#include <hmi_ui.h>
#include <Uart.h>
#include <vector>

using namespace std;

int HmiUI::getPage()
{
    string cmd;
    vector<string> array;
    array.push_back("sendme");
    array.push_back("get page_sub.val");
    array.push_back("get sys0");
    comCmd(cmd, array);
    ut.writeFromBuffer((char *)&cmd[0], cmd.size());

    char data[1024];
    int size = ut.readToBuffer(&data[0], 1024);
    if (size == 0)
    {
        ROS_INFO("read buffer size is 0");
        page = -1;
        return -1;
    }
    //split 0xff 0xff 0xff
    //ROS_WARN("size:%i", size);
    int start = 0;
    int end = 0;
    hmiBack.clear();
    for (int i = 0; i < size - 2; i++)
    {
        //ROS_INFO("i:%i data:%i", i, data[i]);
        if (data[i] == -1 && data[i + 1] == -1 && data[i + 2] == -1)
        {
            end = i;
            vector<char> cmd_result;
            int k = 0;
            for (int j = start; j < end; j++)
            {
                //ROS_INFO("j:%i data:%x", j, data[j]);
                cmd_result.push_back(data[j]);
                if (k == 0)
                {
                    if (data[j] == 0x66)
                    {
                        page = data[start + 1];
                    }
                    else if (data[j] == 0X71)
                    {
                        if (i > 12)
                        {
                            sys0 = data[start + 1];
                        }
                        else
                        {
                            pageSub = data[start + 1];
                        }
                    }
                }
                k++;
            }
            hmiBack.push_back(cmd_result);
            start = i + 3;
            i += 2;
        }
    }
    ROS_INFO("page:%i sub:%i sys0:%i", page, pageSub, sys0);
    return 0;
}

bool HmiUI::openStatus()
{
    return isOpen;
}

void HmiUI::comCmd(std::string &cmd, vector<std::string> vec)
{
    if (vec.size() > 0)
    {
        for (int i = 0; i < vec.size(); i++)
        {
            cmd += vec[i];
            cmd += 0xff;
            cmd += 0xff;
            cmd += 0xff;
        }
    }
}

bool HmiUI::sendCmd(string &cmd)
{
    ROS_INFO("cmd %x", cmd);
    ut.writeFromBuffer((char *)&cmd[0], cmd.size());

    char data[1024];
    int size = ut.readToBuffer(&data[0], 1024);
    if (size == 4 && data[0] == 0x00)
    {
        ROS_INFO("error code:0x00:无效指令");
    }
    if (size == 4 && data[0] == 0x01)
    {
        ROS_INFO("指令成功执行");
    }
    if (size == 4 && data[0] == 0x1a)
    {
        ROS_INFO("error code:0x1a:变量名称无效");
    }
    return true;
}

void HmiUI::deleteAllMark(string &s, const string &mark)
{
    size_t nSize = mark.size();
    while (1)
    {
        size_t pos = s.find(mark); //  尤其是这里
        if (pos == string::npos)
        {
            return;
        }

        s.erase(pos, nSize);
    }
}

void HmiUI::freshPageDevice()
{
    ROS_INFO("freshPageDevice");
    //Parse data
    hwStatus.clear();
    for (int i = 0; i < robotData.status.size(); i++)
    {
        diagnostic_msgs::DiagnosticStatus status = robotData.status[i];
        string hwHead("/hw");
        bool isHW = status.name.compare(0, hwHead.size(), hwHead) == 0;
        if (isHW)
        {
            hwStatus[status.name] = status;
        }
    }
    ROS_INFO("new size:%i", hwStatus.size());
    if (hwStatus.size() == 0)
        return;

    string cmd = "";
    vector<string> array;
    //title
    array.push_back("t2.txt=\"IP:250\"");
    //cpu
    map<std::string, diagnostic_msgs::DiagnosticStatus>::iterator iter;
    iter = hwStatus.find("/hw/cpu");
    double cpuPercent = 0;
    string cpuStr = "0";
    if (iter != hwStatus.end())
    {
        diagnostic_msgs::DiagnosticStatus st = iter->second;
        for (int j = 0; j < st.values.size(); j++)
        {
            diagnostic_msgs::KeyValue kv = st.values[j];
            double cp = atof(kv.value.c_str());
            cpuPercent += cp;
        }
        char buffer[20];
        sprintf(buffer, "%3.0f", cpuPercent);
        cpuStr = buffer;
        //cpuStr = std::to_string(cpuPercent);
    }
    else
    {
        ROS_INFO("Do not Find CPU");
    }

    array.push_back("t0.txt=\"" + cpuStr + "\"");
    string j0Str = "j0.val=" + cpuStr;
    deleteAllMark(j0Str, " ");
    array.push_back(j0Str);
    //mem
    ROS_INFO("calc mem");
    iter = hwStatus.find("/hw/mem");
    double memPercent = 0;
    int memUsed = 0;
    string memStr = "0";
    if (iter != hwStatus.end())
    {
        diagnostic_msgs::DiagnosticStatus st = iter->second;
        if (st.values.size() != 0)
        {
            diagnostic_msgs::KeyValue kv = st.values[0];
            memUsed = atoi(kv.value.c_str());
            memPercent = memUsed / (1024 * 1024 * 1024 * 16.0) * 100;
            //memStr = std::to_string(memPercent);
            char buffer[20];
            sprintf(buffer, "%3.0f", memPercent);
            memStr = buffer;
        }
    }
    else
    {
        ROS_INFO("Do not Find MEM");
    }
    array.push_back("t1.txt=\"" + memStr + "\"");
    string j1Str = "j1.val=" + memStr;
    deleteAllMark(j1Str, " ");
    array.push_back(j1Str);
    //network
    iter = hwStatus.find("/hw/network");
    int txtNum = 3;
    if (iter != hwStatus.end())
    {
        diagnostic_msgs::DiagnosticStatus st = iter->second;
        if (st.values.size() != 0)
        {
            diagnostic_msgs::KeyValue totalKv = st.values[0];
            int total = atoi(totalKv.value.c_str());
            total = total > 3 ? 3 : total;
            for (int j = 0; j < total; j++)
            {
                for (int k = 0; k < 3; k++)
                {
                    int index = 1 + 3 * j + k;
                    //ROS_INFO("index:%i",index);
                    diagnostic_msgs::KeyValue kv = st.values[index];
                    if (txtNum % 3 == 0)
                    {
                        array.push_back("t" + std::to_string(txtNum) + ".txt=\"" + kv.value.c_str() + "\"");
                    }
                    else
                    {
                        double percent = atof(kv.value.c_str()) / 1024.0 / 1024;
                        char buffer[20];
                        sprintf(buffer, "%4.2f", percent);
                        string pStr = buffer;
                        pStr = pStr + "Mb";
                        array.push_back("t" + std::to_string(txtNum) + ".txt=\"" + pStr + "\"");
                    }
                    txtNum++;
                }
            }
        }
    }
    else
    {
        ROS_INFO("Do not Find network");
    }
    for (; txtNum < 12;)
    {
        array.push_back("t" + std::to_string(txtNum) + ".txt=\"\"");
        txtNum++;
    }
    //cmd
    comCmd(cmd, array);
    sendCmd(cmd);
}

void HmiUI::freshPageLog()
{
    ROS_INFO("freshPageLog");
    string cmd = "";
    vector<string> array;
    //Parse data
    logStatus.clear();
    for (int i = 0; i < robotData.status.size(); i++)
    {
        diagnostic_msgs::DiagnosticStatus status = robotData.status[i];
        //
        string hwHead("/hw");
        bool isHW = status.name.compare(0, hwHead.size(), hwHead) == 0;
        string monHead("/rosmon");
        bool isMon = status.name.compare(0, monHead.size(), monHead) == 0;
        if (!isHW && !isMon)
        {
            logStatus.push_back(status);
        }
    }
    if (logStatus.size() == 0)
        return;
    //fill data
    int pageNum = logStatus.size() / 5;
    int pageRem = logStatus.size() % 5;
    pageNum = pageRem > 0 ? pageNum++ : pageNum;
    int txtNum = 3;
    //std::cout << "pageNum:" << pageNum << std::endl;
    array.push_back("total_page.val=" + std::to_string(pageNum));
    if (pageNum <= 1)
    {
        array.push_back("vis b0,0");
        array.push_back("vis b1,0");
    }
    else
    {
        if (pageSub == 0)
        {
            array.push_back("vis b0,0");
            array.push_back("vis b1,1");
        }
        else if (pageSub == pageNum - 1)
        {
            array.push_back("vis b0,1");
            array.push_back("vis b1,0");
        }
        else
        {
            array.push_back("vis b0,1");
            array.push_back("vis b1,1");
        }
    }

    //calc page num
    int start = pageSub * 3;
    int end = logStatus.size() >= (start + 3) ? (start + 3) : (start + logStatus.size() % 3);
    std::cout << "start:" << start << " end:" << end << std::endl;
    for (start; start < end; start++)
    {
        diagnostic_msgs::DiagnosticStatus status = logStatus[start];
        string nodeStr = "t" + std::to_string(txtNum) + ".txt=\"" + status.name + "\"";
        array.push_back(nodeStr);
        txtNum++;
        if (status.level == 0)
        {
            array.push_back("t" + std::to_string(txtNum) + ".txt=\"OK\"");
        }
        else if (status.level == 1)
        {
            array.push_back("t" + std::to_string(txtNum) + ".txt=\"WARN\"");
        }
        else if (status.level == 2)
        {
            array.push_back("t" + std::to_string(txtNum) + ".txt=\"ERROR\"");
        }
        txtNum++;
        array.push_back("t" + std::to_string(txtNum) + ".txt=\"" + status.message + "\"");
        txtNum++;
    }
    for (; txtNum < 12;)
    {
        array.push_back("t" + std::to_string(txtNum) + ".txt=\"\"");
        txtNum++;
    }
    comCmd(cmd, array);
    sendCmd(cmd);
}

void HmiUI::freshPageNode()
{
    ROS_INFO("freshPageNode");
    string cmd = "";
    vector<string> array;
    monStatus.clear();
    //Parse data
    for (int i = 0; i < robotData.status.size(); i++)
    {
        diagnostic_msgs::DiagnosticStatus status = robotData.status[i];
        string monHead("/rosmon");
        bool isMon = status.name.compare(0, monHead.size(), monHead) == 0;
        if (isMon && status.values.size() > 0)
        {
            diagnostic_msgs::KeyValue totalKv = status.values[0];
            int num = atoi(totalKv.value.c_str());
            int array_sub = 1;

            for (int j = 0; j < num; j++)
            {
                diagnostic_msgs::KeyValue kv1 = status.values[array_sub];
                array_sub++;
                diagnostic_msgs::KeyValue kv2 = status.values[array_sub];
                array_sub++;
                diagnostic_msgs::KeyValue kv3 = status.values[array_sub];
                array_sub++;
                diagnostic_msgs::KeyValue kv4 = status.values[array_sub];
                array_sub++;
                diagnostic_msgs::KeyValue kv5 = status.values[array_sub];
                array_sub++;
                diagnostic_msgs::KeyValue kv6 = status.values[array_sub];
                array_sub++;
                MonNodeS mns;
                mns.mon = status.name;
                mns.node = kv1.value;
                mns.state = kv2.value;
                mns.res = kv3.value;
                mns.cpu = std::to_string(atof(kv4.value.c_str()) + atof(kv5.value.c_str()));
                mns.mem = std::to_string(atoi(kv6.value.c_str()) / 1024 / 1024) + " MB";
                monStatus.push_back(mns);
            }
        }
    }
    if (monStatus.size() == 0)
        return;
    //std::cout << "size:" << monStatus.size() << std::endl;
    //TODO:set page num
    int pageNum = monStatus.size() / 5;
    int pageRem = monStatus.size() % 5;
    pageNum = pageRem > 0 ? pageNum++ : pageNum;
    //array.push_back("t1.txt=\""+memStr+"\"");
    int txtNum = 5;
    //std::cout << "pageNum:" << pageNum << std::endl;
    array.push_back("total_page.val=" + std::to_string(pageNum));
    if (pageNum <= 1)
    {
        array.push_back("vis b0,0");
        array.push_back("vis b1,0");
    }
    else
    {
        if (pageSub == 0)
        {
            array.push_back("vis b0,0");
            array.push_back("vis b1,1");
        }
        else if (pageSub == pageNum - 1)
        {
            array.push_back("vis b0,1");
            array.push_back("vis b1,0");
        }
        else
        {
            array.push_back("vis b0,1");
            array.push_back("vis b1,1");
        }
    }
    //calc page num
    int start = pageSub * 5;
    int end = monStatus.size() >= (start + 5) ? (start + 5) : (start + monStatus.size() % 5);
    std::cout << "start:" << start << " end:" << end << std::endl;
    //clear
    for (start; start < end; start++)
    {
        MonNodeS monNode = monStatus[start];
        array.push_back("t" + std::to_string(txtNum) + ".txt=\"" + monNode.mon + "\"");
        txtNum++;
        array.push_back("t" + std::to_string(txtNum) + ".txt=\"" + monNode.node + "\"");
        txtNum++;
        array.push_back("t" + std::to_string(txtNum) + ".txt=\"" + monNode.res + "\"");
        txtNum++;
        array.push_back("t" + std::to_string(txtNum) + ".txt=\"" + monNode.cpu + "\"");
        txtNum++;
        array.push_back("t" + std::to_string(txtNum) + ".txt=\"" + monNode.mem + "\"");
        txtNum++;
    }
    for (; txtNum < 30;)
    {
        array.push_back("t" + std::to_string(txtNum) + ".txt=\"\"");
        txtNum++;
    }
    comCmd(cmd, array);
    sendCmd(cmd);
}
void HmiUI::freshPageOther()
{
    if (sys0 == 1)
    {
        ROS_INFO("restart ...");
    }
    //reset
    string cmd = "";
    vector<string> array;
    array.push_back("sys0=0");
    comCmd(cmd, array);
    sendCmd(cmd);
}
void HmiUI::freshUI()
{
    try
    {
        if (!isOpen)
        {
            ROS_INFO("serial is disconnect !Try reconnecting...");
            reopen();
            return;
        }
        getPage();
        if (page == 0)
        {
        }
        else if (page == 1)
        {
            freshPageDevice();
        }
        else if (page == 2)
        {
            freshPageLog();
        }
        else if (page == 3)
        {
            freshPageNode();
        }
        else if (page == 4)
        {
            freshPageOther();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}