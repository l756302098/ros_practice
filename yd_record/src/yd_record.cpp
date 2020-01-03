
#include <rosbag_record_cpp/rosbag_record.h>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include "std_msgs/Int16.h"
#include <iostream>
#include <ctime>
#include <string>
#include <time.h>

using namespace std;

std::string topic, path_save, file_name;
ros::Time start_time, t;
ros::Duration duration;
bool isRecord, isStop;
int seconds;

void cmdCallback(const std_msgs::Int16::ConstPtr &msg)
{
  ROS_INFO_STREAM("msg>>>>: " << msg->data);
  //1 start 0 end
  if (msg->data == 1)
  {
    isStop = false;
  }
  else if (msg->data == 0)
  {
    isStop = true;
    //ros::shutdown();
  }
}

time_t StringToDatetime(string str)
{
  char *cha = (char *)str.data();                                                 // 将string转换成char*。
  tm tm_;                                                                         // 定义tm结构体。
  int year, month, day, hour, minute, second;                                     // 定义时间的各个int临时变量。
  sscanf(cha, "%d-%d-%d-%d-%d-%d", &year, &month, &day, &hour, &minute, &second); // 将string存储的日期时间，转换为int临时变量。
  tm_.tm_year = year - 1900;                                                      // 年，由于tm结构体存储的是从1900年开始的时间，所以tm_year为int临时变量减去1900。
  tm_.tm_mon = month - 1;                                                         // 月，由于tm结构体的月份存储范围为0-11，所以tm_mon为int临时变量减去1。
  tm_.tm_mday = day;                                                              // 日。
  tm_.tm_hour = hour;                                                             // 时。
  tm_.tm_min = minute;                                                            // 分。
  tm_.tm_sec = second;                                                            // 秒。
  tm_.tm_isdst = 0;                                                               // 非夏令时。
  time_t t_ = mktime(&tm_);                                                       // 将tm结构体转换成time_t格式。
  return t_;                                                                      // 返回值。
}

void *update(void *tmp)
{
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle *p = (ros::NodeHandle *)tmp;
  ros::Subscriber sub = p->subscribe<std_msgs::Int16>("/yd_record/cmd", 1, cmdCallback);
  ros::spin();
}

bool startswith(const std::string &str, const std::string &start)
{
  int srclen = str.size();
  int startlen = start.size();
  if (srclen >= startlen)
  {
    string temp = str.substr(0, startlen);
    if (temp == start)
      return true;
  }

  return false;
}

bool endswith(const std::string &str, const std::string &end)
{
  int srclen = str.size();
  int endlen = end.size();
  if (srclen >= endlen)
  {
    string temp = str.substr(srclen - endlen, endlen);
    if (temp == end)
      return true;
  }

  return false;
}

//strutils.cpp
vector<string> split(const string &srcstr, const string &delimeter)
{
  vector<string> ret(0); //use ret save the spilted reault
  if (srcstr.empty())    //judge the arguments
  {
    return ret;
  }
  string::size_type pos_begin = srcstr.find_first_not_of(delimeter); //find first element of srcstr

  string::size_type dlm_pos;        //the delimeter postion
  string temp;                      //use third-party temp to save splited element
  while (pos_begin != string::npos) //if not a next of end, continue spliting
  {
    dlm_pos = srcstr.find(delimeter, pos_begin); //find the delimeter symbol
    if (dlm_pos != string::npos)
    {
      temp = srcstr.substr(pos_begin, dlm_pos - pos_begin);
      pos_begin = dlm_pos + delimeter.length();
    }
    else
    {
      temp = srcstr.substr(pos_begin);
      pos_begin = dlm_pos;
    }
    if (!temp.empty())
      ret.push_back(temp);
  }
  return ret;
}

string deal(const string &srcstr)
{
  string ret = "";
  int srclen = srcstr.size();
  string temp = srcstr.substr(0, srclen - 4);
  //ROS_INFO_STREAM("temp:    " << temp);
  int templen = temp.size();
  ret = temp.substr(7, templen - 1);
  //ROS_INFO_STREAM("ret:    " << ret);
  return ret;
}

string dealA(const string &srcstr)
{
  string ret = "";
  int srclen = srcstr.size();
  string temp = srcstr.substr(0, srclen - 7);
  //ROS_INFO_STREAM("temp:    " << temp);
  int templen = temp.size();
  ret = temp.substr(7, templen - 1);
  //ROS_INFO_STREAM("ret:    " << ret);
  return ret;
}

bool compare(const string &srcstr)
{
  time_t t1 = StringToDatetime(srcstr);
  std::cout << "t1:" << t1 << std::endl;
  time_t timep;
  struct tm *p;
  p = localtime(&timep);
  timep = mktime(p);
  time(&timep);
  std::cout << "now:" << timep << std::endl;
  if (timep - t1 > 2 * seconds)
  {
    return true;
  }
  return false;
}

int main(int argc, char **argv)
{
  ROS_INFO_STREAM_NAMED("yd_record", "yd_record");
  ros::init(argc, argv, "yd_record");

  ros::NodeHandle nh;

  //pthread_t tid;
  //pthread_create(&tid, NULL, update, (void *)&nh);
  ros::Subscriber sub = nh.subscribe<std_msgs::Int16>("/yd_record/cmd", 1, cmdCallback);

  rosbag_record_cpp::ROSBagRecord recorder;

  //init params
  ros::param::get("/record/topic", topic);
  ros::param::get("/record/path_save", path_save);
  ros::param::get("/record/file_name", file_name);
  ros::param::get("/record/seconds", seconds);
  ROS_INFO_STREAM("topics:    " << topic);
  ROS_INFO_STREAM("name:      " << path_save);
  ROS_INFO_STREAM("file_name: " << file_name);
  ROS_INFO_STREAM("seconds: " << seconds);

  duration = ros::Duration(seconds);
  // Set the options for this recording
  rosbag_record_cpp::RecorderOptions options;
  //deal topis list
  boost::regex rgx("\\s+");
  boost::sregex_token_iterator iter(topic.begin(), topic.end(), rgx, -1);
  boost::sregex_token_iterator end;
  std::vector<std::string> topics;
  for (; iter != end; ++iter)
  {
    ROS_INFO_STREAM("topic: " << *iter);
    topics.push_back(*iter);
  }
  options.topics = topics;
  options.prefix = path_save + "/" + file_name;
  options.max_duration = duration;
  options.split = true;

  start_time = ros::Time::now();
  isRecord = false;
  isStop = false;
  ros::Rate loop_rate(1);
  std::vector<std::string> delete_files;
  recorder.startRecording(options);

  while (ros::ok())
  {
    t = ros::Time::now();
    ROS_INFO_STREAM("yd_record Recording ... ");
    if (isStop)
    {
      recorder.stopRecording();
      break;
      //ros::Duration(1).sleep();
      //recorder.startRecording(options);
      isStop = false;
    }
    if (t - start_time > duration)
    {
      boost::filesystem::recursive_directory_iterator beg_iter(path_save);
      boost::filesystem::recursive_directory_iterator end_iter;
      delete_files.clear();
      for (; beg_iter != end_iter; ++beg_iter)
      {
        if (boost::filesystem::is_directory(*beg_iter))
        {
          continue;
        }
        else
        {
          std::string strPath = beg_iter->path().string(); //遍历出来的文件名称
          if (boost::filesystem::exists(strPath))
          {
            ROS_INFO_STREAM("exists strPath " << strPath);

            bool isEnd = endswith(strPath, ".bag");
            if (isEnd)
            {
              vector<string> files = split(strPath, "/");
              int num = files.size();
              if (num > 0)
              {
                string endName = files[num - 1];
                string result = deal(endName);
                ROS_INFO_STREAM(" result :" << result);
                if (compare(result))
                {
                  ROS_INFO_STREAM("add strPath :" << strPath);
                  delete_files.push_back(strPath);
                }
              }
            }
            //delete active
            isEnd = endswith(strPath, ".active");
            if (isEnd)
            {
              vector<string> files = split(strPath, "/");
              int num = files.size();
              if (num > 0)
              {
                string endName = files[num - 1];
                string result = dealA(endName);
                ROS_INFO_STREAM(" result :" << result);
                if (compare(result))
                {
                  ROS_INFO_STREAM("add strPath :" << strPath);
                  delete_files.push_back(strPath);
                }
              }
            }
          }
        }
      }
      int count = delete_files.size();
      for (int i = 0; i < count; i++)
      {
        ROS_INFO_STREAM("delete strPath " << delete_files[i]);
        boost::filesystem::remove(delete_files[i]);
      }
      start_time = ros::Time::now();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  recorder.stopRecording();
  ROS_INFO_STREAM_NAMED("yd_record", "Shutting down...");

  return 0;
}
