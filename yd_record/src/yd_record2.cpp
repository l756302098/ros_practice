
#include <rosbag_record_cpp/rosbag_record.h>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include "std_msgs/Int16.h"
#include <exception>

using namespace std;

class record_node
{
private:
  //ros::Subscriber cmd_sub;
  bool isRecord;
  std::string topic,
      path_save,
      file_name;
  ros::Time start_time, t;
  ros::Duration duration;
  int seconds;
  rosbag_record_cpp::RecorderOptions options;
  rosbag_record_cpp::ROSBagRecord recorder;
  std::vector<std::string> delete_files;

public:
  static bool isStop;
  ros::NodeHandle node;
  pthread_t tid;
  record_node();
  ~record_node();

  static void cmdCallback(const std_msgs::Int16::ConstPtr &msg);

  static void *update(void *tmp);
};

bool record_node::isStop = false;

record_node::record_node()
{
  ros::param::get("/record/topic", topic);
  ros::param::get("/record/path_save", path_save);
  ros::param::get("/record/file_name", file_name);
  ros::param::get("/record/seconds", seconds);
  ROS_INFO_STREAM("topics:    " << topic);
  ROS_INFO_STREAM("name:      " << path_save);
  ROS_INFO_STREAM("file_name: " << file_name);
  ROS_INFO_STREAM("seconds: " << seconds);

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

  duration = ros::Duration(seconds);
  start_time = ros::Time::now();
  isRecord = false;
  //isStop = false;

  //cmd_sub = node.subscribe<std_msgs::Int16>("/yd_record/cmd", 10, &record_node::cmdCallback, this);

  //pthread_create(&tid, NULL, record_node::update, (void *)this);
}

record_node::~record_node()
{
  delete_files.clear();
}

void *record_node::update(void *tmp)
{
  record_node *p = (record_node *)tmp;
  ros::Duration duration = ros::Duration(1);
  while (ros::ok())
  {
    p->t = ros::Time::now();
    ROS_INFO_STREAM("child thread Recording ... ");
    //std::cout << "yd_record Recording ... " << std::endl;
    //stop
    if (record_node::isStop && p->isRecord)
    {
      ROS_INFO_STREAM("yd_record isStop 1");
      try
      {
        p->recorder.stopRecording();
        p->isRecord = false;
      }
      catch (std::exception &e)
      {
        ROS_ERROR_STREAM("exception:" << e.what());
      }
    }

    // Start
    if (!p->isRecord && !record_node::isStop)
    {
      ROS_INFO_STREAM("yd_record Start 2");
      p->recorder.startRecording(p->options);
      p->start_time = ros::Time::now();
      p->isRecord = true;
    }
    if (p->t - p->start_time > p->duration && p->isRecord)
    {
      ROS_INFO_STREAM("yd_record Stop 3");
      // Stop
      try
      {
        p->recorder.stopRecording();
      }
      catch (exception &e)
      {
        ROS_ERROR_STREAM("exception:" << e.what());
      }

      //delete
      boost::filesystem::recursive_directory_iterator beg_iter(p->path_save);
      boost::filesystem::recursive_directory_iterator end_iter;
      p->delete_files.clear();
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
            std::string::size_type end_postion = strPath.find(".bag");
            std::string::size_type name_postion = strPath.find(p->file_name);
            if (end_postion != std::string::npos && name_postion != std::string::npos)
            {

              p->delete_files.push_back(strPath);
            }
          }
        }
      }
      //delete
      int count = p->delete_files.size();
      for (int i = 0; i < count; i++)
      {
        ROS_INFO_STREAM("delete strPath " << p->delete_files[i]);
        boost::filesystem::remove(p->delete_files[i]);
      }

      p->recorder.startRecording(p->options);
      p->start_time = ros::Time::now();
      p->isRecord = true;
    }
    duration.sleep();
  }
}

void record_node::cmdCallback(const std_msgs::Int16::ConstPtr &msg)
{
  ROS_INFO_STREAM("msg>>>>: " << msg->data);
  //1 start 0 end
  if (msg->data == 1)
  {
    record_node::isStop = true;
  }
  else
  {
    record_node::isStop = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "yd_record");
  record_node rec_node;
  ROS_INFO("yd_record ros node started...");

  ros::Subscriber cmd_sub = rec_node.node.subscribe<std_msgs::Int16>("/yd_record/cmd", 10, &record_node::cmdCallback);
  ros::spin();

  rec_node.recorder.startRecording();

  //ros::Rate loop_rate(1);

  // while (ros::ok())
  // {
  //   ROS_INFO("main thread...");
  //   //node.update();
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  return 0;
}