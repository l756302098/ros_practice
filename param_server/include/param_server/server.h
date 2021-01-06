#ifndef __PARAM_SERVER_H__
#define __PARAM_SERVER_H__

#include <boost/function.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <map>
#include "yaml-cpp/yaml.h"
#include <param_server/Param.h>
#include <param_server/KeyValue.h>
#include <param_server/KeyValues.h>

//1.load config
//2.update param
//3.save config
namespace param_server
{
    typedef std::map<std::string,std::string> SimpleType;
    typedef boost::function<void(SimpleType &)> CallbackType;
    class Server
    {
    private:
        /* data */
        std::string package_name_,config_name_;
        std::string path_,temp_path_;
        ros::NodeHandle node_handle_;
        ros::ServiceServer set_service_,del_service_;
        ros::Publisher notify_pub_;
        SimpleType map_;
        boost::recursive_mutex &mutex_,own_mutex_;
        CallbackType callback_;
        YAML::Node config_;
        ros::Timer timer_;
        void load();
        void updateConfig();
        bool setConfigCallback(param_server::Param::Request &req,param_server::Param::Response &rsp)
        {
            boost::recursive_mutex::scoped_lock lock(mutex_);
            int index = 0;
            std::vector<param_server::KeyValue> values = req.data;
            std::vector<param_server::KeyValue>::iterator iter = values.begin();
            for(; iter != values.end(); ++iter){
                param_server::KeyValue v = *iter;
                std::string key = v.key;
                std::string value = v.value;
                //check key,overwrite if exist
                if(config_[key]){
                    ROS_INFO("config update key:%s value:%s",key.c_str(),value.c_str());
                    config_[key] = value;
                    index++;
                }else{
                    ROS_INFO("config add key:%s value:%s",key.c_str(),value.c_str());
                    config_[key] = value;
                    index++;
                }
            }
            //update cofig
            if(index>0){
                updateConfig();
                rsp.success = true;
            }
            return true;
        }
        bool delConfigCallback(param_server::Param::Request &req,param_server::Param::Response &rsp){
            boost::recursive_mutex::scoped_lock lock(mutex_);
            int index = 0;
            std::vector<param_server::KeyValue> values = req.data;
            std::vector<param_server::KeyValue>::iterator iter = values.begin();
            for(; iter != values.end(); ++iter){
                param_server::KeyValue v = *iter;
                std::string key = v.key;
                //check key,overwrite if exist
                if(config_[key]){
                    ROS_INFO("config remove key:%s",key.c_str());
                    config_.remove(key);
                    index++;
                }else{
                    ROS_INFO("config no key:%s",key.c_str());
                }
            }
            //update cofig
            if(index>0){
                updateConfig();
                rsp.success = true;
            }
            return true;
        }
        void pub(const ros::TimerEvent &event){
            //publish all key
            param_server::KeyValues values;
            for (auto &kv : map_) {
                param_server::KeyValue data;
                data.key = kv.first;
                data.value = kv.second;
                values.data.push_back(data);
            }
            notify_pub_.publish(values);
        }
    public:
        Server(std::string package_name,std::string config_name,const ros::NodeHandle &nh = ros::NodeHandle("~")):
        node_handle_(nh),
        mutex_(own_mutex_)
        {
            package_name_ = package_name;
            config_name_ = config_name;
            std::string path = ros::package::getPath(package_name_);
            path_ = path +"/"+ config_name_;
            temp_path_ = path_+".temp";
            load();
            //register ros
            notify_pub_ = node_handle_.advertise<param_server::KeyValues>("/param_server/list",1);
            set_service_ = node_handle_.advertiseService("param_server/set",&Server::setConfigCallback, this);
            del_service_ = node_handle_.advertiseService("param_server/del",&Server::delConfigCallback, this);
            timer_ = node_handle_.createTimer(ros::Duration(1.0), &Server::pub, this, false);
        }
        ~Server(){}
        void setCallback(const CallbackType &callback);
        void clearCallback();
        template<typename t>
        void get(const std::string& key, t& value)
        {
            boost::recursive_mutex::scoped_lock lock(mutex_);
            value = config_[key].as<t>(); 
        }
        template<typename t>
        void set(const std::string& key, t value)
        {
            boost::recursive_mutex::scoped_lock lock(mutex_);
            config_[key] = value;
            updateConfig();
        }
        bool exist(const std::string& key){
            return !config_[key].IsNull();
        }
    };    
}
#endif