#include "param_server/server.h"
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>

namespace param_server
{
void Server::setCallback(const CallbackType &callback)
{
    boost::recursive_mutex::scoped_lock lock(mutex_);
    callback_ = callback;
}

void Server::clearCallback()
{
    boost::recursive_mutex::scoped_lock lock(mutex_);
    callback_.clear();
}

void Server::load(){
    boost::recursive_mutex::scoped_lock lock(mutex_);
    ROS_INFO("config path:%s",path_.c_str());
    //check file
    boost::filesystem::path p(path_);
    if(!boost::filesystem::exists(p)){
        ROS_ERROR("file does not exist,path:%s",path_.c_str());
        return;
    }
    //load
    config_ = YAML::LoadFile(path_);
    //std::cout << "config:" << config_ <<std::endl;
    ROS_INFO("Node type %i, size %i",config_.Type(),config_.size());
    ROS_INFO("config load,params:");
    for(YAML::const_iterator it=config_.begin();it!=config_.end();++it) {
        std::string key = it->first.as<std::string>();
        std::string value = it->second.as<std::string>();
        ROS_INFO("key:%s value:%s",key.c_str(),value.c_str());
        map_[key] = value;
    }
}

void Server::updateConfig(){
    map_.clear();
    ROS_INFO("config reload, params:");
    for(YAML::const_iterator it=config_.begin();it!=config_.end();++it) {
        std::string key = it->first.as<std::string>();
        std::string value = it->second.as<std::string>();
        ROS_INFO("key:%s value:%s",key.c_str(),value.c_str());
        map_[key] = value;
    }
    //save to temp file
    std::ofstream file(temp_path_);
    file << config_ << std::endl;
    //replace file overwrite
    boost::filesystem::copy_file(temp_path_, path_, boost::filesystem::copy_option::overwrite_if_exists);
    //remove temp file
    boost::filesystem::remove(temp_path_);
    //notify callback
    if(callback_!=NULL)
        callback_(map_);
}
}