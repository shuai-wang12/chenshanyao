#pragma once

#include <ros/ros.h>
#include <deque>
#include <mutex>
#include <memory>
template <typename T>
class subscriber{
public:
    subscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);
    void parseData(std::deque<T>& deque_msg_ptr);
private:
    void messageCallback(const T& msg_ptr);

private:
    ros::Subscriber subscriber_;
    std::deque<T> deque_;
    std::mutex buff_mutex_;
};

template <typename T>
subscriber<T>::subscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size){
    subscriber_ = nh.subscribe(topic_name,buff_size,&subscriber<T>::messageCallback,this);
}

template <typename T>
void subscriber<T>::parseData(std::deque<T>& deque_msg_ptr){
    buff_mutex_.lock();
    if (!deque_.empty()) {
        deque_msg_ptr.insert(deque_msg_ptr.end(),
                                    deque_.begin(),
                                    deque_.end()
        );

        deque_.clear();
    }
    buff_mutex_.unlock();
}

template <typename T>
void subscriber<T>::messageCallback(const T& msg_ptr){
    buff_mutex_.lock();
    deque_.emplace_back(msg_ptr);
    buff_mutex_.unlock();
}
