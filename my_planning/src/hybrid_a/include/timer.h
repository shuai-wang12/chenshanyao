#pragma once

#include <chrono>
#include <iostream>
class Timer
{
private:
    /* data */
public:
    Timer(){
        Begin();
    }
    void Begin(){
        start_ = std::chrono::system_clock::now();
    }
    void End(const std::string &task_name) {
        end_ = std::chrono::system_clock::now();
        std::chrono::duration<double> use_time = end_ - start_;

        std::cout.precision(3);
        std::cout << task_name << " use time(ms): " << use_time.count() * 1000 << std::endl;
    }

    double End() {
        end_ = std::chrono::system_clock::now();
        std::chrono::duration<double> use_time = end_ - start_;
        return use_time.count() * 1000.0;
    }
    std::chrono::time_point<std::chrono::system_clock> start_, end_;
};
