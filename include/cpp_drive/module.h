#include <iostream>
#include <string>
#include <random>
#include <cstdlib>
#include <deque>
#include <numeric>
#include <ros/ros.h>
// #include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <xycar_msgs/xycar_motor.h>
#include <sensor_msgs/Image.h>

class MovingAverage
{
    public:
        MovingAverage(const int &number): sampling_number(number)
        {
            for (int i = 1; i <= sampling_number; ++i) {
                weights.push_back(i);
            }
        }
        ~MovingAverage() = default;
    
    private:
        const int sampling_number;
        std::deque<int> data;
        std::vector<int> weights;

    public:
        void add_sample(int &new_sample)
        {
            data.push_back(new_sample);
            if (data.size() > sampling_number) {
                data.pop_front();   
            }
        }
        float get_wmm()
        {
            int s = 0, sum = 0;
            auto start = weights.begin();
            for (int i = 0; i < data.size(); ++i) {
                s += data[i] * weights[i];
            }
            for (int i = 0; i < data.size(); ++i) {
                sum += weights[i];
            }
            return static_cast<float>(s) / static_cast<float>(sum);
        }
};

class PID
{
    public:
        PID(float &kp_, float &ki_, float &kd_) : kp(kp_), ki(ki_), kd(kd_)
        {
            p_error = 0.0;
            i_error = 0.0;
            d_error = 0.0;
        }
    
    private:
        float kp, ki, kd;
        float p_error, i_error, d_error;
    
    public:
        float pid_control(int &cte_)
        {
            float cte = static_cast<float>(cte_);
            d_error = cte - p_error;
            p_error = cte;
            i_error += cte;
            return kp*p_error + ki*i_error + kd*d_error;
        }
};
