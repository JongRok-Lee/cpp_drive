#include "cpp_drive/module.h"

// Functions
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
std::vector<float> process_image();
std::pair<std::vector<cv::Vec4i>, std::vector<cv::Vec4i>> divide_left_right(std::vector<cv::Vec4i> &lines);
std::pair<int, float> get_line_pos(std::vector<cv::Vec4i> &lines, bool left, bool right);
std::pair<float, float> get_line_params(std::vector<cv::Vec4i> &lines);
void draw_lines(std::vector<cv::Vec4i> &lines);
void draw_rectangles(int &lpos, int &rpos, int &ma_mpos);
void velocity_control(float &angle);
void drive(ros::Publisher &pub, float &angle, float &speed);

// Global variables
cv_bridge::CvImagePtr cv_ptr;
cv::Mat frame, show;
int low_threshold = 150;
int high_threshold = 250;
int Width = 640;
int Height = 480;
int Offset = 340;
int Gap = 40;
int low_slope_threshold = 0, high_slope_threshold = 10;
const int sampling_number = 20;
float speed = 0.0;
bool show_img = true;

int main(int argc, char** argv) {

    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<xycar_msgs::xycar_motor>("xycar_motor", 1000);
    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw/", 1, imageCallback);
    
    MovingAverage ma(sampling_number);
    float kp, ki, kd;
    ros::param::get("kp", kp);
    ros::param::get("ki", ki);
    ros::param::get("kd", kd);
    ros::param::get("show_img", show_img);
    std::cout << "Kp: " << kp << "\tKi: " << ki << "\tKd: " << kd << "\tshow_img: " << show_img <<"\n";
    PID pid(kp, ki, kd);

    int lpos, rpos, mpos, ma_pos, cte;
    float l_slope, r_slope, steer_angle;
    while (ros::ok()) {
        ros::spinOnce();
        if (frame.cols != 640) {
            continue;
        }

        std::vector<float> pos_and_slope = process_image();
        lpos = static_cast<int>(pos_and_slope[0]), rpos = static_cast<int>(pos_and_slope[1]);
        l_slope = pos_and_slope[2], r_slope = pos_and_slope[3];
        mpos = (lpos + rpos) * 0.5;
        ma.add_sample(mpos);
        ma_pos = ma.get_wmm();

        cte = ma_pos - Width * 0.5;
        steer_angle = pid.pid_control(cte);

        steer_angle = std::max(-50.0f, std::min(steer_angle, 50.0f));
        velocity_control(steer_angle);
        drive(pub, steer_angle, speed);

        // Visualization
        if (show_img == true) {
            draw_rectangles(lpos, rpos, ma_pos);
            cv::imshow("show", show);
            cv::waitKey(1);
        }
        

        std::cout << "l_pos: " << pos_and_slope[0] << "\tr_pos: " << pos_and_slope[1] << "\tspeed: " << speed << "\tangle: " << steer_angle << "\n";
    }

    return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  frame = cv_ptr->image;
}

std::vector<float> process_image()
{
    std::vector<float> pos_and_slope;
    // gray
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // canny edge
    cv::Mat canny;
    cv::Canny(gray, canny, low_threshold, high_threshold);

    cv::Mat roi = canny(cv::Rect(0 , Offset, Width, Gap));

    // HoughLinesP
    std::vector<cv::Vec4i>all_lines;
    cv::HoughLinesP(roi, all_lines, 1, M_PI/180, 40, 35, 10);
    
    // divide left, right lines
    if (all_lines.size() == 0) {
        for (int i = 0; i < 4; ++i) pos_and_slope.push_back(0.0);
        return pos_and_slope;
    }
    std::vector<cv::Vec4i> left_lines, right_lines;
    left_lines = divide_left_right(all_lines).first;
    right_lines = divide_left_right(all_lines).second;

    // Draw line
    if (show_img == true) {
        frame.copyTo(show);
        draw_lines(left_lines);
        draw_lines(right_lines);
    }

    // get center of lines
    int lpos, rpos;
    float l_slope, r_slope;
    lpos = get_line_pos(left_lines, true, false).first;
    rpos = get_line_pos(right_lines, false, true).first;
    l_slope = get_line_pos(left_lines, true, false).second;
    r_slope = get_line_pos(right_lines, false, true).second;

    pos_and_slope.push_back(static_cast<float>(lpos));
    pos_and_slope.push_back(static_cast<float>(rpos));
    pos_and_slope.push_back(l_slope);
    pos_and_slope.push_back(r_slope);
    return pos_and_slope;
}

std::pair<std::vector<cv::Vec4i>, std::vector<cv::Vec4i>> divide_left_right(std::vector<cv::Vec4i> &lines)
{
    std::vector<cv::Vec4i> new_lines;
    std::vector<float> slopes;
    int x1, y1, x2, y2;
    float slope;
    for(auto &line : lines) {
        x1 = line[0], y1 = line[1];
        x2 = line[2], y2 = line[3];
        if (x2 - x1 == 0) {
            slope = 0.0;
        }
        else {
            slope = static_cast<float>(y2 - y1) / static_cast<float>(x2 - x1);
        }

        if (std::abs(slope) > low_slope_threshold && std::abs(slope) < high_slope_threshold) {
            slopes.push_back(slope);
            new_lines.push_back(line);
        }
    }

    // divide lines left to right
    std::vector<cv::Vec4i> left_lines, right_lines;
    float left_x_sum = 0.0, right_x_sum = 0.0;
    float left_x_avg, right_x_avg;
    cv::Vec4i line;
    for (int j = 0; j < slopes.size(); ++j) {
        line = new_lines[j];
        slope = slopes[j];
        x1 = line[0], y1 = line[1];
        x2 = line[2], y2 = line[3];
        if (slope < 0) {
            left_lines.push_back(line);
            left_x_sum += static_cast<float>(x1 + x2) * 0.5;
        }
        else {
            right_lines.push_back(line);
            right_x_sum += static_cast<float>(x1 + x2) * 0.5;
        }
    }

    if (left_lines.size() != 0 && right_lines.size() != 0) {
        left_x_avg = left_x_sum / left_lines.size();
        right_x_avg = right_x_sum / right_lines.size();
        if (left_x_avg > right_x_avg) {
            left_lines.clear();
            right_lines.clear();
            std::cout << "Invalide Path!" << "\n";
        }
    }
    std::pair<std::vector<cv::Vec4i>, std::vector<cv::Vec4i>> left_right_lines(left_lines, right_lines);
    return left_right_lines;
}

std::pair<int, float> get_line_pos(std::vector<cv::Vec4i> &lines, bool left, bool right)
{   
    float pos;

    std::pair<float, float> m_and_b = get_line_params(lines);
    float m = m_and_b.first, b = m_and_b.second;

    float y;
    if (m == 0.0 && b == 0.0) {
        if (left == true) {
            pos = 0.0;
        }
        else {
            pos = static_cast<float>(Width);
        }
    }
    else {
        y = static_cast<float>(Gap) * 0.5;
        pos = (y - b) / m;
    }
    std::pair<int, float> pos_and_m(static_cast<int>(pos), m);
    return pos_and_m;
}

std::pair<float, float> get_line_params(std::vector<cv::Vec4i> &lines)
{
    std::pair<float, float> m_and_b;
    float x_sum = 0.0, y_sum = 0.0, m_sum = 0.0;
    int size = lines.size();
    if (size == 0) {
        m_and_b.first = 0.0, m_and_b.second = 0.0;
        return m_and_b;
    }

    int x1, y1, x2, y2;
    for (auto &line : lines) {
        x1 = line[0], y1 = line[1];
        x2 = line[2], y2 = line[3];

        x_sum += x1 + x2;
        y_sum += y1 + y2;
        m_sum += static_cast<float>(y2 - y1) / static_cast<float>(x2 - x1);
    }

    float x_avg, y_avg, m, b;
    x_avg = x_sum / static_cast<float>(size * 2);
    y_avg = y_sum / static_cast<float>(size * 2);
    m = m_sum / static_cast<float>(size);
    b = y_avg - m * x_avg;

    m_and_b.first = m, m_and_b.second = b;
    return m_and_b;
}

void draw_lines(std::vector<cv::Vec4i> &lines)
{
    cv::Point2i pt1, pt2;
    cv::Scalar color;

    for (auto &line : lines) {
        pt1 = cv::Point2i(line[0], line[1] + Offset);
        pt2 = cv::Point2i(line[2], line[3] + Offset);
        int r, g, b;
        r = static_cast<int>(static_cast<float>(std::rand())/RAND_MAX*255);
        g = static_cast<int>(static_cast<float>(std::rand())/RAND_MAX*255);
        b = static_cast<int>(static_cast<float>(std::rand())/RAND_MAX*255);
        color = cv::Scalar(b, g, r);
        
        cv::line(show, pt1, pt2, color, 2);
    }
}

void draw_rectangles(int &lpos, int &rpos, int &ma_mpos)
{
    cv::rectangle(show, cv::Point(lpos - 5, 15 + Offset), cv::Point(lpos + 5, 25 + Offset), cv::Scalar(0, 255, 0), 2);
    cv::rectangle(show, cv::Point(rpos - 5, 15 + Offset), cv::Point(rpos + 5, 25 + Offset), cv::Scalar(0, 255, 0), 2);
    cv::rectangle(show, cv::Point(ma_mpos-5, 15 + Offset), cv::Point(ma_mpos+5, 25 + Offset), cv::Scalar(0, 255, 0), 2);
    cv::rectangle(show, cv::Point(315, 15 + Offset), cv::Point(325, 25 + Offset), cv::Scalar(0, 0, 255), 2);
}

void velocity_control(float &angle)
{
    if (std::abs(angle) > 30) {
        speed -= 0.1;
        speed = std::max(speed, 15.0f);
    }
    else {
        speed += 0.05;
        speed = std::min(speed, 50.0f);
    }
}

void drive(ros::Publisher &pub, float &angle, float &speed)
{
    xycar_msgs::xycar_motor msg;
    msg.angle = std::round(angle);
    msg.speed = std::round(speed);


    pub.publish(msg);
}