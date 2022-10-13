#include "cpp_drive/module.h"

//------------------------------------------
//             Functions 
//------------------------------------------
// void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void imageCallback(const sensor_msgs::Image& msg);
std::vector<float> process_image();
std::pair<std::vector<cv::Vec4i>, std::vector<cv::Vec4i>> divide_left_right(std::vector<cv::Vec4i> &lines);
std::pair<int, float> get_line_pos(std::vector<cv::Vec4i> &lines, bool left, bool right);
std::pair<float, float> get_line_params(std::vector<cv::Vec4i> &lines);
void draw_lines(std::vector<cv::Vec4i> &lines);
void draw_rectangles(int &lpos, int &rpos, int &ma_mpos);
void velocity_control(float &angle);
void drive(ros::Publisher &pub, float &angle, float &speed);
//------------------------------------------
//           Global variables
//------------------------------------------
// cv_bridge::CvImagePtr cv_ptr;
cv::Mat frame, show;            // frame: cv image, show: visualization image
int low_threshold = 150;        // canny edge low threshold
int high_threshold = 250;       // canny edge high threshold
int Width = 640;                // Image width
int Height = 480;               // Image Height
int Offset = 340;               // Region of Interest (ROI) Box y1
int Gap = 40;                   // Region of Interest (ROI) Box y2 = y1 + gap
float slope_range = 10.0f;      // -10 <= slope range <= 10
const int sampling_number = 20; // sampling number of Moving Average filter
float speed = 0.0;              // Initial speed of Xycar
float max_speed = 40.0f;        // Max speed of Xycar
float min_speed = 15.0f;        // Min speed of Xycar
bool show_img = true;           // Visualiation mode (ON / OFF)
cv::Scalar cv_red(0, 0, 255);   // OpenCV Color Red
cv::Scalar cv_green(0, 255, 0); // OpenCV Color Green
cv::Scalar cv_blue(255, 0, 0);  // OpenCV Color Blue

//------------------------------------------
//             Main Function
//------------------------------------------
int main(int argc, char** argv) {

    ros::init(argc, argv, "test");      // ROS init function
    ros::NodeHandle nh;                 // ROS nodehandle funtion 
    ros::Publisher pub =                // ROS Publischer Node
        nh.advertise<xycar_msgs::xycar_motor>("xycar_motor", 1000);
    ros::Subscriber sub =               // ROS Subscriber Node
        nh.subscribe("/usb_cam/image_raw/", 1, imageCallback);
    

    MovingAverage ma(sampling_number);      // Moving Average Filter Class
    float kp, ki, kd;                       // PID gain Setting
    ros::param::get("kp", kp);              // Get P gain from launch argument
    ros::param::get("ki", ki);              // Get I gain from launch argument
    ros::param::get("kd", kd);              // Get D gain from launch argument
    ros::param::get("show_img", show_img);  // Get Visualization mode from launch argument

    std::cout << "Kp: " << kp               // Print PID Gain and Visualization mode
              << "\tKi: " << ki
              << "\tKd: " << kd
              << "\tshow_img: " << show_img << std::endl;

    PID pid(kp, ki, kd);                    // PID Class

    int lpos, rpos,                         // lpos : Left Lane position, rpos : Right Lane position
        mpos, ma_pos, cte;                  // mpos : middle position of the Lanes, ma_pos : Filtered mpos
    float l_slope, r_slope,                 // l_slope : slope of Left Lane, r_slope : slope of Right Lane, 
        steer_angle;                        // Steering angle Value

    // Driving Start (Get Camera Image)    
    while (ros::ok()) {
        ros::spinOnce();                                    // Get Camera frame once

        if (frame.cols != 640) {                            // Wait for camera image to be obtained
            continue;
        }

        // Step 1: Image Processing
        std::vector<float> pos_and_slope = process_image(); // Get each Position & Slope of the Left and Right Lanes
        lpos = static_cast<int>(pos_and_slope[0]);
        rpos = static_cast<int>(pos_and_slope[1]);
        l_slope = pos_and_slope[2];
        r_slope = pos_and_slope[3];

        // Step 2: Calculate the center of the Lanes
        mpos = (lpos + rpos) * 0.5;                         // Mid = (Left + Right) / 2
        ma.add_sample(mpos);                                // Middle Position Filtering (1/2)
        ma_pos = ma.get_wmm();                              // Middle Position Filtering (2/2)

        // Step 3: Get the error of PID control
        cte = ma_pos - Width * 0.5;                         // e = current middle lane position - Center of the image

        // Step 4: Get the optimal steering angle with PID Controller
        steer_angle = pid.pid_control(cte);
        steer_angle = std::max(-50.0f,                      // -50 <= angle <= +50
            std::min(steer_angle, 50.0f));

        // Step 5: Get the optimal Xycar speed with steering angle
        velocity_control(steer_angle);

        // Step 6: Publish the Speed and Steering angle 
        drive(pub, steer_angle, speed);

        // Visualization
        if (show_img == true) {
            draw_rectangles(lpos, rpos, ma_pos);
            cv::imshow("show", show);
            cv::waitKey(1);
        }

        // std::cout << "l_pos: " << pos_and_slope[0] << "\tr_pos: " << pos_and_slope[1] << "\tspeed: " << speed << "\tangle: " << steer_angle << "\n";
    }

    return 0;
}

//------------------------------------------
//      Image Callback Function
//------------------------------------------
void imageCallback(const sensor_msgs::Image& msg)
{
    // Get Image pointer
//     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // Update the image
//     frame = cv_ptr->image;
    cv::Mat src = cv::Mat(480, 640, CV_8UC3, const_cast<uchar *>(&msg.data[0]), msg.step);
    cv::cvtColor(src, frame, cv::COLOR_RGB2BGR);
}

//------------------------------------------
//      Image Processing Function
//------------------------------------------
std::vector<float> process_image()
{
    std::vector<float> pos_and_slope;
    // Make gray Image for Canny Edge Detection
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // Get ROI Canny Edge image
    cv::Mat canny;
    cv::Canny(gray, canny, low_threshold, high_threshold);
    cv::Mat roi = canny(cv::Rect(0 , Offset, Width, Gap));

    // Get lines with HoughLinesP funtion
    std::vector<cv::Vec4i> all_lines;
    cv::HoughLinesP(roi, all_lines, 1, M_PI/180, 40, 35, 10);
    if (all_lines.size() == 0) {                                    // If there is not line, return zero vector. 
        return std::vector<float>(4, 0.0f);
    }

    // Divide left and right lines
    std::pair<std::vector<cv::Vec4i>, std::vector<cv::Vec4i>> left_right_lines = divide_left_right(all_lines);
    std::vector<cv::Vec4i> left_lines = left_right_lines.first;
    std::vector<cv::Vec4i> right_lines = left_right_lines.second;

    // Draw lines for visualization
    if (show_img == true) {
        frame.copyTo(show);
        draw_lines(left_lines);
        draw_lines(right_lines);
    }

    // Get postion and slope values
    std::pair<int, float> lpos_lslope = get_line_pos(left_lines, true, false);
    std::pair<int, float> rpos_rslope = get_line_pos(right_lines, false, true);
    int lpos = lpos_lslope.first;
    int rpos = rpos_rslope.first;
    float l_slope = lpos_lslope.second;
    float r_slope = rpos_rslope.second;

    pos_and_slope.push_back(static_cast<float>(lpos));
    pos_and_slope.push_back(static_cast<float>(rpos));
    pos_and_slope.push_back(l_slope);
    pos_and_slope.push_back(r_slope);
    return pos_and_slope;
}

//------------------------------------------
//      Line Division Function
//------------------------------------------
std::pair<std::vector<cv::Vec4i>, std::vector<cv::Vec4i>> divide_left_right(std::vector<cv::Vec4i> &lines)
{
    // Step 1. Filtering the lines with slope value
    std::vector<cv::Vec4i> new_lines;
    std::vector<float> slopes;
    int x1, y1, x2, y2;
    float slope;

    // Get x,y values from lines
    for(auto &line : lines) {
        x1 = line[0], y1 = line[1];
        x2 = line[2], y2 = line[3];
        if (x2 - x1 == 0) {                 // If x1 = x2, slope = 0.
            slope = 0.0;
        }
        else {                              // Else, slope = (y2 - y1) / (x2 - x1) 
            slope = static_cast<float>(y2 - y1) / static_cast<float>(x2 - x1);
        }

        if (std::abs(slope) <= slope_range) {// If -10 <= slope <= 10, push back.
            slopes.push_back(slope);
            new_lines.push_back(line);
        }
    }

    // Step 2. Divide left and right lines
    std::vector<cv::Vec4i> left_lines, right_lines;
    float left_x_sum = 0.0, right_x_sum = 0.0;
    float left_x_avg, right_x_avg;
    cv::Vec4i line;
    
    //Get x, y values from filtered line (new_lines) and Divide the lines.
    for (int j = 0; j < new_lines.size(); ++j) {
        line = new_lines[j];
        slope = slopes[j];
        x1 = line[0], y1 = line[1];
        x2 = line[2], y2 = line[3];

        // If slope < 0 (line shape : /), left line. 
        if (slope < 0) {
            left_lines.push_back(line);
            left_x_sum += static_cast<float>(x1 + x2) * 0.5;
        }
        // If slope >= 0 (line shape : \), right line.
        else {
            right_lines.push_back(line);
            right_x_sum += static_cast<float>(x1 + x2) * 0.5;
        }
    }

    // Step 3. Exception handling (No line)
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

//------------------------------------------
//      Get Lane Position Function
//------------------------------------------
std::pair<int, float> get_line_pos(std::vector<cv::Vec4i> &lines, bool left, bool right)
{   
    // Step 1. get m & b values (y = mx + b)
    std::pair<float, float> m_and_b = get_line_params(lines);
    float m = m_and_b.first;
    float b = m_and_b.second;

    // Step 2. Get Lane position
    float y, pos;
    if (m == 0.0 && b == 0.0) {                 // If there is no line, pos = 0 or 640.
        if (left == true) {
            pos = 0.0;
        }
        else {
            pos = static_cast<float>(Width);
        }
    }
    else {                                      // Else, y = mx + b --> x = (y - b) / m
        y = static_cast<float>(Gap) * 0.5;
        pos = (y - b) / m;
    }
    std::pair<int, float> pos_and_m(static_cast<int>(pos), m);
    return pos_and_m;
}

//------------------------------------------
//      Get Line Parameter (m & b) Function
//------------------------------------------
std::pair<float, float> get_line_params(std::vector<cv::Vec4i> &lines)
{
    int size = lines.size();
    // Step 1. Exception handling (No lines)
    if (size == 0) {
        return std::pair<float, float>(0.0f, 0.0f);
    }

    // Step 2. Get sum(m), sum(x), and sum(y)
    int x1, y1, x2, y2;
    float x_sum = 0.0, y_sum = 0.0, m_sum = 0.0;
    for (auto &line : lines) {
        x1 = line[0], y1 = line[1];
        x2 = line[2], y2 = line[3];

        x_sum += x1 + x2;
        y_sum += y1 + y2;
        m_sum += static_cast<float>(y2 - y1) / static_cast<float>(x2 - x1);
    }

    // Step 3. Get m and b
    float x_avg, y_avg, m, b;
    x_avg = x_sum / static_cast<float>(size * 2);
    y_avg = y_sum / static_cast<float>(size * 2);
    m = m_sum / static_cast<float>(size);
    b = y_avg - m * x_avg;

    std::pair<float, float> m_and_b(m, b);
    return m_and_b;
}

//------------------------------------------
//      Line Drawing Function
//------------------------------------------
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

//------------------------------------------
//      Rectangle Drawing Function
//------------------------------------------
void draw_rectangles(int &lpos, int &rpos, int &ma_mpos)
{
    // lpos, rpos = Green, mpos = red, center = blue
    cv::rectangle(show, cv::Point(lpos - 5, 15 + Offset), cv::Point(lpos + 5, 25 + Offset), cv_green, 2);
    cv::rectangle(show, cv::Point(rpos - 5, 15 + Offset), cv::Point(rpos + 5, 25 + Offset), cv_green, 2);
    cv::rectangle(show, cv::Point(ma_mpos-5, 15 + Offset), cv::Point(ma_mpos+5, 25 + Offset), cv_red, 2);
    cv::rectangle(show, cv::Point(315, 15 + Offset), cv::Point(325, 25 + Offset), cv_blue, 2);
}

//------------------------------------------
//  Acceleration and Deceleration Function (Optional)
//------------------------------------------
void velocity_control(float &angle)
{
    if (std::abs(angle) > 30) {
        speed -= 0.1;
        speed = std::max(speed, min_speed);
    }
    else {
        speed += 0.05;
        speed = std::min(speed, max_speed);
    }
}

//------------------------------------------
//  Publish Function (Optional)
//------------------------------------------
void drive(ros::Publisher &pub, float &angle, float &speed)
{
    xycar_msgs::xycar_motor msg;
    msg.angle = std::round(angle);
    msg.speed = std::round(speed);

    pub.publish(msg);
}
