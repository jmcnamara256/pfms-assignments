#ifndef PRMINTERFACE_H
#define PRMINTERFACE_H

#include <mutex>
#include <memory>

#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "prm/prm.h"
#include "a4_setup/RequestGoal.h"


class PrmInterface{

public:
    PrmInterface(ros::NodeHandle nh);

    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    bool requestGoal(a4_setup::RequestGoal::Request & req, a4_setup::RequestGoal::Response & res);

    void drawPathThread();          // Main loop - runs seperate thread
    void prmProcessingThread();     // 

private:

    void pixelToGlobal(int x, int y, double &gx, double &gy);
    void globalToPixel(double x, double y, int &px, int &py);
    void drawPathPoints(cv_bridge::CvImage &image);
    void drawAllPoints(cv_bridge::CvImage &image);
    cv::Mat scanToConfigSpace(cv::Mat og_map);
    bool processInput(geometry_msgs::Pose);


    // Sub/Pub
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Publisher path_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber goal_sub_;
    image_transport::Subscriber img_sub_;
    cv_bridge::CvImagePtr cvPtr_;
    ros::ServiceServer service_;

    std::unique_ptr<Prm> prm_ptr;

    //Data
    const double robot_radius_ = 0.1;   // robot radius constant
    std::atomic<double> resolution_;    // meters / pixel
    std::atomic<bool> prm_running;         // True if a new goal is requested that is valid
    std::atomic<bool> new_request;
    std::atomic<bool> time_to_draw;

    // Threadsafe Data structures
    struct PoseBuffer
    {
        std::mutex mtx;
        geometry_msgs::Pose pose;
    }pose_buffer_;

    struct ImageBuffer
    {
        std::mutex mtx;
        cv::Mat og_map;
    }img_buffer_;

    struct PathBuffer
    {
        std::mutex mtx;
        std::vector<Point> path;
        geometry_msgs::Pose start_pose;
        geometry_msgs::Pose goal_pose;
        cv::Mat working_img;
        Point working_start;
        Point working_goal;
    }path_buffer_;
    
};


#endif // PRMINTERFACE_H