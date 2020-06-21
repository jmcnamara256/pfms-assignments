#include "prminterface.h"
#include "ros/package.h"
#include "geometry_msgs/PoseArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <memory>
#include <chrono>

namespace enc = sensor_msgs::image_encodings;

/**
 * @brief Construct a new Prm Interface
 * 
 * Initialise subscribers and publishers.
 * Gets parameters from the parameter server and nodehandle.
 * 
 * @param nh 
 */
PrmInterface::PrmInterface(ros::NodeHandle nh) : nh_(nh), it_(nh)
{
    image_transport::ImageTransport it(nh);
    
    /**
     * Subscribers
     * Odom - current robot position
     * Image - the OG map is converted to a cv image because this is easy to manipulate
     * 
     * Publishers
     * Image - publishes the map with PRM info overlayed
     * 
     * Services
     * Resolution - get the resolution from the parameter server
    **/
    odom_sub_ = nh_.subscribe("robot_0/odom", 1000, &PrmInterface::odomCallback, this);
    img_sub_ = it.subscribe("map_image/full", 1, &PrmInterface::imageCallback, this);
    goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &PrmInterface::goalCallback,this);

    image_pub_ = it_.advertise("/test/image", 1);
    path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/path", 1);

    service_ = nh_.advertiseService("request_goal", &PrmInterface::requestGoal, this);

    // Get the resolution from the parameter server
    double res;
    nh_.getParam("/local_map/map_resolution", res);
    resolution_ = res;

    // Get the maximum distance nodes can make a connection.
    // Defaults to 600 - this is the squared distance in pixels
    int max_connect_dist;
    ros::NodeHandle ph("~");
    ph.param<int>("dist", max_connect_dist, 600);

    /**
     * Initialise a new PRM object.
     * This does the bulk of the PRM processing
    **/
    prm_ptr = std::unique_ptr<Prm>(new Prm);
    prm_ptr->setMaxConnectDist(max_connect_dist);


    // TESTING OTHER IMAGES
    /*
    std::string path = ros::package::getPath("prm");
    path += "/maps/";
    std::string file = path + "narrow.png";
    img_buffer_.og_map = cv::imread(file , 0);
    */

    // These flags will be false until a request is made
    new_request = false;
    time_to_draw = false;
}


/**
 * @brief Callback for goal request service. Returns true if the goal is valid, and
 * prm will begin.
 * 
 * @param req The requested goal pose.
 * @param res Boolean reply, true if valid goal.
 * @return true 
 * @return false 
 */
bool PrmInterface::requestGoal(a4_setup::RequestGoal::Request & req, a4_setup::RequestGoal::Response & res)
{
    // Extract the x,y position from the request (a pose2d)
    geometry_msgs::Pose pose;
    pose.position.x = req.pose.x;
    pose.position.y = req.pose.y;
    
    // Get the orientation of the goal, convert to Quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, req.pose.theta);
    q.normalize();
    pose.orientation = tf2::toMsg(q);

    // Check if the input is valid, and start PRM
    if( processInput(pose) )
    {
        res.ack = true;
    }
    else
    {
        res.ack = false;
    }
    // The service returned successfully
    return true;    
}


/**
 * @brief  Handles new goal requests, since there are multiple input pathways.
 * Both the request_goal service and the move_base_simple topic pass the goal
 * pose to this function. 
 * 
 * Checks if the goal is valid, if so, sets the flag to begin prm creation.
 * 
 * @param req Goal pose
 * @return true 
 * @return false 
 */
bool PrmInterface::processInput(geometry_msgs::Pose req)
{
    // Working data
    int px,py;
    cv::Mat image;
    geometry_msgs::Pose pose;
    bool result = false;
    int sx, sy;
            
    
    // Get a local copy of the most current map/pose
    {
        std::lock_guard<std::mutex> lock_i(img_buffer_.mtx);
        image = img_buffer_.og_map.clone();
    }
    {
        std::lock_guard<std::mutex> lock(pose_buffer_.mtx);
        pose = pose_buffer_.pose;
    }
    globalToPixel(req.position.x, req.position.y, px, py);
    globalToPixel(pose.position.x, pose.position.y, sx, sy);    // This should always be 100,100 since the OG map is centered

    // Check if request is within og_map, and free space
    if (px > 0 && px < image.cols &&
        py > 0 && py < image.rows) 
    {
        // Now check the pixel value at the request location
        if(image.at<uint8_t >(py, px)  == (int)State::FREE)
        {
            std::lock_guard<std::mutex> lock(path_buffer_.mtx);
            // This returns true if the images are different
            if( !path_buffer_.working_img.empty() &&
                (pose.position.x != path_buffer_.start_pose.position.x ||
                 pose.position.y != path_buffer_.start_pose.position.y) )
            {
                ROS_INFO_STREAM("Aborting Prm and starting new!");
                // Reassignment deletes old object
                // Start a new prm
                // Abort blocks until any current prm operations finish.
                prm_ptr->abort();
                prm_ptr = std::unique_ptr<Prm>(new Prm);
            }
            // Set the new_request flag so the Prm is initiated
        
            path_buffer_.working_img    = image.clone();    // map
            path_buffer_.working_start  = Point(sx, sy);    // start pixel coord
            path_buffer_.working_goal   = Point(px, py);    // goal pixel coord
            path_buffer_.start_pose     = pose;             // start pose as a global coord. Used to compare if the start pose has changed
            path_buffer_.goal_pose      = req;              // goal pose, keep to get the final orientation later
            new_request = true;
            return true;
        }        
    }
    return false;
}


/**
 * @brief Callback for odometry messages, from stage.
 * The local copy is updated.
 * 
 * @param msg 
 */
void PrmInterface::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(pose_buffer_.mtx);

    pose_buffer_.pose = msg->pose.pose;
}


/**
 * @brief Callback for the move_base_simple goal request messages.
 * This callback invokes the processInput function, which handles
 * input for both sources of goal request.
 * 
 * @param msg  Pointer to the pose message sent.
 */
void PrmInterface::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    processInput(msg->pose);
}


/**
 * @brief Callback for image messages, which are derived from the laserscan.
 * These are converted to the robot configuration space by inflating the walls slightly.
 * The level of inflation is based on the robot radius.
 * A local copy of this image is stored.
 * 
 * @param msg Pointer to the image message sent.
 */
void PrmInterface::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    //! Below code converts from sensor message to a pointer to an opencv image and time to a deque
    // to share across threads
    try
    {
        if (enc::isColor(msg->encoding))
            cvPtr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
        else
            cvPtr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    // Convert scan to config space
    cv::Mat image = cvPtr_->image;
    cv::Mat config_space = scanToConfigSpace(image);
    
    // Store it 
    std::lock_guard<std::mutex> lock(img_buffer_.mtx);
    img_buffer_.og_map = config_space.clone();

}


/**
 * @brief This thread runs in a loop.
 * Its role is to publish an image with path and map info drawn on top.
 */
void PrmInterface::drawPathThread()
{
    cv_bridge::CvImage image;

    ros::Rate rate_limiter(10.0);
    while (ros::ok())
    {  
        // True if prm found a valid path
        if( time_to_draw )
        {
            {   // Get the image from the img_buffer
                std::lock_guard<std::mutex> lock(img_buffer_.mtx);

                cv::cvtColor(img_buffer_.og_map, image.image, CV_GRAY2RGB);
            }

            // draw on top of the config space
            drawAllPoints(image);
            drawPathPoints(image);

            time_to_draw = false;
        }
       
        // Publish image
        image.encoding = "bgr8";
        image.header = std_msgs::Header();
        image_pub_.publish(image.toImageMsg());

        // Frequency
        rate_limiter.sleep();
    }
}


/**
 * @brief Runs continuously. If a goal request is made, a flag is set. This thread
 * then executes the PRM generation and sets the path in the path buffer when done.
 * 
 * This is threaded so that other operations are not blocked while generating the PRM.
 * 
 */
void PrmInterface::prmProcessingThread()
{
    // Loop 10Hz
    ros::Rate rate(10);
    while( ros::ok() )
    {
        if(new_request)
        {
            /**
             * Lock the mutex for the path buffer to get a copy
             */
            std::unique_lock<std::mutex> lock(path_buffer_.mtx);

            /**
             * Prm needs 3 things to run. Start, end, map.
             */
            Point start = path_buffer_.working_start;
            Point goal = path_buffer_.working_goal;
            cv::Mat map = path_buffer_.working_img.clone();

            /**
             * @Leave the mutex unlocked while the function runs
             * so that the callback can still update the request 
             */
            lock.unlock();

            /**
             * This function runs the PRM. If the map is very large >10^6 pixels, this can take a while.
             * To stop this function from running indefinately, a maximum number of nodes is set in the prm.h file.
             * 
             * If the max nodes is reached, but the start/end are not connected this will return an empty path.
             */
            auto path = prm_ptr->planPath(start, goal, map);

            /**
             * IO about the path. It seemed OK to put IO in this class, since it's
             * purpose is to be a communication interface for the actual PRM process.
             */
            ROS_INFO_STREAM("Path Length: " << path.size() << "  Graph Nodes: " << prm_ptr->getAllNodes().size());
        
            /**
             * Create the pose_array message to be published. 
             */
            geometry_msgs::PoseArray posearray_msg;
            posearray_msg.header.stamp = ros::Time::now();
            for(auto point : path)
            {
                geometry_msgs::Pose p;
                double x, y;

                // Convert from pixel->global for the message
                pixelToGlobal(point.x, point.y, x, y);
                p.position.x = x;
                p.position.y = y;
                posearray_msg.poses.push_back(p);
            }

            /**
             * If the path is empty then PRM failed.
             * In this case dont draw.
             * 
             * Set the new_request flag to false so the callbacks know its done.
             * Set the orientation of the last pose to the requested orientation.
             */
            if( !path.empty() )
            {
                lock.lock();
                path_buffer_.path = path;
                posearray_msg.poses.back().orientation = path_buffer_.goal_pose.orientation;
                lock.unlock();
                time_to_draw = true;
            }
            else
            {
                ROS_INFO_STREAM("Prm failed!");
            }
            
            // We have dealt with this request now
            new_request = false;

            // publish
            path_pub_.publish(posearray_msg);
        }
        

        rate.sleep();
    }
}


/**
 * @brief A helper function to convert the map image to configuration space.
 * This is done by inflating the walls by the radius of the robot.
 * 
 * @param image Image to convert
 * @return cv::Mat 
 */
cv::Mat PrmInterface::scanToConfigSpace(cv::Mat image)
{
    cv::Mat config_space = image.clone();   // Make a copy to work with

    // Convert radius to pixels
    int px_buffer = double(robot_radius_ / resolution_);

    for (int i = 0; i<image.rows; i++)  // row
    {
        for (int j = 0; j <image.cols; j++) // col
        {
            uint8_t &pixel = image.at<uint8_t>(i, j);   // opencv is row-major

            // If the pixel is unknown or obstacle it should be marked as occupied
            if(pixel != (uint8_t)State::FREE)
            {
                config_space.at<uint8_t>(i, j) = 0;
                continue;
            }

            // Check each free space for its distance to obstacle
            else
            {
                // Only check the local pixels, dont look at the whole image again
                for(int dx = -px_buffer; dx <= px_buffer; dx++)
                {
                    for(int dy = -px_buffer; dy <= px_buffer; dy++)
                    {
                        // If the nearby pixel is obstacle
                        uint8_t &pixel_test = image.at<uint8_t>(i + dy,j + dx);
                        if(pixel_test != (uint8_t)State::FREE)
                        {
                            
                            // If its closer than the robot radius (in pixels)
                            int dist = pow(dx, 2) + pow (dy, 2);
                            if(dist > pow(px_buffer, 2))
                            {
                                // Set pixel as occupied
                                config_space.at<uint8_t>(i, j) = 0;
                                goto pixel_tested;
                            }
                        }
                    } // py
                } // px

                pixel_tested:;
            }   // free
        } // cols
    } // rows
    return config_space;
}


/**
 * @brief Draws the points contained in the path returned by
 * the prm process.
 * 
 * Start/end points are green, other points are red.
 * 
 * @param image The image to draw the points on
 */
void PrmInterface::drawPathPoints(cv_bridge::CvImage &image)
{
    int x;
    int y;
    int prev_x;
    int prev_y;

    path_buffer_.mtx.lock();
    std::vector<Point> path = path_buffer_.path;
    path_buffer_.mtx.unlock();

    /**
     * If the PRM fails, or reaches max number of nodes the path will be empty.
     */
    if(path.empty())
    {
        return;
    }

    // The previous point at the start is just the start
    //globalToPixel(path.front().position.x, path.front().position.y, prev_x, prev_y);
    prev_x = path.front().x;
    prev_y = path.front().y;

    // For each point along the path
    for(auto point : path)
    {
        //globalToPixel(point.position.x, point.position.y, x, y);
        x = point.x;
        y = point.y;
        cv::Point pt(x, y);
        cv::Point prev_pt(prev_x, prev_y);

        // The connection line
        cv::line(image.image, pt, prev_pt, CV_RGB(220, 20, 20), 2, 8, 0);


        // Draw node point, start/end get big points
        if(point == path.front() || point ==  path.back())
        {
            cv::circle(image.image, pt, 8, CV_RGB(0, 255, 0), CV_FILLED, 8, 0);
        }
        else
        {
            cv::circle(image.image, pt, 4, CV_RGB(255, 0, 0), CV_FILLED, 8, 0);
        }
        prev_x = x;
        prev_y = y;
    }
}


/**
 * @brief Draws all the points contained within the prm graph.
 * Nodes are shown in blue, connections as grey lines.
 * 
 * @param image The image to draw the points onto
 */
void PrmInterface::drawAllPoints(cv_bridge::CvImage &image)
{
    // For each node in the graph
    for(auto node_ptr : prm_ptr->getAllNodes())
    {
        int x = node_ptr->data.x;
        int y = node_ptr->data.y;

        // Draw all its connections
        for(auto connection : node_ptr->getEdges())
        {
            int nx = connection.node->data.x;
            int ny = connection.node->data.y;

            cv::Point p1(x, y);
            cv::Point p2(nx, ny);

            // They are being drawn very small so that its easy to see the path
            cv::line(image.image, p1, p2, CV_RGB(80, 80, 80), 1, 8, 0);
            cv::circle(image.image, p1, 2, CV_RGB(0, 0, 255), 1);
        }
    }
}


/**
 * @brief Converts a pixel location to a global location. The conversion uses the current map
 * and pose. These are stored in the pose_buffer_ and img_buffer_ respectively.
 * 
 * @param x Pixel x
 * @param y Pixel y
 * @param gx Global x
 * @param gy Global y
 */
void PrmInterface::pixelToGlobal(int x, int y, double &gx, double &gy)
{
    std::lock(pose_buffer_.mtx, img_buffer_.mtx);
    std::lock_guard<std::mutex> lock_pose(pose_buffer_.mtx, std::adopt_lock);
    std::lock_guard<std::mutex> lock_img(img_buffer_.mtx, std::adopt_lock);

    gx = pose_buffer_.pose.position.x + (x - img_buffer_.og_map.cols/2) * resolution_;
    gy = pose_buffer_.pose.position.y + (y - img_buffer_.og_map.rows/2) * resolution_;
}


/**
 * @brief Converts a global location to a pixel location. The conversion uses the current map
 * and pose. These are stored in the pose_buffer_ and img_buffer_ respectively.
 * 
 * @param x Global x
 * @param y Global y
 * @param px Pixel x
 * @param py Pixel y
 */
void PrmInterface::globalToPixel(double x, double y, int &px, int &py)
{
    std::lock(pose_buffer_.mtx, img_buffer_.mtx);
    std::lock_guard<std::mutex> lock_pose(pose_buffer_.mtx, std::adopt_lock);
    std::lock_guard<std::mutex> lock_img(img_buffer_.mtx, std::adopt_lock);

    px = img_buffer_.og_map.cols/2 + (x - pose_buffer_.pose.position.x) / resolution_;
    py = img_buffer_.og_map.rows/2 - (y - pose_buffer_.pose.position.y) / resolution_;
}

