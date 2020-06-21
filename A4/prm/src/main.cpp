
#include "ros/ros.h"
#include "prminterface/prminterface.h"

#include <thread>

int main(int argc, char **argv)
{
    /*
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line. For programmatic
    * remappings you can use a different version of init() which takes remappings
    * directly, but for most command-line programs, passing argc and argv is the easiest
    * way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    *
    */
    ros::init(argc, argv, "prm");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle nh;

    /**
    * Create an object of type PfmsSample  and pass it a node handle
    */
    std::shared_ptr<PrmInterface> prm_ptr(new PrmInterface(nh));

    /**
     * Starts the main program loop for the interface class. Primarily this handles
     * drawing the path and prm nodes once the process is complete.
     * 
     * In order to begin the prm process, a goal must be requested. The two ways to do this are via
     * the ROS service "request_goal".
     * Alternatively a pose can be published via nav_goals
     */
    std::thread t(&PrmInterface::drawPathThread, prm_ptr);
    std::thread tp(&PrmInterface::prmProcessingThread, prm_ptr);

    ros::spin();

    /**
    * Let's cleanup everything, shutdown ros and join the thread
    */
    ros::shutdown();
    t.join();
    tp.join();

    return 0;
}
