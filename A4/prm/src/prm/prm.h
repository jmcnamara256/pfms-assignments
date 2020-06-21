#ifndef PRM_H
#define PRM_H

#include "graph/graph.h"
#include "point.h"           // 2D vector struct
#include "cv_bridge/cv_bridge.h"    // for the map


typedef graph::Graph<Point, int> GraphP;
typedef graph::Node<Point, int> NodeP;

// Pixel state in map
// Black is OCCUPIED
// Grey is UNKNOWN
// White is free
enum class State
{
    FREE = 255,
    UNKNOWN = 127,
    OCCUPIED = 0
};

/**
 * @brief Creates a probabilistic roadmap from the 2 points, and a black/white image.
 * Once connectivity between the 2 points is achieved by adding random nodes, a shortest route
 * is planned and the nodes of the route are returned.
 * 
 * Because creating a PRM for large images can take a long time, the abort function is uncluded.
 * Calling abort will set a stop flag internally, and the process will return once the next
 * checkpoint is reached. Images < 10^5 pixels are computed quickly < 2s generally.
 * Tests on images with 10^6 pixels can take up to 30s.
 * 
 * Although this function is intended to be coupled with ROS, in order to keep it modular,
 * no ROS datatypes were used. It requires only a CV image, and 2 points.
 * 
 */
class Prm
{
    public:
        // constructors
        Prm(){use_bridges_ = true;};
        Prm(bool use_bridges) : use_bridges_(use_bridges){};
        // Public functions
        std::vector<Point> planPath(Point start, Point goal, cv::Mat map);
        std::vector<NodeP*> getAllNodes();
        void setMaxConnectDist(int);
        bool abort();
        
    private:
        // Helper functions
        int     distanceSquared(Point, Point);         // Computes the squared distance between 2 points
        bool    inBounds(Point);                       // Check a point is in the image
        bool    lineOfSight(Point, Point);             // Check a line can be drawn between 2 points without passing over any obstacles
        bool    checkSteepLine(int, int, int, int);    // LOS check for steep lines slope > 1
        bool    checkShallowLine(int, int, int, int);  // LOS check for shallow lines slope < 1
        bool    free(int, int);                        // Check a point on the image is free
        bool    populateMap();                         // Populates the map with nodes and adds them to the graph
        void    connectNode(Point);                    // Connects a node to its neighbours
        Point   midpoint(Point, Point);                // Computes the midpoint of 2 points

        // Data
        GraphP              graph;
        cv::Mat             img;
        std::atomic<bool>   prm_ok;
        std::atomic<bool>   running;
        bool                use_bridges_;
        int                 max_connect_dist    = 1200;
        const int           max_nodes           = 5000;
        const int           max_bridge_length   = 40;
        const int           max_connections     = 5;
        const int           normal_bridge_ratio = 6;

        Point start;
        Point goal;
};


#endif // PRM_H