#include "prm.h"

#include <random>
#include <chrono>   // for random number seed

/**
 * @brief Returns true if two points can be connected by a line without
 * passing over any occupied pixels.
 * 
 * @param p1 Start Point
 * @param p2 End Point
 * @return true 
 * @return false 
 */
bool Prm::lineOfSight(Point p1, Point p2)
{
    int x0 = p1.x;
    int y0 = p1.y;
    int x1 = p2.x;
    int y1 = p2.y;

    if(x0 == x1)    // Vertical line
    {
        int ymin = (y0 < y1) ? y0 : y1;     // The smaller one
        int ymax = (ymin == y0) ? y1 : y0;  // The larger one

        for(; ymin < ymax; ymin++)
        {
            if( !free(x0, ymin) )
            {
                return false;
            }
        }
    }
    else if(abs(y1 - y0) < abs(x1 - x0))    // Slope is shallow -1 < m < 1
    {
        if(x0 > x1)     // reverse end-points if necessary
        {
            return checkShallowLine(x1, y1, x0, y0);
        }
        else
        {
            return checkShallowLine(x0, y0, x1, y1);
        }
    }
    else    // Slope is steep -1 > m || m > 1
    {
        if(y0 > y1)     // reverse end-points if necessary
        {
            return checkSteepLine(x1, y1, x0, y0);
        }
        else
        {
            return checkSteepLine(x0, y0, x1, y1);
        }
    }
}


/**
 * @brief Returns true if two points can be connected by a line without
 * passing over any occupied pixels. This is called in the case of a 
 * steep line: slope > 1.
 * 
 * @param x0 Start x coord
 * @param y0 Start y coord
 * @param x1 End x coord
 * @param y1 End y coord
 * @return true 
 * @return false 
 */
bool Prm::checkSteepLine(int x0, int y0, int x1, int y1)
{
    int dx = x1 - x0;
    int dy = y1 - y0;
    int xi = 1;     // increment

    if(dx < 0)
    {
        xi = -1;    // negative increment
        dx = -dx;
    }

    int err = 2*dx - dy;
    int x = x0;

    for(int y = y0; y <= y1; y++)   // increment y by 1px every loop
    {
        if( !free(x, y) )           // check coord
        {
            return false;
        }

        if(err > 0)                 // increment error
        {
            x = x + xi;
            err -= 2*dy;
        }
        err += 2*dx;
    }
    return true;
}


/**
 * @brief Returns true if two points can be connected by a line without
 * passing over any occupied pixels. This is called in the case of a 
 * shallow line: slope < 1. 
 * 
 * @param x0 
 * @param y0 
 * @param x1 
 * @param y1 
 * @return true 
 * @return false 
 */
bool Prm::checkShallowLine(int x0, int y0, int x1, int y1)
{
    int dx = x1 - x0;
    int dy = y1 - y0;
    int yi = 1;     // increment

    if(dy < 0)
    {
        yi = -1;    // negative increment
        dy = -dy;
    }

    int err = 2*dy - dx;
    int y = y0;

    for(int x = x0; x <= x1; x++)   // increment y by 1px every loop
    {
        if( !free(x, y) )           // check coord
        {
            return false;
        }

        if(err > 0)                 // increment error
        {
            y = y + yi;
            err -= 2*dx;
        }
        err += 2*dy;
    }
    return true;
}


/**
 * @brief Returns true if the given pixel is FREE in the cv::mat img.
 * 
 * @param x x coordinate
 * @param y y coordinate
 * @return true 
 * @return false 
 */
bool Prm::free(int x, int y)
{
    // true if the pixel is free
    return img.at<uint8_t>(y, x) == (int)State::FREE;
}


/**
 * @brief Returns the square of the distance between two points.
 * 
 * @param p1 Point 1
 * @param p2 Point 2
 * @return int 
 */
int Prm::distanceSquared(Point p1, Point p2)
{
    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y;
    return dx*dx + dy*dy;
}


/**
 * @brief Attempts to populate the underlying graph with new points untill
 * the start and end point can be connected.
 * 
 * Each iteration, check if there is connectivity. If yes, finished.
 * If no, add new normal points and bridges in the ratio specified in the header.
 * 
 * There are two possible causes for failure.
 *  1. The points are impossible to connect.
 *  2. The algorithm failed to find the connection before reaching max nodes.
 * 
 * @return true 
 * @return false 
 */
bool Prm::populateMap()
{
    // width or heigh, whichever larger
    int max_dim = std::max(img.cols, img.rows);

    // Setup the random generator
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> x_distribution(0, img.cols);
    std::uniform_int_distribution<int> y_distribution(0, img.rows);
    std::uniform_int_distribution<int> bridge_distribution(-max_bridge_length, max_bridge_length);

    // check for bridge success
    bool no_bridge      = true;
    int normal_count    = 0;
    int node_increment  = 0;

    // Loop - while not too many nodes
    while(graph.getNodes().size() < max_nodes && prm_ok)
    {
        if( graph.bfSearch(start, goal) )   // Check if start-goal connected. Breadth-first search.
        {
            return true;
        }

        no_bridge = true;
        normal_count = 0;

        while(normal_count < normal_bridge_ratio)   // Make normal points
        {
            int x = x_distribution(generator);
            int y = y_distribution(generator);

            if( free(x, y) )    // Is random point free?
            {
                normal_count++;
                Point p{x, y};

                graph.add(p);   // Add to the graph, and make its connection with nearby nodes
                connectNode(p); 
            }
        }
        // start a random bridge
        if(use_bridges_)
        {
            while(no_bridge)    // Try until bridge is made
            {
                int x_b0 = x_distribution(generator);
                int y_b0 = y_distribution(generator);

                if( !free(x_b0, y_b0) )     // Generate first occupied point
                {
                    int x_b1 = bridge_distribution(generator) + x_b0;
                    int y_b1 = bridge_distribution(generator) + y_b0;
                    if( !inBounds({x_b1, y_b1}) )   // If its off the image start again
                    {
                        continue;
                    }

                    if( !free(x_b1, y_b1) )     // Second occupied point
                    {
                        Point mp = midpoint( {x_b0, y_b0}, {x_b1, y_b1} );  // Found 2 end points
                        if( free(mp.x, mp.y))    // Is the midpoint free?
                        {
                            // yay bridge
                            no_bridge = false;
                            graph.add(mp);
                            connectNode(mp);
                        }
                    }
                }
            }
        }
    }
    return false;
}


/**
 * @brief Populates a prm graph until end points can be connected. Then finds the shortest
 * connecting path. Returns path.
 * 
 * @param start_ Starting point
 * @param goal_ Goal point
 * @param map Configuration space
 * @return std::vector<Point> 
 */
std::vector<Point> Prm::planPath(Point start_, Point goal_, cv::Mat map)
{
    running = true;
    prm_ok = true;
    // Add to local copies
    img = map.clone();
    start = start_;
    goal = goal_;

    // Start/end of graph. Try to connect them
    graph.add(start);
    graph.add(goal);
    connectNode(start);

    std::vector<Point> path;

    // Add random nodes untill start/end connected
    if ( populateMap() )
    {
        // Get the shortest path
        auto path_nodes = graph.dijkstras(start, goal);
        
        // Add the nodes to a container of points
        for(auto node : path_nodes)
        {
            path.push_back(node.data);
        }
    }
    
    running = false;
    // If path is empty, the search failed
    return path;
} 


/**
 * @brief Connects a given point to nodes within max_connect_dist untill 
 * it has max_connections or there a no other nodes.
 * 
 * @param node The node to connect
 */
void Prm::connectNode(Point node)
{
    auto nodes = graph.getNodes();  // All the nodes in the graph
    int connections = graph.at(node)->getEdges().size();    // Connection counter

    // For each node in the graph
    for(auto node_ptr : nodes)
    {
        // Finished connecting
        if(connections >= max_connections)
        {
            break;
        }
        else
        {
            /**
             * These if statements are nested so each successive calculation
             * is only done if the previous condidition succeeds. This 
             * reduces the computation requred to connect a node. When the graph is
             * small this is not an issue, but for large maps, connection can be slow. 
             */
            if(node_ptr->getEdges().size() < max_connections)
            {
                int dist = distanceSquared(node_ptr->data, node);       // In range
                if( dist < max_connect_dist)
                {
                    if( lineOfSight(node, node_ptr->data) )             // Line of sight
                    { 
                        graph.connect(node_ptr->data, node, dist);      // Make connection
                        connections++;
                    }
                }
            }
        }
    }
}


/**
 * @brief Returns all nodes (as pointer) in the prm graph.
 * 
 * @return std::vector<NodeP*>
 */
std::vector<NodeP*> Prm::getAllNodes()
{
    return graph.getNodes();
}


/**
 * @brief Returns the mid-point of two points.
 * 
 * @param p1 Point 1
 * @param p2 Point 2
 * @return Point 
 */
Point Prm::midpoint(Point p1, Point p2)
{
    int xm = (p1.x + p2.x) * 0.5;   // Average x
    int ym = (p1.y + p2.y) * 0.5;   // Average y

    return {xm, ym};
}


/**
 * @brief Returns true if a point is within the config space image.
 * 
 * @param p Point
 * @return true 
 * @return false 
 */
bool Prm::inBounds(Point p)
{
    return p.x > 0 && p.x < img.cols && p.y > 0 && p.y < img.rows;
}

/**
 * @brief Attempts to abort the current prm creation.
 * 
 * @return true 
 * @return false 
 */
bool Prm::abort()
{
    prm_ok = false;

    while(running)
    {
        // Wait for any current prm processes to stop
    }

    return true;
}


/**
 * @brief Set max_connect_dist, the maximum SQUARED distance a node connection
 * can be made.
 * 
 * @param distance SQUARED distance
 */
void Prm::setMaxConnectDist(int distance)
{
    max_connect_dist = distance;
}

