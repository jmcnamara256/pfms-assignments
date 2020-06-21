#include "gtest/gtest.h"
#include "prm/prm.h"
#include "graph/graph.h"
#include "prm/point.h"

#include "opencv2/highgui.hpp"
#include "ros/package.h"

#include <iostream>

/**
 * @brief Check validity of shortest path algorithm included in the graph class.
 * 
 * The graph class implements dijkstras shortest path algorithm. Here a simplified graph
 * is created, and the results from the calculation are compared to the correct 
 * path.
 * 
 * A diagram of the graph used is includled in the documentation!!
 */
TEST(GRAPHTEST, shortest_path)
{
    graph::Graph<char, int> g;

    // Add nodes
    g.add('A'); g.add('B'); g.add('C');
    g.add('D'); g.add('E'); g.add('F');
    g.add('G'); g.add('H'); g.add('I');
    g.add('J'); g.add('K'); g.add('L');
    g.add('M'); g.add('N'); g.add('O');

    // Connect nodes
    g.connect('A', 'B', 1); g.connect('A', 'C', 8); g.connect('A', 'D', 1);
    g.connect('B', 'E', 5); g.connect('B', 'F', 8); g.connect('B', 'G', 7);
    g.connect('C', 'G', 2); g.connect('C', 'I', 2); g.connect('D', 'J', 1);
    g.connect('E', 'O', 4); g.connect('G', 'H', 2); g.connect('H', 'I', 1);
    g.connect('J', 'I', 1); g.connect('K', 'L', 1); g.connect('K', 'N', 2);
    g.connect('L', 'M', 1); g.connect('L', 'N', 3); g.connect('M', 'N', 1);
    g.connect('N', 'O', 4);

    std::vector<char> expected_ag = {'A', 'D', 'J', 'I', 'H', 'G'};
    std::vector<char> expected_bl = {'B', 'E', 'O', 'N', 'M', 'L'};
    std::vector<char> actual_ag;
    std::vector<char> actual_bl;

    // get the calculated path A->G
    auto sp = g.dijkstras('A', 'G');
    for(auto n : sp)
    {
        actual_ag.push_back(n.data);
    }

    // get the calculated path B->L
    sp = g.dijkstras('B', 'L');
    for(auto n : sp)
    {
        actual_bl.push_back(n.data);
    }

    EXPECT_EQ(expected_ag, actual_ag);
    EXPECT_EQ(expected_bl, actual_bl);
}


/**
 * @brief Verify that the breadth first search of the graph functions as expected.
 * 
 * The expectation is returns true if there is a possible connection between 
 * 2 points. Otherwise returns false.
 */
TEST(GRAPHTEST, bfs)
{
    graph::Graph<char, int> g;
    // Add nodes
    g.add('A'); g.add('B'); g.add('C');
    g.add('D'); g.add('E'); g.add('F');
    g.add('G'); g.add('H'); g.add('I');
    g.add('J'); g.add('K'); g.add('L');
    g.add('M'); g.add('N'); g.add('O');

    // Connect nodes
    g.connect('A', 'B', 1); g.connect('A', 'C', 8); g.connect('A', 'D', 1);
    g.connect('B', 'F', 8); g.connect('B', 'G', 7); g.connect('N', 'O', 4);
    g.connect('C', 'G', 2); g.connect('C', 'I', 2); g.connect('D', 'J', 1);
    g.connect('E', 'O', 4); g.connect('G', 'H', 2); g.connect('H', 'I', 1);
    g.connect('J', 'I', 1); g.connect('K', 'L', 1); g.connect('K', 'N', 2);
    g.connect('L', 'M', 1); g.connect('L', 'N', 3); g.connect('M', 'N', 1);
    
    bool jl = g.bfSearch('J', 'L');
    bool jf = g.bfSearch('J', 'F');

    bool expected_jl = false;
    bool expected_jf = true;

    EXPECT_EQ(jl, expected_jl);
    EXPECT_EQ(jf, expected_jf);

    // Now reconnect the graph by adding the missing node.
    g.connect('B', 'E', 5);

    jl = g.bfSearch('J', 'L');
    expected_jl = true;

    EXPECT_EQ(jl, expected_jl);
}


/**
 * @brief Test if adding nodes with identical data payloads results in duplicate
 * entries.
 * 
 * The expected result is that before adding a node to the graph, check if
 * data is already contained and if so, do not add node.
 */
TEST(GRAPHTEST, unique)
{
    graph::Graph<char, int> g;

    g.add('A');
    g.add('A');

    auto size = g.getNodes().size();

    EXPECT_EQ(size, 1);
}


/**
 * @brief Test if the graph contains function works as expected.
 * 
 * The expected result is returns true if a node with data==arg exists.
 * False is no node exists.
 */
TEST(GRAPHTEST, contains)
{
    graph::Graph<char, int> g;

    g.add('A'); g.add('B'); g.add('C'); g.add('D');

    bool A = g.contains('A');
    bool D = g.contains('D');
    
    bool N = g.contains('N');
    bool a = g.contains('a');

    EXPECT_EQ(A, true);
    EXPECT_EQ(D, true);
    EXPECT_EQ(N, false);
    EXPECT_EQ(a, false);
}


/**
 * @brief Test if the hashing function for custom class 'Point' generates unique hashes
 * for a range of values. 
 */
TEST(POINTTEST, hash)
{
    std::vector<size_t> hashes;

    // Check 500 combinations of point
    int max = 500;
    for(int i = 0; i < max; i++)
    {
        Point p(i, max-i);
        size_t hash = std::hash<Point>{}(p);

        auto it = find(hashes.begin(), hashes.end(), hash);

        // Hashes should all be unique
        EXPECT_EQ(it, hashes.end());

        hashes.push_back(hash);
    }
}


/**
 * @brief Test if the PRM plan function returns the expected result when using 
 * bridges to detect narrow passages
 * 
 * The expected result is: when using the bridges, maps with narrow passages are
 * solved faster than without bridges
 */
TEST(PRM, narrow_passage)
{
    // Load the test image
    std::string path = ros::package::getPath("prm");
    path += "/maps/test.png";
    cv::Mat img = cv::imread(path , 0);

    Prm prm_n;        prm_n.setMaxConnectDist(800);     // Prm with bridges
    Prm prm_r(false); prm_r.setMaxConnectDist(800);     // No bridges

    auto path_n = prm_n.planPath({125,100}, {350, 425}, img);
    auto path_r = prm_r.planPath({125,100}, {350, 425}, img);

    // With bridges should resolve with less nodes, since the narrow passages are addressed
    EXPECT_TRUE( prm_n.getAllNodes().size() < prm_r.getAllNodes().size() );

}


/**
 * @brief Test if the path is returned when an impossible goal is given
 * 
 * Because PRM doesnt technically check if a goal is possible, a maximum number
 * of nodes is set. If the graph exceeds this without locating the goal, it
 * is assumed that the goal is unreachable 
 */
TEST(PRM, path_fail)
{
    // Load the test image
    std::string path = ros::package::getPath("prm");
    path += "/maps/test.png";
    cv::Mat img = cv::imread(path , 0);

    Prm prm_f;
    prm_f.setMaxConnectDist(800);

    auto path_f = prm_f.planPath({125,100}, {350, 100}, img);

    // The path should be empty
    EXPECT_TRUE( path_f.empty() );
}

/**
 * @brief Test if all connections made when populating prm are under
 * the maximum connection length set.
 */
TEST(PRM, connection_length)
{
    int max_connect_dist = 400;
    // Load the test image
    std::string path = ros::package::getPath("prm");
    path += "/maps/test.png";
    cv::Mat img = cv::imread(path , 0);

    Prm prm_f;
    prm_f.setMaxConnectDist(max_connect_dist);

    auto path_f = prm_f.planPath({85,425}, {400, 425}, img);

    // For each node, ensure that the connect distances are correct
    for(auto n_ptr : prm_f.getAllNodes())
    {
        for( auto edge : n_ptr->getEdges() )
        {
            EXPECT_TRUE( edge.weight <  max_connect_dist);
        }
    }
}


/**
 * @brief Test if each node has less (or equal) connections to max number of connections
 * 
 */
TEST(PRM, connection_number)
{
    // Load the test image
    std::string path = ros::package::getPath("prm");
    path += "/maps/test.png";
    cv::Mat img = cv::imread(path , 0);

    Prm prm_f;
    prm_f.setMaxConnectDist(800);

    auto path_f = prm_f.planPath({85,425}, {400, 425}, img);

    // For each node, check it doesnt have too many conections
    for(auto n_ptr : prm_f.getAllNodes())
    {
        EXPECT_TRUE( n_ptr->getEdges().size() <= 5 );
    }
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}