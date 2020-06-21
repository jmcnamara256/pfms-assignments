#ifndef GRAPH_H
#define GRAPH_H

#include <list>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>

#include <algorithm>
#include <limits>
#include <assert.h>

/*
    Class declarations

    Graph is made up of 3 classes. Graph - Node - Edge.

    A graph is made up of nodes, which can be connected by edges. 
    
    Each node has a container of its edges
    
    An edge is a connection between two nodes. The start of an edge is the owner of the edge object.
    Each edge has a pointer to a node, which is the destination.
    Edges have a weight which must be a comparible type
*/

namespace graph
{

// Forward declaration of edge and node
template <class T_node, class T_weight> class Edge;
template <class T_node, class T_weight> class Node;

/**
 * @brief Undirected, weighted graph template. A graph has a container of nodes, and
 * some utility functions.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 */
template <class T_node, class T_weight>
class Graph
{
    public:
        void add(const T_node &n);
        void remove(const T_node &n);
        void connect(const T_node &n1, const T_node &n2, T_weight weight);
        bool contains(const T_node &n);
        Node<T_node, T_weight>  * at(const T_node &n);
        std::vector< Node<T_node, T_weight>* > getNodes();

        std::vector< Node<T_node, T_weight> > dijkstras(const T_node &start, const T_node &goal);
        bool bfSearch(const T_node& start, const T_node& goal);

    private:
        // The primary graph container
        std::vector< Node<T_node, T_weight>* > nodes;
};


/**
 * @brief The vertex's contained in the graph. The node is made of a data payload, and
 * a container of edges.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 */
template <class T_node, class T_weight>
class Node
{
    public:
        Node(T_node data_) : data(data_){}
        Node() {};
        void addEdge(Node<T_node, T_weight>* destination, T_weight weight);
        void removeEdge(Node<T_node, T_weight>* destination);
        bool connectedTo(const Node<T_node, T_weight> &n);
        std::vector<Edge<T_node, T_weight>> getEdges();
        bool operator==(const Node<T_node, T_weight> &n1) const;

        // The actual object @ this node
        T_node data;

    private:
        // Each node has a list of edges
        std::vector< Edge<T_node, T_weight> > edges;

};


/**
 * @brief An edge is a connection between two nodes. The owner of the edge object
 * is the start of the connection, and the edge contains a pointer to the destination.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 */
template <class T_node, class T_weight>
class Edge
{
    public:
        Edge(Node<T_node, T_weight> * node_, T_weight weight_) : node(node_), weight(weight_){};
        T_weight weight;
        Node<T_node, T_weight>*  node;
};


//############################################################################


/**
 * @brief Adds a new node to the graph with payload n. Only adds to the graph
 * if the node is unique.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 * @param n The data payload for the node
 */
template<class T_node, class T_weight>
void Graph<T_node, T_weight>::add(const T_node &n)
{
    if(!contains(n))
    {
        Node<T_node, T_weight> * node = new Node<T_node, T_weight> (n);
        nodes.push_back(node); // Add the node if it does not already exist within the graph
    }
}


/**
 * @brief Removes a node from the prm graph. The node is identified by its data
 * payload.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 * @param n Data of node to delete
 */
template<class T_node, class T_weight>
void Graph<T_node, T_weight>::remove(const T_node &n)
{
    auto it = find_if(nodes.begin(), nodes.end(),
            [&n](const Node<T_node, T_weight> &n_){return n == n.data;});
    // The node exists
    if(it != nodes.end())
    {
        nodes.erase(it);    // Remove the node
        for(auto edge : (*it)->getEdges())
        {
            edge.node->removeEdge(*it);
        }
        delete *it;
    }
}


/**
 * @brief Returns the node container of the graph.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 * @return std::vector< Node<T_node, T_weight>* > 
 */
template<class T_node, class T_weight>
std::vector< Node<T_node, T_weight>* >  Graph<T_node, T_weight>::getNodes()
{
    return nodes;
}


/**
 * @brief Returns true if the graph contains a node with the given data.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 * @param n Data of the node to find
 * @return true 
 * @return false 
 */
template<class T_node, class T_weight>
bool Graph<T_node, T_weight>::contains(const T_node &n)
{
    auto it = find_if(nodes.begin(), nodes.end(),
            [&n](Node<T_node, T_weight> *n_){return n_->data == n;});
    return it != nodes.end();
}


/**
 * @brief Returns a read/write reference to the node with the given data.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 * @param n Data of the node get
 * @return Node<T_node, T_weight>* 
 */
template<class T_node, class T_weight>
Node<T_node, T_weight> * Graph<T_node, T_weight>::at(const T_node &n)
{
    auto it = find_if(nodes.begin(), nodes.end(),
            [&n](Node<T_node, T_weight> * n_){return n_->data == n;});
    return *it;
}


/**
 * @brief Makes an undirected connection between two nodes, with given weight.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 * @param n1 Data of node 1
 * @param n2 Data of node 2
 * @param weight Weight of connection
 */
template<class T_node, class T_weight>
void Graph<T_node, T_weight>::connect(const T_node &n1, const T_node &n2, T_weight weight)
{
    // Dont add a node to itself
    if(n1 != n2)
    {
        Node<T_node, T_weight> * node1, * node2;

        node1 = at(n1);
        node2 = at(n2);

        // Create edges
        node1->addEdge(node2, weight);
        node2->addEdge(node1, weight);
    }
}


/**
 * @brief Add an edge to a nodes list of connections. A connection
 * requires a node to connect to, and a weight.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 * @param n Data of node to connect to
 * @param weight Weight of connection
 */
template<class T_node, class T_weight>
void Node<T_node, T_weight>::addEdge(Node<T_node, T_weight> * n, T_weight weight)
{
    // Check if this node already has this edge
    if(!connectedTo(*n))
    {
        Edge<T_node, T_weight> e(n, weight);
        edges.push_back(e);
    }
}


/**
 * @brief Returns true if this node is connected to the node with given data.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 * @param n Data of node to check
 * @return true 
 * @return false 
 */
template<class T_node, class T_weight>
bool Node<T_node, T_weight>::connectedTo(const Node<T_node, T_weight> &n)
{
    auto it = find_if(edges.begin(), edges.end(),
            [&n](const Edge<T_node, T_weight> &e){return e.node->data == n.data;});
    
    return it != edges.end();
}


/**
 * @brief Returns the container of edges this node has.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 * @return std::vector< Edge<T_node, T_weight> > 
 */
template<class T_node, class T_weight>
std::vector< Edge<T_node, T_weight> > Node<T_node, T_weight>::getEdges()
{
    return edges;
}


/**
 * @brief Compares two nodes for equality by checking the underlying data payload.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 * @param n1 rhs
 * @return true 
 * @return false 
 */
template<class T_node, class T_weight>
bool Node<T_node, T_weight>::operator==(const Node<T_node, T_weight> &n1) const
{
    return this->data == n1.data;
}


/**
 * @brief Dijkstra's shortest path search algorithm. Returns the shortest path (by weight) 
 * between two nodes contained in the graph.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 * @param start Data of start node
 * @param goal Data of goal node
 * @return std::vector< Node<T_node, T_weight> > 
 */
template<class T_node, class T_weight>
std::vector< Node<T_node, T_weight> > Graph<T_node, T_weight>::dijkstras(const T_node &start, const T_node &goal)
{
    // Types 
    typedef Node<T_node, T_weight>          NodeType;
    typedef std::pair<NodeType, T_weight>   w_node;
    struct NodeData {T_weight distance; NodeType previous; bool visited; };

    // Sort function
    auto greater = [&](const w_node& n1, const w_node& n2) { return n1.second > n2.second; };

    // Containers
    std::unordered_map  <NodeType, NodeData>                                node_data;
    std::priority_queue <w_node, std::vector<w_node>, decltype(greater)>    unvisited (greater);

    // Graph nodes
    std::vector< Node<T_node, T_weight>* > nodes = getNodes();

    NodeType start_node = *at(start);
    NodeType goal_node = *at(goal); 
    
    // Populate the unvisited queue with all nodes
    unvisited.emplace(std::make_pair(start_node, 0));
    node_data.emplace(std::make_pair(start_node, NodeData{0, start_node, false}));

    // While there are still unvisited nodes
    // 1. get the closest node in terms of total distance
    // 2. check the total distance to all its neighbours
    // 3. replace any distances that are shorter
    // 4. mark the current node as visited
    while(!unvisited.empty())
    {
        // Get the closest node/distance pair
        auto & cp = unvisited.top(); unvisited.pop();
        NodeType current            = cp.first;
        T_weight current_distance   = cp.second;

        // Destination reached
        if(current == goal)
        {
            break;
        }

        // Get data about the current node
        //auto current_node = node_data.find(current);
        auto current_node = node_data.find(current);

        // If its visited, or not the updated priority, move on.
        if(current_node->second.distance != current_distance)
        {
            continue;
        }
        else if(current_node->second.visited)
        {
            continue;
        }

        // Otherwise, mark it visited - we are visiting it now
        current_node->second.visited = true;

        // For each connection (edge)
        for(auto &connection : current.getEdges())
        {
            // The node at the end of the edge - the neighbour
            NodeType * neighbour_node = connection.node;
            
            // TOTAL distance to neighbour is distance to current + weight of edge to neighbour
            auto dist_neighbour = current_distance + connection.weight;

            // Get data about the neighbour
            auto neighbour_data = node_data.find(*neighbour_node);

            // Discovered shorter route to neighbour
            if(neighbour_data == node_data.end())
            {
                unvisited.emplace(std::make_pair(*neighbour_node, dist_neighbour));
                node_data.emplace(std::make_pair(*neighbour_node, NodeData{dist_neighbour, current, false}));
            }
            else if(dist_neighbour < neighbour_data->second.distance)
            {
                // Updates its data
                neighbour_data->second.distance = dist_neighbour;
                neighbour_data->second.previous = current;


                // Add this new closest version to the queue
                unvisited.emplace(std::make_pair(*neighbour_node, dist_neighbour));
            }
        }
    }
    // Get the path via the parents of each node
    std::vector< NodeType > path;

    NodeType path_node = goal_node;
    
    // If the goal was reachable back track for the path
    auto found_goal = node_data.find(goal_node);
    if(found_goal != node_data.end())
    {
        // While not at the start
        do
        {
            path.push_back(path_node);  // Add the node to the path
            path_node = node_data.at(path_node).previous;   // Move on to the previous node
        }
        while(path.back().data != start_node.data);

        // Reverse the path, because the above put it goal->start
        std::reverse(path.begin(), path.end());
    }

    return path;
} // Djikstra's


/**
 * @brief Runs a breadth-first search of nodes, starting from start. Returns true if goal is found.
 * 
 * @tparam T_node 
 * @tparam T_weight 
 * @param start Data of start node
 * @param goal Data of goal node
 * @return true 
 * @return false 
 */
template<class T_node, class T_weight>
bool Graph<T_node, T_weight>::bfSearch(const T_node& start, const T_node& goal)
{
    typedef Node<T_node, T_weight> NodeType;
    
    std::unordered_set<NodeType> to_do;
    std::unordered_set<NodeType> done;

    NodeType n = *at(start);
    to_do.insert(n);

    // While not discovered all nodes
    while( !to_do.empty() )
    {
        // Pop the next element to do
        auto current = *to_do.begin(); to_do.erase(to_do.begin());
        done.insert(current);

        // For each connected node
        for(auto connection : current.getEdges())
        {
            auto neighbour = *connection.node;

            // If its the goal, return
            if(neighbour.data == goal)
            {
                return true;
            }
            auto it_t = to_do.find(neighbour);
            auto it_d = done.find(neighbour);

            // Undiscovered, add it to to-do
            if(it_t == to_do.end() && it_d == done.end())
            {
                to_do.insert(neighbour);
            }
        }
    }
    return false;
}

} // graph

namespace std
{
    /**
     * @brief We need to define a hash function to use unordered_map with custom classes.
     * Since this is templated its hard to implement a general hash for all possible templates.
     * This simply calls the hash of the underlying data of each node. The assumption is that 
     * the data class will have a hash function. If a standard c++ type like int/char is used
     * it will have a hash. In the case of 'Point' which is used for prm, I just made a basic
     * hash specialisation.
     * 
     * @tparam T_node
     * @tparam T_weight 
     */
    template <class T_node, class T_weight>
    struct hash< graph::Node<T_node, T_weight> >
    {
        size_t operator()( const graph::Node<T_node, T_weight>& k ) const
        {
            return hash<T_node>{}(k.data);
        }
    };
} // std

#endif //GRAPH_H