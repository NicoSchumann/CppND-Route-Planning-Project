#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // PASSED 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = & m_Model.FindClosestNode( start_x, start_y );
    start_node->visited = true;
    open_list.push_back( start_node );
    end_node = & m_Model.FindClosestNode( end_x, end_y );

}


// Implements the CalculateHValue method.
// - Uses the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

    return end_node->distance( *node );
}


// Expands the current node by adding all unvisited neighbors to the open list.
// - Uses the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, it sets the parent, the h_value, the g_value. 
// - Uses CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, it adds the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors) {
        if (neighbor->visited)
            continue;
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue( neighbor );
        neighbor->g_value = current_node->g_value + current_node->distance( *neighbor );
        neighbor->visited = true;
        open_list.push_back( neighbor );
    }
}

// - Sorts the open_list according to the sum of the h value and g value.
// - Creates a pointer to the node in the list with the lowest sum.
// - Removes that node from the open_list.
// - Returns the pointer.

RouteModel::Node *RoutePlanner::NextNode() {

    std::sort(open_list.begin(), open_list.end(),[](RouteModel::Node *fst, RouteModel::Node *snd) 
                  {return fst->g_value + fst->h_value  >=  snd->g_value + snd->h_value;}
    );
    RouteModel::Node *nearest = open_list.back();
    open_list.pop_back();
    return nearest;
}


// - This method takes the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector is in the correct order: the start node is the first element
//   of the vector, the end node is the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    if (current_node == nullptr)
        return path_found;
        
    while (current_node->parent) {
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    
    path_found.push_back(*current_node); // Add at least the root mode.
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// - Uses the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Uses the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, the ConstructFinalPath method returns the final path that was found.
// - Stores the final path in the m_Model.path attribute before the method exits. This path will then
//   be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    while (! open_list.empty()) {
        RouteModel::Node *current_node = NextNode();
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node); 
    }
}