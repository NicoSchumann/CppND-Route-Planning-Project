#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
  
    ///
    /// Input arguments are the RouteModel object and the start and goal values
    /// for the path to search.
    ///
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    
    ///
    /// Returns the distance of the found path.
    ///
    float GetDistance() const {return distance;}
    
    ///
    /// Performs a path search with A* Search algorithm.
    ///
    void AStarSearch();

    ///
    /// Adds those neighbor nodes to RoteModel::neighbors vector
    /// whose are not marked as visited. Then, they will get set visited.
    ///
    void AddNeighbors(RouteModel::Node *current_node);
    
    
    ///
    /// Calculates air distance of the node to the goal_node.
    ///
    float CalculateHValue(RouteModel::Node const *node);
    
    ///
    /// Returns a vector which holds all nodes building the path.
    ///
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    
    ///
    /// Returns the node with the smallest g + h value (which builds the f value).
    /// This node will popped from the open_list;
    ///
    RouteModel::Node *NextNode();

  private:
    // Add private variables or methods declarations here.
    std::vector<RouteModel::Node*> open_list;  ///< Holds all visited nodes.
    RouteModel::Node *start_node;   ///< Start node for the search
    RouteModel::Node *end_node;     ///< Goal node for the serch. 

    float distance = 0.0f;   ///< length of the found path.
    RouteModel &m_Model;  ///< Model of the map.
};

#endif