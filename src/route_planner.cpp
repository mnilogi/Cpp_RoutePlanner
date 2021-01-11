#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &(m_Model.FindClosestNode(start_x,start_y));
    end_node = &(m_Model.FindClosestNode(end_x,end_y));
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return(node -> distance(*end_node));
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
 current_node->FindNeighbors();
    for(auto *node_neighbor : current_node->neighbors)
    {
        node_neighbor ->parent = current_node;
        node_neighbor ->h_value = CalculateHValue(node_neighbor);
        node_neighbor ->g_value = current_node->g_value + node_neighbor->distance(*current_node);
        open_list.emplace_back(node_neighbor);
        node_neighbor ->visited = true;
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
std::sort(open_list.begin(),open_list.end(),[](const auto &node1,const auto &node2){
    return node1->g_value + node1->h_value > node2->g_value + node2->h_value;});
RouteModel::Node *pointer_to_last;
pointer_to_last = open_list.back();
open_list.pop_back();
return pointer_to_last;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *last_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node *current_node = last_node;
    while (current_node->parent != nullptr)
    {
        path_found.emplace_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }
    path_found.emplace_back(*current_node);
    std::reverse(path_found.begin(),path_found.end());
    distance *= m_Model.MetricScale(); 
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node;
    current_node = start_node;
    current_node->visited = true;
    current_node->g_value = 0.0;
    current_node->h_value = CalculateHValue(current_node);
    open_list.push_back(current_node);
    while (open_list.size() > 0)
    {
        current_node = RoutePlanner::NextNode();
        if(current_node == end_node)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        RoutePlanner::AddNeighbors(current_node);
    }
    std::cout <<  "\nNo path found";
}
