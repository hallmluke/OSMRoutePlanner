#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    auto start_node = &m_Model.FindClosestNode(start_x, start_y);
    auto end_node = &m_Model.FindClosestNode(end_x, end_y);
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node * current_node) {
    std::vector<RouteModel::Node> path_found;
    distance = 0.0f;

    while(current_node->parent != nullptr) {
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = *(current_node->parent);
    }

    path_found.push_back(*current_node);
    distance *= m_Mode.MetricScale();
    return path_found;
}

void RoutePlanner::AStarSearch() {
    end_node->parent = start_node;
    m_Model.path = ConstructFinalPath(&end_node);
    return;
}

float RouteModel::CalculateHValue(const *RouteModel::Node start_node) {
    return end_node->distance(*start_node);
}

bool CompareFValue(*RouteModel::Node node1, *RouteModel::Node node2){
    return node1->g_value + node1->h_value < node2->g_value + node2->h_value;
}

*RouteModel::Node RouteModel::NextNode() {
    std::sort(open_list.begin(), open_list.end(), CompareFValue);
    *RouteModel::Node next_node = open_list.begin();
    open_list.erase(open_list.begin());
    return next_node;
}