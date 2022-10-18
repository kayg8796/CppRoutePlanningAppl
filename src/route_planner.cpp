#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

     this->start_node = &m_Model.FindClosestNode(start_x,start_y);
     this->end_node = &m_Model.FindClosestNode(end_x, end_y);

}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node);

}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (const auto node : current_node->neighbors){
        node->parent = current_node;
        node->g_value = current_node->g_value + current_node->distance(*node); // find where and how to get the gvalue
        node->h_value = this->CalculateHValue(node);
        open_list.push_back(node);
        node->visited = true;
    }

}

bool sortBySum(const RouteModel::Node *node1, const RouteModel::Node *node2)
{
  return (node1->g_value + node1->h_value) > (node2->g_value + node2->h_value);
}

RouteModel::Node *RoutePlanner::NextNode()
{
  RouteModel::Node *next_node = nullptr;
  std::sort(open_list.begin(), open_list.end(), sortBySum);
  if (!open_list.empty())
  {
    next_node = open_list.back();
    open_list.pop_back();
  }
  return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.insert(path_found.begin(),*current_node);
    RouteModel::Node *parentNode = current_node->parent ;
    while (parentNode != nullptr)
    {
        distance += current_node->distance(*parentNode);
        path_found.insert(path_found.begin(),*parentNode);

        current_node = parentNode;
        parentNode = parentNode->parent;
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {

    RouteModel::Node *current_node = nullptr;

    start_node->visited = true;
    open_list.push_back(start_node);
    while (!open_list.empty())
    {
    current_node = NextNode();
    if (current_node->distance(*end_node) == 0)
    {
        m_Model.path = ConstructFinalPath(current_node);
        return;
    }
    AddNeighbors(current_node);
    }
}