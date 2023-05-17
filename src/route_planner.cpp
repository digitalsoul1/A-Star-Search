#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  this->start_node = &m_Model.FindClosestNode(start_x, start_y);
  this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();

  auto neighbors = current_node->neighbors;

  for (auto neighbor : neighbors) {
    if (!neighbor->visited) {
      neighbor->g_value =
          current_node->g_value + neighbor->distance(*current_node);
      neighbor->parent = current_node;
      neighbor->h_value = this->CalculateHValue(neighbor);
      neighbor->visited = true;

      open_list.push_back(neighbor);
    }
  }
}

bool RoutePlanner::Compare(const RouteModel::Node *a,
                           const RouteModel::Node *b) {
  float f1 = a->g_value + a->h_value; // f1 = g1 + h1
  float f2 = b->g_value + b->h_value; // f2 = g2 + h2
  return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(open_list.begin(), open_list.end(), Compare);
  auto node = open_list.back();
  open_list.pop_back();
  return node;
}

std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  RouteModel::Node *node = current_node;

  while (node != start_node) {
    distance += node->distance(*node->parent);
    path_found.push_back(*node);
    node = node->parent;
  }

  path_found.push_back(*start_node);

  distance *= m_Model.MetricScale(); 

  std::reverse(path_found.begin(), path_found.end());
  return path_found;
}

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = start_node;
  current_node->parent = nullptr;
  current_node->g_value = 0;
  current_node->h_value = CalculateHValue(current_node);
  current_node->visited = true;

  AddNeighbors(current_node);

  while (open_list.size() > 0) {

    auto next_node = NextNode();

    if (next_node->x == end_node->x && next_node->y == end_node->y) {
      m_Model.path = ConstructFinalPath(next_node);
    }

    AddNeighbors(next_node);
  }
}
