#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  // RoutePlanner's start_node and end_node attributes
  this->start_node = &m_Model.FindClosestNode(start_x, start_y);
  this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*this->end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (RouteModel::Node *neighbor : current_node->neighbors) {
    if (!neighbor->visited) {
      neighbor->parent = current_node;
      neighbor->g_value =
          current_node->g_value + current_node->distance(*neighbor);
      neighbor->h_value = this->CalculateHValue(neighbor);
      this->open_list.emplace_back(neighbor);
      neighbor->visited = true;
    }
  }
}


// NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(this->open_list.begin(), this->open_list.end(),
            [](auto &a, const auto &b) {
              return (a->h_value + a->g_value) > (b->h_value + b->g_value);
            });
  RouteModel::Node *next_node = this->open_list.back();
  this->open_list.pop_back();
  return next_node;
}


// Method that returns the final path found
std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;
  path_found.push_back(*current_node);
  
  while (current_node != this->start_node) {
    path_found.push_back(*(current_node->parent));
    distance += current_node->distance(*(current_node->parent));
    current_node = current_node->parent;
  }

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of
                                     // the map to get meters.
  std::reverse(path_found.begin(), path_found.end());
  return path_found;
}

// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node
// to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method
// to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits.
// This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;
  current_node = this->start_node;
  current_node->visited = true;
  while(current_node != this->end_node){
    this->AddNeighbors(current_node);
    current_node = this->NextNode();
  }
  auto path_final = this->ConstructFinalPath(current_node);
  this->m_Model.path = path_final;
}