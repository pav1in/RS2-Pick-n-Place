#include "rrt_star_planner_lib.h"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <algorithm>
#include <ctime>

double euclideanDistance(Node* n1, Node* n2) {
  return sqrt(pow(n1->x - n2->x, 2) +
              pow(n1->y - n2->y, 2) +
              pow(n1->z - n2->z, 2));
}

RRTStarPlannerLib::RRTStarPlannerLib(double start_x, double start_y, double start_z,
                                     double goal_x, double goal_y, double goal_z,
                                     double x_min, double x_max,
                                     double y_min, double y_max,
                                     double z_min, double z_max,
                                     double step_size, int max_iter,
                                     double goal_sample_rate, double neighbor_radius)
  : x_min_(x_min), x_max_(x_max),
    y_min_(y_min), y_max_(y_max),
    z_min_(z_min), z_max_(z_max),
    step_size_(step_size), max_iter_(max_iter),
    goal_sample_rate_(goal_sample_rate), neighbor_radius_(neighbor_radius)
{
  std::srand(std::time(0));
  start_ = new Node(start_x, start_y, start_z);
  goal_ = new Node(goal_x, goal_y, goal_z);
  tree_.push_back(start_);
}

RRTStarPlannerLib::~RRTStarPlannerLib() {
  for (size_t i = 0; i < tree_.size(); ++i)
    delete tree_[i];
  delete goal_;
}

bool RRTStarPlannerLib::plan() {
  for (int i = 0; i < max_iter_; i++) {
    Node* rand_node = sampleRandomNode();
    Node* nearest_node = getNearestNode(rand_node);
    Node* new_node = steer(nearest_node, rand_node);
    delete rand_node; // Free the random sample.
    
    if (collisionFree(nearest_node, new_node)) {
      std::vector<Node*> neighbors = nearNodes(new_node);
      chooseParent(new_node, neighbors);
      tree_.push_back(new_node);
      rewire(new_node, neighbors);
    } else {
      delete new_node;
    }
    
    if (euclideanDistance(new_node, goal_) < step_size_) {
      if (collisionFree(new_node, goal_)) {
        goal_->parent = new_node;
        goal_->cost = new_node->cost + euclideanDistance(new_node, goal_);
        tree_.push_back(goal_);
        return true;  // Plan succeeded.
      }
    }
  }
  return false;
}

std::vector<geometry_msgs::Pose> RRTStarPlannerLib::getPath() {
  std::vector<Node*> node_path = extractPath();
  std::vector<geometry_msgs::Pose> path;
  for (size_t i = 0; i < node_path.size(); i++) {
    geometry_msgs::Pose p;
    p.position.x = node_path[i]->x;
    p.position.y = node_path[i]->y;
    p.position.z = node_path[i]->z;
    p.orientation.w = 1.0;  // Default orientation.
    path.push_back(p);
  }
  return path;
}

Node* RRTStarPlannerLib::sampleRandomNode() {
  if ((double)std::rand() / RAND_MAX < goal_sample_rate_)
    return new Node(goal_->x, goal_->y, goal_->z);
  double x = x_min_ + (x_max_ - x_min_) * ((double)std::rand() / RAND_MAX);
  double y = y_min_ + (y_max_ - y_min_) * ((double)std::rand() / RAND_MAX);
  double z = z_min_ + (z_max_ - z_min_) * ((double)std::rand() / RAND_MAX);
  return new Node(x, y, z);
}

Node* RRTStarPlannerLib::getNearestNode(Node* random_node) {
  Node* nearest = nullptr;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < tree_.size(); i++) {
    double d = euclideanDistance(tree_[i], random_node);
    if (d < min_dist) {
      min_dist = d;
      nearest = tree_[i];
    }
  }
  return nearest;
}

Node* RRTStarPlannerLib::steer(Node* from_node, Node* to_node) {
  double d = euclideanDistance(from_node, to_node);
  Node* new_node = nullptr;
  if (d <= step_size_) {
    new_node = new Node(to_node->x, to_node->y, to_node->z);
  } else {
    double theta = atan2(to_node->y - from_node->y, to_node->x - from_node->x);
    double horizontal_dist = sqrt(pow(to_node->x - from_node->x, 2) +
                                   pow(to_node->y - from_node->y, 2));
    double phi = atan2(to_node->z - from_node->z, horizontal_dist);
    double new_x = from_node->x + step_size_ * cos(theta) * cos(phi);
    double new_y = from_node->y + step_size_ * sin(theta) * cos(phi);
    double new_z = from_node->z + step_size_ * sin(phi);
    new_node = new Node(new_x, new_y, new_z);
  }
  new_node->parent = from_node;
  new_node->cost = from_node->cost + euclideanDistance(from_node, new_node);
  return new_node;
}

bool RRTStarPlannerLib::collisionFree(Node* from_node, Node* to_node) {
  // Placeholder: assume no obstacles.
  return true;
}

std::vector<Node*> RRTStarPlannerLib::nearNodes(Node* new_node) {
  std::vector<Node*> neighbors;
  for (size_t i = 0; i < tree_.size(); i++) {
    if (euclideanDistance(tree_[i], new_node) < neighbor_radius_)
      neighbors.push_back(tree_[i]);
  }
  return neighbors;
}

void RRTStarPlannerLib::chooseParent(Node* new_node, const std::vector<Node*>& neighbors) {
  Node* best_parent = new_node->parent;
  double best_cost = new_node->cost;
  for (size_t i = 0; i < neighbors.size(); i++) {
    if (collisionFree(neighbors[i], new_node)) {
      double cost = neighbors[i]->cost + euclideanDistance(neighbors[i], new_node);
      if (cost < best_cost) {
        best_parent = neighbors[i];
        best_cost = cost;
      }
    }
  }
  new_node->parent = best_parent;
  new_node->cost = best_cost;
}

void RRTStarPlannerLib::rewire(Node* new_node, const std::vector<Node*>& neighbors) {
  for (size_t i = 0; i < neighbors.size(); i++) {
    if (neighbors[i] == new_node->parent)
      continue;
    if (collisionFree(new_node, neighbors[i])) {
      double cost_through_new = new_node->cost + euclideanDistance(new_node, neighbors[i]);
      if (cost_through_new < neighbors[i]->cost) {
        neighbors[i]->parent = new_node;
        neighbors[i]->cost = cost_through_new;
      }
    }
  }
}

std::vector<Node*> RRTStarPlannerLib::extractPath() {
  std::vector<Node*> path;
  Node* closest = getNearestNode(goal_);
  Node* current = closest;
  while (current != nullptr) {
    path.push_back(current);
    current = current->parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}
