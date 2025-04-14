#ifndef RRT_STAR_PLANNER_LIB_H
#define RRT_STAR_PLANNER_LIB_H

#include <vector>
#include <geometry_msgs/Pose.h>

// Simple structure for an RRT* node.
struct Node {
  double x, y, z, cost;
  Node* parent;
  Node(double x_val, double y_val, double z_val)
    : x(x_val), y(y_val), z(z_val), cost(0.0), parent(nullptr) {}
};

// Helper function (used internally)
double euclideanDistance(Node* n1, Node* n2);

class RRTStarPlannerLib {
public:
  RRTStarPlannerLib(double start_x, double start_y, double start_z,
                    double goal_x, double goal_y, double goal_z,
                    double x_min, double x_max,
                    double y_min, double y_max,
                    double z_min, double z_max,
                    double step_size, int max_iter,
                    double goal_sample_rate, double neighbor_radius);
  ~RRTStarPlannerLib();

  // Run the planner. Returns true if a solution is found.
  bool plan();

  // Get the resulting path as a vector of Cartesian poses.
  std::vector<geometry_msgs::Pose> getPath();

private:
  Node* start_;
  Node* goal_;
  std::vector<Node*> tree_;

  // Parameters defining the planning region and algorithm behavior.
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
  double step_size_, goal_sample_rate_, neighbor_radius_;
  int max_iter_;

  // RRT* helper functions.
  Node* sampleRandomNode();
  Node* getNearestNode(Node* random_node);
  Node* steer(Node* from_node, Node* to_node);
  bool collisionFree(Node* from_node, Node* to_node);
  std::vector<Node*> nearNodes(Node* new_node);
  void chooseParent(Node* new_node, const std::vector<Node*>& neighbors);
  void rewire(Node* new_node, const std::vector<Node*>& neighbors);
  std::vector<Node*> extractPath();
};

#endif // RRT_STAR_PLANNER_LIB_H
