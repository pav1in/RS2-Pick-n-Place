#ifndef BP_RRT_STAR_LIB_H
#define BP_RRT_STAR_LIB_H

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <geometry_msgs/Pose.h>

struct Bounds {
  Eigen::Vector3d low, high;
};

struct Node {
  Eigen::Vector3d x;
  Node* parent = nullptr;
  double cost   = 0.0;
  Node(const Eigen::Vector3d& pt) : x(pt) {}
};

class BP_RRTStar {
public:
  BP_RRTStar(const Eigen::Vector3d& start,
             const Eigen::Vector3d& goal,
             const Bounds& bounds,
             double step_size,
             int max_iters,
             double goal_bias,
             double neighbor_radius);
  ~BP_RRTStar() = default;

  bool plan();
  std::vector<geometry_msgs::Pose> getPathMsg() const;

private:
  // Owns all the nodes we ever new'd
  std::vector<std::unique_ptr<Node>> tree_;

  // Holds the goal until we first connect; then we move it into tree_
  std::unique_ptr<Node> goal_uptr_;
  Node*                  goal_ptr_;

  Bounds   bounds_;
  double   step_size_;
  int      max_iters_;
  double   goal_bias_;
  double   neighbor_radius_;

  // Helpers:
  std::unique_ptr<Node> sampleRandom() const;
  Node*                 nearestNeighbor(const Eigen::Vector3d& x) const;
  std::vector<Node*>    nearNeighbors(Node* n) const;
  void                  chooseParent(Node* n, const std::vector<Node*>& nbrs);
  void                  rewire(Node* n, const std::vector<Node*>& nbrs);
  bool                  collisionFree(const Node& a, const Node& b) const;
  Eigen::Vector3d       steer(const Eigen::Vector3d& from,
                              const Eigen::Vector3d& to) const;
  double                dist(const Eigen::Vector3d& a,
                              const Eigen::Vector3d& b) const;
  std::vector<Node*>    extractPath() const;
};

#endif  // BP_RRT_STAR_LIB_H
