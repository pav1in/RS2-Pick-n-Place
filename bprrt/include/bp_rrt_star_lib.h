#pragma once
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <geometry_msgs/Pose.h>

/// Axis‐aligned sampling region
struct Bounds
{
  Eigen::Vector3d low, high;
  Bounds() = default;
  Bounds(const Eigen::Vector3d&_low, const Eigen::Vector3d& _high) : low(_low),high(_high) {}
};

/// Single tree node
struct Node
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d x;       ///< Cartesian position
  double         cost;     ///< cost‐to‐come
  Node*          parent;   ///< back‐pointer

  Node(const Eigen::Vector3d& _x)
    : x(_x), cost(0.0), parent(nullptr) {}
};

class BP_RRTStar
{
public:
  /// ctor: must pass start, goal, sampling bounds, and algorithm params
  BP_RRTStar(const Eigen::Vector3d& start,
             const Eigen::Vector3d& goal,
             const Bounds&         bounds,
             double                step_size,
             int                   max_iters,
             double                goal_bias,
             double                neighbor_radius);

  ~BP_RRTStar();

  /// run planner (returns true if goal reached)
  bool plan();

  /// get planned path as a vector of geometry_msgs::Pose
  std::vector<geometry_msgs::Pose> getPathMsg() const;

private:
  // config & problem data
  Bounds           bounds_;
  double           step_size_;
  int              max_iters_;
  double           goal_bias_;
  double           neighbor_radius_;

  // tree storage
  Node*            start_;
  Node*            goal_;
  std::vector<Node*> tree_;

  // core RRT* steps
  Node*            sampleRandom();
  Node*            nearestNeighbor(const Eigen::Vector3d& x_rand) const;
  Eigen::Vector3d  steer(const Eigen::Vector3d& from,
                         const Eigen::Vector3d& to) const;
  std::vector<Node*> nearNeighbors(const Node* n) const;
  void             chooseParent(Node* n, const std::vector<Node*>& nbrs);
  void             rewire(Node* n, const std::vector<Node*>& nbrs);

  // utilities
  bool             collisionFree(const Node& a, const Node& b) const;
  double           dist(const Eigen::Vector3d& a,
                        const Eigen::Vector3d& b) const;

  std::vector<Node*> extractPath() const;

  // Disallow copying
  BP_RRTStar(const BP_RRTStar&) = delete;
  BP_RRTStar& operator=(const BP_RRTStar&) = delete;
};
