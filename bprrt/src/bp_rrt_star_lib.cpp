#include "bp_rrt_star_lib.h"
#include <cstdlib>
#include <ctime>
#include <limits>
#include <ros/console.h>

BP_RRTStar::BP_RRTStar(const Eigen::Vector3d& start,
                       const Eigen::Vector3d& goal,
                       const Bounds&         bounds,
                       double                step_size,
                       int                   max_iters,
                       double                goal_bias,
                       double                neighbor_radius)
  : bounds_(bounds)
  , step_size_(step_size)
  , max_iters_(max_iters)
  , goal_bias_(goal_bias)
  , neighbor_radius_(neighbor_radius)
{
  std::srand(std::time(nullptr));
  start_ = new Node(start);
  goal_  = new Node(goal);
  tree_.push_back(start_);
}

BP_RRTStar::~BP_RRTStar()
{
  for (auto *n : tree_) delete n;
  delete goal_;
}

bool BP_RRTStar::plan()
{
  for (int i = 0; i < max_iters_; ++i)
  {
    // 1) sample
    Node* x_rand_node = sampleRandom();
    // 2) nearest
    Node* x_near = nearestNeighbor(x_rand_node->x);
    // 3) steer
    Eigen::Vector3d x_new_pt = steer(x_near->x, x_rand_node->x);
    delete x_rand_node;

    Node* x_new = new Node(x_new_pt);
    x_new->parent = x_near;
    x_new->cost   = x_near->cost + dist(x_near->x, x_new_pt);

    // 4) collision check
    if (!collisionFree(*x_near, *x_new))
    {
      delete x_new;
      continue;
    }

    // 5) find neighbors, choose best parent & rewire
    auto neighbors = nearNeighbors(x_new);
    chooseParent(x_new, neighbors);
    tree_.push_back(x_new);
    rewire(x_new, neighbors);

    // 6) check for goal reach
    if (dist(x_new->x, goal_->x) <= step_size_ &&
        collisionFree(*x_new, *goal_))
    {
      goal_->parent = x_new;
      goal_->cost   = x_new->cost + dist(x_new->x, goal_->x);
      tree_.push_back(goal_);
      ROS_INFO("BP‑RRT* reached goal in %d iters", i);
      return true;
    }
  }
  ROS_WARN("BP‑RRT* failed to find a path within %d iters", max_iters_);
  return false;
}

Node* BP_RRTStar::sampleRandom()
{
  double r = (double)std::rand() / RAND_MAX;
  if (r < goal_bias_)
    return new Node(goal_->x);

  Eigen::Vector3d p;
  p.x() = bounds_.low.x() + (bounds_.high.x() - bounds_.low.x()) * ((double)std::rand()/RAND_MAX);
  p.y() = bounds_.low.y() + (bounds_.high.y() - bounds_.low.y()) * ((double)std::rand()/RAND_MAX);
  p.z() = bounds_.low.z() + (bounds_.high.z() - bounds_.low.z()) * ((double)std::rand()/RAND_MAX);
  return new Node(p);
}

Node* BP_RRTStar::nearestNeighbor(const Eigen::Vector3d& x_rand) const
{
  Node* best = nullptr;
  double best_d = std::numeric_limits<double>::infinity();
  for (auto *n : tree_)
  {
    double d = dist(n->x, x_rand);
    if (d < best_d)
    {
      best_d = d;
      best = n;
    }
  }
  return best;
}

Eigen::Vector3d BP_RRTStar::steer(const Eigen::Vector3d& from,
                                  const Eigen::Vector3d& to) const
{
  Eigen::Vector3d delta = to - from;
  double len = delta.norm();
  if (len <= step_size_)
    return to;
  return from + (delta / len) * step_size_;
}

std::vector<Node*> BP_RRTStar::nearNeighbors(const Node* n) const
{
  std::vector<Node*> nbrs;
  for (auto *m : tree_)
    if (dist(n->x, m->x) <= neighbor_radius_)
      nbrs.push_back(m);
  return nbrs;
}

void BP_RRTStar::chooseParent(Node* n, const std::vector<Node*>& nbrs)
{
  Node* best_parent = n->parent;
  double best_cost = n->cost;

  for (auto *m : nbrs)
  {
    if (!collisionFree(*m, *n)) continue;
    double c = m->cost + dist(m->x, n->x);
    if (c < best_cost)
    {
      best_cost   = c;
      best_parent = m;
    }
  }
  n->parent = best_parent;
  n->cost   = best_cost;
}

void BP_RRTStar::rewire(Node* n, const std::vector<Node*>& nbrs)
{
  for (auto *m : nbrs)
  {
    if (m == n->parent) continue;
    if (!collisionFree(*n, *m)) continue;
    double c_through_n = n->cost + dist(n->x, m->x);
    if (c_through_n < m->cost)
    {
      m->parent = n;
      m->cost   = c_through_n;
    }
  }
}

bool BP_RRTStar::collisionFree(const Node& a, const Node& b) const
{
  // ---- your convex‐hull or other checker goes here ----
  return true;
}

double BP_RRTStar::dist(const Eigen::Vector3d& a,
                        const Eigen::Vector3d& b) const
{
  return (a - b).norm();
}

std::vector<Node*> BP_RRTStar::extractPath() const
{
  std::vector<Node*> path;
  // find the final goal in the tree
  Node* cur = nullptr;
  for (auto *n : tree_)
    if (n == goal_) { cur = n; break; }

  // back‐chain:
  while (cur)
  {
    path.push_back(cur);
    cur = cur->parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<geometry_msgs::Pose> BP_RRTStar::getPathMsg() const
{
  auto nodes = extractPath();
  std::vector<geometry_msgs::Pose> poses;
  poses.reserve(nodes.size());
  for (auto *n : nodes)
  {
    geometry_msgs::Pose p;
    p.position.x = n->x.x();
    p.position.y = n->x.y();
    p.position.z = n->x.z();
    p.orientation.w = 1.0;  // identity
    poses.push_back(p);
  }
  return poses;
}
