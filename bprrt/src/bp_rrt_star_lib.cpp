#include "bprrt/bp_rrt_star_lib.hpp"
#include <cstdlib>
#include <ctime>
#include <limits>
#include <algorithm>

BP_RRTStar::BP_RRTStar(const Eigen::Vector3d& start,
                       const Eigen::Vector3d& goal,
                       const Bounds&         bounds,
                       double                step_size,
                       int                   max_iters,
                       double                goal_bias,
                       double                neighbor_radius,
                       rclcpp::Logger        logger)
  : bounds_(bounds)
  , step_size_(step_size)
  , max_iters_(max_iters)
  , goal_bias_(goal_bias)
  , neighbor_radius_(neighbor_radius)
  , logger_(logger)
{
  std::srand(std::time(nullptr));
  tree_.push_back(std::make_unique<Node>(start));
  goal_uptr_ = std::make_unique<Node>(goal);
  goal_ptr_  = goal_uptr_.get();
}

bool BP_RRTStar::plan()
{
  for (int i = 0; i < max_iters_; ++i)
  {
    auto x_rand_up = sampleRandom();
    Node* x_rand   = x_rand_up.get();

    Node* x_near   = nearestNeighbor(x_rand->x);
    Eigen::Vector3d x_new_pt = steer(x_near->x, x_rand->x);

    auto x_new_up = std::make_unique<Node>(x_new_pt);
    Node* x_new   = x_new_up.get();
    x_new->parent = x_near;
    x_new->cost   = x_near->cost + dist(x_near->x, x_new_pt);

    if (!collisionFree(*x_near, *x_new))
      continue;

    auto nbrs = nearNeighbors(x_new);
    chooseParent(x_new, nbrs);
    tree_.push_back(std::move(x_new_up));
    rewire(x_new, nbrs);

    if (dist(x_new->x, goal_ptr_->x) <= step_size_ &&
        collisionFree(*x_new, *goal_ptr_))
    {
      RCLCPP_INFO(logger_, "BP-RRT* connected to goal at iter %d, cost %.3f",
                  i, x_new->cost + dist(x_new->x, goal_ptr_->x));
      goal_ptr_->parent = x_new;
      goal_ptr_->cost   = x_new->cost + dist(x_new->x, goal_ptr_->x);
      tree_.push_back(std::move(goal_uptr_));
      return true;
    }
  }

  RCLCPP_WARN(logger_, "BP-RRT* did not connect in %d iters", max_iters_);
  return false;
}

std::unique_ptr<Node> BP_RRTStar::sampleRandom() const
{
  double r = double(std::rand())/RAND_MAX;
  if (r < goal_bias_)
    return std::make_unique<Node>(goal_ptr_->x);

  Eigen::Vector3d p;
  p.x() = bounds_.low.x() + (bounds_.high.x() - bounds_.low.x()) * (double(std::rand()) / RAND_MAX);
  p.y() = bounds_.low.y() + (bounds_.high.y() - bounds_.low.y()) * (double(std::rand()) / RAND_MAX);
  p.z() = bounds_.low.z() + (bounds_.high.z() - bounds_.low.z()) * (double(std::rand()) / RAND_MAX);
  return std::make_unique<Node>(p);
}

Node* BP_RRTStar::nearestNeighbor(const Eigen::Vector3d& x_rand) const
{
  Node* best = nullptr;
  double bestD = std::numeric_limits<double>::infinity();
  for (auto& up : tree_)
  {
    double d = dist(up->x, x_rand);
    if (d < bestD) { bestD = d; best = up.get(); }
  }
  return best;
}

std::vector<Node*> BP_RRTStar::nearNeighbors(Node* n) const
{
  std::vector<Node*> nbrs;
  for (auto& up : tree_)
    if (dist(n->x, up->x) <= neighbor_radius_)
      nbrs.push_back(up.get());
  return nbrs;
}

void BP_RRTStar::chooseParent(Node* n, const std::vector<Node*>& nbrs)
{
  Node* bestP = n->parent;
  double cMin = n->cost;
  for (auto* m : nbrs)
  {
    if (!collisionFree(*m, *n)) continue;
    double c = m->cost + dist(m->x, n->x);
    if (c < cMin)
    {
      cMin  = c;
      bestP = m;
    }
  }
  n->parent = bestP;
  n->cost   = cMin;
}

void BP_RRTStar::rewire(Node* n, const std::vector<Node*>& nbrs)
{
  for (auto* m : nbrs)
  {
    if (m == n->parent) continue;
    if (!collisionFree(*n, *m)) continue;
    double cNew = n->cost + dist(n->x, m->x);
    if (cNew < m->cost)
    {
      m->parent = n;
      m->cost   = cNew;
    }
  }
}

bool BP_RRTStar::collisionFree(const Node& a, const Node& b) const
{
  return true;  // still placeholder
}

Eigen::Vector3d BP_RRTStar::steer(const Eigen::Vector3d& from,
                                  const Eigen::Vector3d& to) const
{
  Eigen::Vector3d d = to - from;
  double len = d.norm();
  if (len <= step_size_) return to;
  return from + (d / len) * step_size_;
}

double BP_RRTStar::dist(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const
{
  return (a - b).norm();
}

std::vector<Node*> BP_RRTStar::extractPath() const
{
  std::vector<Node*> path;
  Node* cur = nullptr;
  for (auto& up : tree_)
    if (up.get() == goal_ptr_) { cur = goal_ptr_; break; }
  while (cur)
  {
    path.push_back(cur);
    cur = cur->parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<geometry_msgs::msg::Pose> BP_RRTStar::getPathMsg() const
{
  auto nodes = extractPath();
  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(nodes.size());
  for (auto* n : nodes)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = n->x.x();
    p.position.y = n->x.y();
    p.position.z = n->x.z();
    p.orientation.w = 1.0;
    poses.push_back(p);
  }
  return poses;
}
