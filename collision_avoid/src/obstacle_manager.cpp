// For each detected object, you will need to:

// Determine the center of the object in 3D space (this is typically provided as part of the detection or can be computed if you have the bounding box and depth information).

// Estimate a radius that safely bounds the object. This can be based on known object dimensions or derived from the detected bounding box size.

// Package these as “obstacle spheres” that your planner can use in its collision checks.

// get obstacle spheres (visualise)
// uniformly distribute points (point cloud) around the surface of each sphere 
// use convex hull package to transform obstacle spheres into point groups/clouds
// split working area into 27 equal volumes 
// divide obstacle clouds into the volumes & compute obstacle volumes in each area
// calculate free volume in each of the 27 volumes (Vfree = Vi - Vobs)
// return obstacle spheres

#include <ros/ros.h>
#include <string>
#include <vector>
#include <std_msgs>
#include <visualization_msgs>
#include <geometry_msgs>

#include <sstream>

class ObstacleManager {
    public:
        double estimateRadius(std::vector<double> centre);
        std::vector<double> objectSpheres(std::vector<double> centre, double radius);  

    private:
        std::vector<double> getObjectCentre();
}

double ObstacleManager::estimateRadius(std::vector<double> centre)
{
    double radius;
    return radius;
}

std::vector<double> ObstacleManager::objectSpheres(std::vector<double> centre, double radius)
{
    std::vector<double> objectSphere;
    return objectSphere;
}

std::vector<double> ObstacleManager::getObjectCentre()
{
    std::vector<double> centre;
    return centre;
}
