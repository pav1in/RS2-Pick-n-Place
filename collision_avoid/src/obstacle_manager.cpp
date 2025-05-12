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

#include "obstacle_manager.hpp"


ObstacleManager::ObstacleManager(std::vector<double> workspaceCorners) {
    this->workspaceCorners = workspaceCorners;
}

ObstacleManager::~ObstacleManager() {
    // Destructor implementation (if needed)
}

void ObstacleManager::estimateRadius(std::vector<std::vector<double>>::Ptr objects, std::vector<double> objCentre, std::vector<std::vector<double>> objBoxCorners)
{
    double radius = 0.0;
    // get the furthest distance from the centre (x, y, z) and the bounding box corner
    for(int i = 0; i < objBoxCorners.size(); i++){
        double x = objBoxCorners[i][0] - objCentre[0];
        double y = objBoxCorners[i][1] - objCentre[1];
        double z = objBoxCorners[i][2] - objCentre[2];
        double distance = sqrt(x*x + y*y + z*z);
        if (distance > radius) {
            radius = distance;
        }
    }
    // add a safety margin to the radius
    radius += 0.001; // 10mm safety margin
    std::vector<double> object = {objCentre[0], objCentre[1], objCentre[2], radius};
    // add the object to the list of objects
    objects->push_back(object);
}

std::vector<std::vector<Point_3>> ObstacleManager::objectSpheres(const std::vector<std::vector<double>> objects)
{
    std::vector<std::vector<Point_3>> objectSpheres;

    // Create spheres around the object centres with the estimated radii
    for(auto object : objects){
        std::vector<Point_3> objectSphere;
        double radius = object[3]; // radius of the sphere
        int num_points = 200; // number of points to generate on the sphere surface
        regularPlacement(&objectSphere, radius, num_points); // generate points on the sphere surface

        // Transform the points to the object's centre
        for(auto& point : objectSphere) {
            point.x += object[0];
            point.y += object[1];
            point.z += object[2];
        }

        objectSpheres.push_back(objectSphere);
    }

    return objectSpheres;
}

void ObstacleManager::regularPlacement(std::vector<Point_3>& cloud, const double radius, int num_point) {
    float a = 4.0 * M_PI * 1.0 / static_cast<float>(num_point);
    float d = sqrt(a);
    size_t num_phi = round(M_PI / d);
    float d_phi = M_PI / static_cast<float>(num_phi);
    float d_theta = a / d_phi;
    for (int m = 0; m < num_phi; ++m) {
        float phi = M_PI * (m + 0.5) / num_phi;
        size_t num_theta = round(2 * M_PI * sin(phi) / d_theta);
        for (int n = 0; n < num_theta; ++n) {
            float theta = 2 * M_PI * n / static_cast<float>(num_theta);
            Point_3 p;
            p.x = radius * sin(phi) * cos(theta);
            p.y = radius * sin(phi) * sin(theta);
            p.z = radius * cos(phi);
            cloud->push_back(p);
        }
    }
}


std::vector<Polyhedron_3> ObstacleManager::createConvexHulls(std::vector<std::vector<Point_3>> objectSpheres) {
    std::vector<Polyhedron_3> convexHulls;
    for(auto objectSphere : objectSpheres) {
        Polyhedron_3 P;
        CGAL::convex_hull_3(objectSphere.begin(), objectSphere.end(), P);
        convexHulls.push_back(P);
        std::cout << CGAL::Polygon_mesh_processing::volume(P) << std::endl; 
    }
    return convexHulls;
}

void ObstacleManager::setWorkspaceCorners(std::vector<double> workspaceCorners) {
    this->workspaceCorners = workspaceCorners;
}

std::vector<double> ObstacleManager::getWorkspaceCorners() {
    return this->workspaceCorners;
}

std::vector<std::vector<double>> ObstacleManager::defineWorkspaceSplit() {
    std::vector<std::vector<double>> splitWorkspace;
    Point_3 point;
    // Define the split workspace based on the corners
    double x_min = this->workspaceCorners.at(0).at(0);
    double x_max = this->workspaceCorners.at(3).at(0);
    double y_min = this->workspaceCorners.at(0).at(1);
    double y_max = this->workspaceCorners.at(3).at(1);
    double z_min = this->workspaceCorners.at(0).at(2);
    double z_max = this->workspaceCorners.at(3).at(2);

    cellVolume = (x_max - x_min) * (y_max - y_min) * (z_max - z_min) / 27.0; // Volume of each cell
    
    for(int i = 0; i < 3; i++){
        splitWorkspace.at(0).push_back(x_min + ((i * (x_max - x_min)) / 3));
    }
    for(int j = 0; j < 3; j++){
        splitWorkspace.at(1).push_back(y_min + ((j * (y_max - y_min)) / 3));
    }
    for(int k = 0; k < 3; k++){
        splitWorkspace.at(2).push_back(z_min + ((k * (z_max - z_min)) / 3));
    }
    return splitWorkspace;
}




// Create a way to split workspace using cubes 

std::vector<Plane> ObstacleManager::defineWorkspacePlanes(std::vector<std::vector<double>> workspacePoints) {
    std::vector<Plane> planes;

    // Define the planes of the workspace using the corners
    // Assuming the workspace is a rectangular prism, creates 12 planes
    // 4 planes for each face of the prism (x, y, z)
    for(auto point : workspacePoints.at(0)){
        Plane plane(1,0,0,-point);
        planes.push_back(plane); // x = point
    }
    for(auto point : workspacePoints.at(1)){
        Plane plane(0,1,0,-point);
        planes.push_back(plane); // y = point
    }
    for(auto point : workspacePoints.at(2)){
        Plane plane(0,0,1,-point);
        planes.push_back(plane); // z = point
    }
    
    CGAL::draw(planes); // Visualize the planes
    std::cout << "Planes: " << planes.size() << std::endl;
    return planes;
}

std::vector<Polyhedron_3> ObstacleManager::workspaceSplit(Polyhedron_3 P, std::vector<Plane> planes) {
    std::vector<Polyhedron_3> slices;
    for(auto plane : planes) {

        CGAL::Polygon_mesh_processing::slice(P, plane, std::back_inserter(slices));
        //
        // CGAL::Polygon_mesh_processing::clip(P, plane, std::back_inserter(slices)); // Clip the polyhedron with the plane
    }
    slices.push_back(P);
    CGAL::draw(slices); // Visualize the slices
    return slices;

}

std::vector<double> ObstacleManager::calculateFreeVolumes(std::vector<Polyhedron_3> objectVolumes, std::vector<std::vector<double>> splitWorkspace){
    std::vector<int> sliceVolumes;
    for(auto slice : objectVolumes){
        sliceVolumes.push_back(findVolumeIndex(&slice, &splitWorkspace));
    }

    std::vector<double> volumes;
    for(int i = 0; i < 26; i++){
        volumes.push_back(cellVolume);
    }
    
    for(int i = 0; i < 26; i++){
        int count = 0;
        for(auto volume : sliceVolumes){
            count++;
            if(volume == i){
                volumes.at(i) -= CGAL::Polygon_mesh_processing::volume(objectVolumes.at(count));
            }
        }
    }
}



int ObstacleManager::findVolumeIndex(const Polyhedron_3& slice, const std::vector<std::vector<double>>& splitWorkspace) {
    // Compute the bounding box of the polyhedron slice
    CGAL::Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(slice);

    // Iterate through the 27 volumes
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                // Get the boundaries of the current volume
                double x_min = splitWorkspace[0][i];
                double x_max = splitWorkspace[0][i + 1];
                double y_min = splitWorkspace[1][j];
                double y_max = splitWorkspace[1][j + 1];
                double z_min = splitWorkspace[2][k];
                double z_max = splitWorkspace[2][k + 1];

                // Check if the bounding box of the slice overlaps this volume
                if (bbox.xmin() >= x_min && bbox.xmax() <= x_max &&
                    bbox.ymin() >= y_min && bbox.ymax() <= y_max &&
                    bbox.zmin() >= z_min && bbox.zmax() <= z_max) {
                    // Return the index of the volume (flattened 3D index)
                    return i + j*3 + k*9 ;
                }
            }
        }
    }

    // If no volume matches, return -1 (indicating an error)
    return -1;
}

void ObstacleManager::visualizeSpheres(std::vector<std::vector<Point_3>> objectSpheres){
    
}

// std::vector<double> ObstacleManager::getObjectCentre()
// {
//     std::vector<double> centre;
//     return centre;
// }
