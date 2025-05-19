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


#include "obstacle_manager.hpp"


ObstacleManager::ObstacleManager() {
}

ObstacleManager::ObstacleManager(std::vector<std::vector<double>> workspaceCorners) {
    this->workspaceCorners = workspaceCorners;
}

ObstacleManager::ObstacleManager(std::vector<double> startPoint, std::vector<double> endPoint) {
    this->workspaceCorners = {startPoint, endPoint};
}

ObstacleManager::~ObstacleManager() {
    // Destructor implementation (if needed)
}

/*
* ObstacleManager::workspaceSplit()
*
* Estimates the radius of a bounding sphere for objects based on the distance from 
*    their centre to the furthest corner of their bounding box
*
* [in|out]* objects = empty vector of object data
* [in] objCentre = Cartesian coordinate values {x, y, z}
* [in] objBoxCorners = vector of coordinate values for bounding box corners
*/
void ObstacleManager::estimateRadius(std::vector<std::vector<double>>* objects, std::vector<double> objCentre, std::vector<std::vector<double>> objBoxCorners)
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

/*
* ObstacleManager::objectSpheres()
*
* Creates point clouds of all object spheres
*
* [in] objects = vector of all object sphere's defining features in the {centreX, centreY, centreZ, radius} format
* 
* [out] objectSpheres = vector of object sphere point clouds
*/
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
        int count = 0;
        for(auto point : objectSphere) {
            Point_3 p = Point_3(point.x() + object[0], point.y() + object[1], point.z() + object[2]);
            objectSphere.at(count) = p;
            count++;
        }
        objectSpheres.push_back(objectSphere);
    }
    return objectSpheres;
}

/*
* ObstacleManager::regularPlacement()
*
* Uniformally distributes points around an object sphere's radius
*
* [in|out]* cloud = point cloud vector
* [in] radius = radius of current object sphere
* [in] num_point = number of points in the point cloud
* 
*/
void ObstacleManager::regularPlacement(std::vector<Point_3>* cloud, const double radius, int num_point) {
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
            Point_3 p(radius * sin(phi) * cos(theta), radius * sin(phi) * sin(theta), radius * cos(phi));
            cloud->push_back(p);
        }
    }
}

/*
* ObstacleManager::createConvexHulls()
*
* Creates CGAL::Polyhedron_3 meshes from each object sphere's surface points
*
* [in] objectSpheres = vector of all object's surface point clouds
* 
* [out] convexHulls = vector of object sphere meshes
*/
std::vector<Polyhedron_3> ObstacleManager::createConvexHulls(std::vector<std::vector<Point_3>> objectSpheres) {
    std::vector<Polyhedron_3> convexHulls;
    for(auto objectSphere : objectSpheres) {
        Polyhedron_3 P;
        CGAL::convex_hull_3(objectSphere.begin(), objectSphere.end(), P);
        convexHulls.push_back(P);
        //std::cout << CGAL::Polygon_mesh_processing::volume(P) << std::endl; 
    }
    return convexHulls;
}

/*
* ObstacleManager::setWorkspaceCorners()
*
* Setter for workspaceCorners in case not provided at construction
*
* [in] workspaceCorners = vector of coordinate points in {x, y, z} format
*/
void ObstacleManager::setWorkspaceCorners(std::vector<std::vector<double>> workspaceCorners) {
    this->workspaceCorners = workspaceCorners;
}

/*
* ObstacleManager::getWorkspaceCorners()
*
* Getter to check current workspace's corners
* 
* [out] workspaceCorners = vector of coordinate points in {x, y, z} format
*/
std::vector<std::vector<double>> ObstacleManager::getWorkspaceCorners() {
    return this->workspaceCorners;
}

/*
* ObstacleManager::defineWorkspaceSplit()
*
* Defines and stores values for cell minimum and maximum coordinates based on the workspace's corners provided at construction.
* Also calculates the starting volumes of each cell
* 
* [out] splitWorkspace = vector of coordinates in {{x0, x1, x2, x3}, {y0, y1, y2, y3}, {z0, z1, z2, z3}} format
*/
std::vector<std::vector<double>> ObstacleManager::defineWorkspaceSplit() {
    std::vector<std::vector<double>> splitWorkspace = {{}, {}, {}};
    // Define the split workspace based on the corners
    double x_min = this->workspaceCorners.at(0).at(0);
    double x_max = this->workspaceCorners.at(1).at(0);
    double y_min = this->workspaceCorners.at(0).at(1);
    double y_max = this->workspaceCorners.at(1).at(1);
    double z_min = this->workspaceCorners.at(0).at(2);
    double z_max = this->workspaceCorners.at(1).at(2);

    cellVolume = (x_max - x_min) * (y_max - y_min) * (z_max - z_min) / 27.0; // Volume of each cell
    
    for(int i = 0; i < 4; i++){
        splitWorkspace.at(0).push_back(x_min + ((i * (x_max - x_min)) / 3.0));
    }
    for(int j = 0; j < 4; j++){
        splitWorkspace.at(1).push_back(y_min + ((j * (y_max - y_min)) / 3.0));
    }
    for(int k = 0; k < 4; k++){
        splitWorkspace.at(2).push_back(z_min + ((k * (z_max - z_min)) / 3.0));
    }
    return splitWorkspace;
}

/*
* ObstacleManager::defineWorkspaceCubes()
*
* Creates Iso_cuboid_3's from bounding boxes (Bbox_3) to represent the workspace cells 
* 
* [out] cubes = vector of each cell's representative cubes (Iso_cuboid_3's)
*/
std::vector<Cube> ObstacleManager::defineWorkspaceCubes(){
    std::vector<Cube> cubes;
    std::vector<std::vector<double>> workspacePoints = defineWorkspaceSplit();

    for(int i = 0; i < 27; i++){
        CGAL::Bbox_3 bbox(workspacePoints.at(0).at(cubePointIdx.at(i).at(0).at(0)),
                          workspacePoints.at(1).at(cubePointIdx.at(i).at(1).at(0)),
                          workspacePoints.at(2).at(cubePointIdx.at(i).at(2).at(0)),
                          workspacePoints.at(0).at(cubePointIdx.at(i).at(0).at(1)),
                          workspacePoints.at(1).at(cubePointIdx.at(i).at(1).at(1)),
                          workspacePoints.at(2).at(cubePointIdx.at(i).at(2).at(1)));
        Cube cube = Cube(bbox);
        cubes.push_back(cube);
    }
    return cubes;
}

/*
* ObstacleManager::workspaceSplit()
*
* Splits and keeps the slices of sphere objects in each of the 27 workspace cells
*
* [in] Polys = vector of all object's CGAL::Polyhedron_3 meshes
* [in] cubes = vector of each cell's representative cubes (Iso_cuboid_3's)
* 
* [out] slices = vector of object parts and their cell indices
*/
std::vector<Slice> ObstacleManager::workspaceSplit(std::vector<Polyhedron_3> Polys, std::vector<Cube> cubes) {
    std::vector<Slice> slices;
    Slice slice;
    for(auto P : Polys){
        slice.poly = P;
        for(int i = 0; i < cubes.size(); i++){
            CGAL::Polygon_mesh_processing::clip(slice.poly, cubes.at(i), CGAL::parameters::clip_volume(true));
            if(!slice.poly.empty()){ 
                slice.volumeIdx = i;     
                slices.push_back(slice);
            }    
            slice.poly = P;
        }
    }    
    return slices;
}

/*
* ObstacleManager::calculateFreeVolumes()
*
* Calculates the remaining volume of the 27 cells of the current workspace, subtracting object slice volume.
*
* [in] objectVolumes = vector of object mesh slices and their cell indices
* 
* [out] volumes = vector of free (remaining) volumes in <double> format
*/
std::vector<double> ObstacleManager::calculateFreeVolumes(std::vector<Slice> objectVolumes){
    //std::cout << "cellVolume before: " + std::to_string(this->cellVolume) << std::endl;
    std::vector<double> volumes;
    for(int i = 0; i < 27; i++){
        volumes.push_back(cellVolume);
    }
    
    for(auto slice : objectVolumes){
        //std::cout << "obj Volume for " + std::to_string(slice.volumeIdx) + ": " + std::to_string(CGAL::Polygon_mesh_processing::volume(slice.poly)) << std::endl;
        volumes.at(slice.volumeIdx) -= CGAL::Polygon_mesh_processing::volume(slice.poly);
    }

    // for(int i = 0; i < 27; i++){
    //     std::cout << "cellVolume" + std::to_string(i) + "after: " + std::to_string(volumes.at(i)) << std::endl;
    // }

    return volumes;
}

/*
* ObstacleManager::samplingProbability()
*
* Calculates the distance weight and final sampling probabilities of each of the 27 cells of the current workspace.
*
* [in] volumes = vector of each cell's free volume values in <double> format
* 
* [out] probabilities = vector of sampling probabilities in <double> format
*/
std::vector<double> ObstacleManager::samplingProbability(std::vector<double> volumes){
    double distanceWeight;
    double pwCoefficent;
    std::vector<double> probabilities;
    double probSum = 0.0;
    for(int i = 0; i < volumes.size(); i++){
        distanceWeight = (1.0/(4.0*sqrt(2*M_PI))) * exp(-(pow(stepsToEnd.at(i), 2)/32.0));
        //std::cout << "distanceWeight for volume" + std::to_string(i) + ": " + std::to_string(distanceWeight) << std::endl;
        pwCoefficent = volumes.at(i) * distanceWeight;
        probSum += pwCoefficent;
        probabilities.push_back(pwCoefficent);
    }
    //std::cout << "probSum: " + std::to_string(probSum) << std::endl;
    for(int i = 0; i < probabilities.size(); i++){
        probabilities.at(i) = (probabilities.at(i)/probSum);
    }

    return probabilities;
}

/*
* ObstacleManager::splitRunner()
*
* Workspace splitter bundle
*
* [in] objCentres = vector of object centres in {x, y, z} format
* [in] objBoxCorners = vector of object bounding box corners. Can provide any number of corners in {x, y, z} format
* [in] cubes = vector of Cube (CGAL::Exact_predicates_inexact_constructions_kernel::Iso_cuboid_3) objects, represents workspace cells.
*           ->  Cube vector needs to be obtained once outside of this function through ObstacleManager::defineWorkspaceCubes();
* 
* [out] probabilities = vector of sampling probabilities in <double> format
*/
std::vector<double> ObstacleManager::splitRunner(std::vector<std::vector<double>> objCentres,
                                             std::vector<std::vector<std::vector<double>>> objBoxCorners, std::vector<Cube> cubes){
    
    std::vector<std::vector<double>> objects;
    for(int i = 0; i < objCentres.size(); i++){
        this->estimateRadius(&objects, objCentres.at(i), objBoxCorners.at(i));
    }

    std::vector<Polyhedron_3> convexHulls = this->createConvexHulls(this->objectSpheres(objects));
    std::vector<Slice> slices = this->workspaceSplit(convexHulls, cubes);
    std::vector<double> probabilities = this->samplingProbability(this->calculateFreeVolumes(slices));

    return probabilities;
}



/*
* ObstacleManager::splitRunnerMeshOut() 
*
* Workspace splitter bundle variation with Mesh Output (Object Spheres & slices)
*
* [in] objCentres = vector of object centres in {x, y, z} format
* [in] objBoxCorners = vector of object bounding box corners. Can provide any number of corners in {x, y, z} format
* [in] cubes = vector of Cube (CGAL::Exact_predicates_inexact_constructions_kernel::Iso_cuboid_3) objects, represents workspace cells.
*           ->  Cube vector needs to be obtained once outside of this function through ObstacleManager::defineWorkspaceCubes();
* 
* [out] probabilities = vector of sampling probabilities in <double> format
* [file out]          = Sphere and slice meshes output into the package's "meshes" folder in .off formats
*/
std::vector<double> ObstacleManager::splitRunnerMeshOut(std::vector<std::vector<double>> objCentres,
                                             std::vector<std::vector<std::vector<double>>> objBoxCorners, std::vector<Cube> cubes){
    
    std::vector<std::vector<double>> objects;
    for(int i = 0; i < objCentres.size(); i++){
        this->estimateRadius(&objects, objCentres.at(i), objBoxCorners.at(i));
    }

    std::vector<Polyhedron_3> convexHulls = this->createConvexHulls(this->objectSpheres(objects));

    for(int i = 0; i < convexHulls.size(); i++){
        std::string fname = "../meshes/sphere" + std::to_string(i) + ".off";
        CGAL::IO::write_OFF(fname, convexHulls.at(i));
    }

    std::vector<Slice> slices = this->workspaceSplit(convexHulls, cubes);

    for(int i = 0; i < slices.size(); i++){
        std::string fname = "../meshes/slice" + std::to_string(i) + ".off";
        CGAL::IO::write_OFF(fname, slices.at(i).poly);
    }

    std::vector<double> probabilities = this->samplingProbability(this->calculateFreeVolumes(slices));

    return probabilities;
}
