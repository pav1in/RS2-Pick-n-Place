#ifndef OBJ_H
#define OBJ_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <CGAL/Iso_cuboid_3.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/IO/OFF.h>
#include <CGAL/Surface_mesh/IO/OFF.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/draw_polyhedron.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/box_intersection_d.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
typedef K::Point_3                                Point_3;
typedef CGAL::Surface_mesh<Point_3>               Surface_mesh;
typedef K::Plane_3                                Plane;
typedef K::Iso_cuboid_3                           Cube;

struct Slice {
    Polyhedron_3 poly;
    int volumeIdx;
};

class ObstacleManager {
    public:
        ObstacleManager();
        ObstacleManager(std::vector<double> startPoint, std::vector<double> endPoint);
        ObstacleManager(std::vector<std::vector<double>> workspaceCorners);
        ~ObstacleManager();

        std::vector<double> splitRunner(std::vector<std::vector<double>> objCentres, 
                                std::vector<std::vector<std::vector<double>>> objBoxCorners, std::vector<Cube> cubes);
        
        std::vector<double> splitRunnerMeshOut(std::vector<std::vector<double>> objCentres, 
                                std::vector<std::vector<std::vector<double>>> objBoxCorners, std::vector<Cube> cubes);

        void estimateRadius(std::vector<std::vector<double>>* objects, std::vector<double> centre, std::vector<std::vector<double>> corners);
        
        std::vector<std::vector<Point_3>> objectSpheres(std::vector<std::vector<double>> objects);  
        std::vector<Polyhedron_3> createConvexHulls(std::vector<std::vector<Point_3>> objectSpheres);
        void setWorkspaceCorners(std::vector<std::vector<double>> workspaceCorners);
        std::vector<std::vector<double>> getWorkspaceCorners();
        std::vector<Cube> defineWorkspaceCubes();
        std::vector<Slice> workspaceSplit(std::vector<Polyhedron_3> Polys, std::vector<Cube> cubes);
        std::vector<std::vector<double>> defineWorkspaceSplit();
        std::vector<double> calculateFreeVolumes(std::vector<Slice> objectVolumes);
        std::vector<double> samplingProbability(std::vector<double> volumes);


    private:
        void regularPlacement(std::vector<Point_3>* cloud, const double radius, int num_point);
        std::vector<std::vector<double>> workspaceCorners;
        std::vector<Plane> workspacePlanes;
        double cellVolume;
        const std::vector<double> stepsToEnd = {6.0, 5.0, 4.0, 5.0, 4.0, 3.0, 4.0, 3.0, 2.0,
                                          5.0, 4.0, 3.0, 4.0, 3.0, 2.0, 3.0, 2.0, 1.0,
                                          4.0, 3.0, 2.0, 3.0, 2.0, 1.0, 2.0, 1.0, 0.0};

        const std::vector<std::vector<std::vector<int>>> cubePointIdx = {{{0,1},{0,1},{0,1}}, {{1,2},{0,1},{0,1}}, {{2,3},{0,1},{0,1}}, 
                                                                         {{0,1},{1,2},{0,1}}, {{1,2},{1,2},{0,1}}, {{2,3},{1,2},{0,1}}, 
                                                                         {{0,1},{2,3},{0,1}}, {{1,2},{2,3},{0,1}}, {{2,3},{2,3},{0,1}},
                                                                         {{0,1},{0,1},{1,2}}, {{1,2},{0,1},{1,2}}, {{2,3},{0,1},{1,2}}, 
                                                                         {{0,1},{1,2},{1,2}}, {{1,2},{1,2},{1,2}}, {{2,3},{1,2},{1,2}}, 
                                                                         {{0,1},{2,3},{1,2}}, {{1,2},{2,3},{1,2}}, {{2,3},{2,3},{1,2}},
                                                                         {{0,1},{0,1},{2,3}}, {{1,2},{0,1},{2,3}}, {{2,3},{0,1},{2,3}}, 
                                                                         {{0,1},{1,2},{2,3}}, {{1,2},{1,2},{2,3}}, {{2,3},{1,2},{2,3}}, 
                                                                         {{0,1},{2,3},{2,3}}, {{1,2},{2,3},{2,3}}, {{2,3},{2,3},{2,3}}};
}; 

#endif // OBJ_H