#ifndef OBJ_H
#define OBJ_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/draw_polyhedron.h>
#include <CGAL/Polygon_mesh_processing/slice.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/volume.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/box_intersection_d.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
typedef K::Point_3                                Point_3;
typedef CGAL::Surface_mesh<Point_3>               Surface_mesh;
typedef K::Plane_3                                Plane;

class ObstacleManager {
    public:
        ObstacleManager(std::vector<double> workspaceCorners);
        void estimateRadius(std::vector<std::vector<double>>::Ptr objects, std::vector<double> centre, std::vector<std::vector<double>> corners);
        std::vector<std::vector<Point_3>> objectSpheres(std::vector<std::vector<double>> objects);  
        std::vector<Polyhedron_3> createConvexHulls(std::vector<std::vector<Point_3>> objectSpheres);
        void visualizeSpheres(std::vector<std::vector<Point_3>> objectSpheres);
        void setWorkspaceCorners(std::vector<double> workspaceCorners);
        std::vector<double> getWorkspaceCorners();
        std::vector<Plane> defineWorkspacePlanes(std::vector<double> workspaceCorners);
        std::vector<Polyhedron_3> workspaceSplit(Polyhedron_3 P, std::vector<Plane> planes);
        std::vector<std::vector<double>> defineWorkspaceSplit();
        std::vector<double> calculateFreeVolumes(std::vector<Polyhedron_3> objectVolumes, std::vector<std::vector<double>> splitWorkspace);


    private:
        void regularPlacement(std::vector<std::vector<Point_3>>& cloud, const double radius, int num_point);
        //std::vector<double> getObjectCentre();
        std::vector<std::vector<double>> workspaceCorners;
        std::vector<Plane> workspacePlanes;
        double cellVolume;
}; 

#endif // OBJ_H