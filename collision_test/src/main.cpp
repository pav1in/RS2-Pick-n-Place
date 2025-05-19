#include <obstacle_manager.hpp>

int main(int argc, char** argv) {

    // std::vector<std::vector<double>> workspaceCorners = {{3.0, 0.0, 0.0}, {3.0, 0.0, 3.0}, {3.0, 3.0, 0.0}, {3.0, 3.0, 3.0}, 
    //                                                     {0.0, 0.0, 0.0}, {0.0, 0.0, 3.0}, {0.0, 3.0, 0.0}, {0.0, 3.0, 3.0}};
    // ObstacleManager obMan = ObstacleManager(workspaceCorners);

    // for(int i = 0; i < 8; i++){
    //     std::cout << obMan.getWorkspaceCorners().at(i).at(0) << obMan.getWorkspaceCorners().at(i).at(1) << obMan.getWorkspaceCorners().at(i).at(2) << std::endl;
    // }

    // std::vector<double> testCentre = {1.5, 1.5, 1.5};
    // std::vector<std::vector<double>> testObjects;
    // // std::vector<std::vector<double>> testObjBoxCorners = {{1.5, 0, 1.5}, {0, 1.5, 1.5}, {3, 1.5, 1.5}, {1.5, 3, 1.5}};

    // // obMan.estimateRadius(&testObjects, testCentre, testObjBoxCorners);

    // // std::cout << testObjects.size() << std::endl;

    // testObjects.push_back({1.5,1.5,1.5,1.25});
    // std::vector<std::vector<Point_3>> testSpheres = obMan.objectSpheres(testObjects);
    // std::vector<Polyhedron_3> testConHulls = obMan.createConvexHulls(testSpheres);

    // CGAL::IO::write_OFF("../meshes/sphereMesh.off", testConHulls.at(0));

    // std::vector<Slice> slices = obMan.workspaceSplit(testConHulls.at(0));

    // std::cout << "workspace split" << std::endl;

    // for(int i = 0; i < slices.size(); i++){
    //     std::string fname = "../meshes/slice" + std::to_string(i) + ".off";
    //     CGAL::IO::write_OFF(fname, slices.at(i).poly);
    // }
    // std::cout << "Mesh files created" << std::endl;




    // RUNNER TEST
    std::vector<std::vector<double>> workspaceCorners = {{0.0, 0.0, 0.0}, {9.0, 9.0, 9.0}};
    ObstacleManager obMan = ObstacleManager(workspaceCorners);

    std::vector<Cube> cubes = obMan.defineWorkspaceCubes();

    std::vector<std::vector<double>> testCentres = {{1.5,1.5,1.5}, {6.0,4.0,3.0}};
    std::vector<std::vector<double>> testObjects;
    std::vector<std::vector<std::vector<double>>> testObjBoxCorners = {{{1.5, 0.0, 1.5}, {0.0, 1.5, 1.5}, {3.0, 1.5, 1.5}, {1.5, 3.0, 1.5}},
                                                                       {{5.0, 4.0, 2.0}, {7.0, 4.0, 4.0}}};
    
    std::vector<double> probabilities = obMan.splitRunnerMeshOut(testCentres, testObjBoxCorners, cubes);

    for(int i = 0; i < probabilities.size(); i++){
        std::cout << "Probability cell " + std::to_string(i) + ": " << probabilities.at(i) << std::endl;
    }

    return 0;
}