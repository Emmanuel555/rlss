#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <rlss/internal/DiscreteSearch.hpp>
#include <rlss/OccupancyGrid.hpp>
#include <rlss/CollisionShapes/AlignedBoxCollisionShape.hpp>
#include <optional>
#include <memory>
#include <rlss/internal/Util.hpp>
#include <Eigen/Dense>

constexpr unsigned int DIM3 = 3;
constexpr unsigned int DIM = 2;

using namespace std;

//TEST_CASE("discrete search in 2d", "internal::DiscreteSearch") {
TEST_CASE("discrete search in 3D", "internal::DiscreteSearch") {
    using OccupancyGrid = rlss::OccupancyGrid<double, DIM3>;
    //using OccupancyGrid3D = rlss::OccupancyGrid<double, 3U>;
    using Coordinate = OccupancyGrid::Coordinate;
    using Index = OccupancyGrid::Index;
    using AlignedBox = OccupancyGrid::AlignedBox;
    using AlignedBoxCollisionShape = rlss::AlignedBoxCollisionShape<double, DIM3>;
    using CollisionShape = rlss::CollisionShape<double, DIM3>;
    using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<double, DIM3>;
    using VectorDIM = rlss::internal::VectorDIM<double, DIM3>;
    using VectorDIM3 = rlss::internal::VectorDIM<double, DIM3>;
    //using TestVector = rlss::internal::VectorDIM<double, 2U>;
    using Eigen::Vector4d;

    //OccupancyGrid grid(Coordinate(0.5, 0.5));
    //grid.setOccupancy(Index(1,5));
    //grid.setOccupancy(Index(1,4));
    //grid.setOccupancy(Index(2,1));
    //grid.setOccupancy(Index(3,2));
    //grid.setOccupancy(Index(3,1));
    //grid.setOccupancy(Index(5,1));

    OccupancyGrid grid3D(Coordinate(0.5, 0.5, 0.5)); // per grid size
    grid3D.setOccupancy(Coordinate(0.034,0.045,-0.067));
    //grid3D.setOccupancy(Index(1,4,0.5));
    //grid3D.removeOccupancy(Coordinate(0.034,0.045,-0.067));
    /*grid3D.setOccupancy(Index(2,1,0.5));
    grid3D.setOccupancy(Index(3,2));
    grid3D.setOccupancy(Index(3,1));
    grid3D.setOccupancy(Index(5,1,0.5));*/


    // grid.setOccupancy(Index(1,3));
    // grid.setOccupancy(Index(2,4));
    // grid.setOccupancy(Index(3,4));
    // grid.setOccupancy(Index(4,5));
    // grid.setOccupancy(Index(5,6));


    //auto al_collision_shape = std::make_shared<AlignedBoxCollisionShape>(AlignedBox(VectorDIM(-0.35, -0.35, -0.35), VectorDIM(0.35, 0.35, 0.35))); // add in the 3D to do* 
    //auto collision_shape = std::static_pointer_cast<CollisionShape>(al_collision_shape);
    //Coordinate start_position(0.74,0.75,0.0);
    //Coordinate goal_position(3.76,3.75,0.0);

    auto al_collision_shape = make_shared<AlignedBoxCollisionShape>(AlignedBox(VectorDIM3(-0.1, -0.1, 0.0), VectorDIM3(1.0, 1.0, 1.0))); // add in the 3D to do* 
    auto collision_shape = static_pointer_cast<CollisionShape>(al_collision_shape);
    Coordinate start_position(0.034,0.045,-0.067); // starting coordinate has to ensure that space is accounted for the collision shapes
    Coordinate goal_position(9.0,0.0,1.5);

    //AlignedBox workspace(VectorDIM(0, 0, 0), VectorDIM(4.5, 4.5, 4.5));
    AlignedBox workspace(VectorDIM3(-50, -50, -1), VectorDIM3(50.0, 50.0, 50.0));

    auto result = rlss::internal::discreteSearch<double, DIM3>(
                            start_position, 
                            goal_position, 
                            grid3D, 
                            workspace, 
                            collision_shape
    );

    //REQUIRE(result != std::nullopt);
    //std::cout << TestVector(1.0,1.0) << std::endl;
    StdVectorVectorDIM result_vector = *result;
    cout << result_vector.size() << endl;
    std::cout << "it should be occupied right?" << "  " << grid3D.isOccupied(Index(0.034,0.045,-0.067)) << std::endl;
    std::cout << "it should be occupied right?" << "  " << grid3D.isOccupied(Index(1,4,0.5)) << std::endl;
    //grid3D.removeOccupancy(Index(1,5,0.5));
    //std::cout << grid3D.isOccupied(Index(1,5,0.5)) << std::endl;
    std::cout << workspace.contains(start_position) << std::endl;
    //cout << grid3D.getCenter(Index(1.0,1.0)) << endl;
    //cout << grid3D.getIndex(Coordinate(0.3,4.0,0.5)) << endl;
    //cout << VectorDIM3(0.1, 0.1, 0.1) << endl;
    //cout << grid3D.getNeighbors(Index(1.0,1.0,0.5)).size() << endl;
    
    //cout << "1 " << grid3D.getNeighbors(Index(1.0,1.0,0.0))[0] << endl;
    //cout << "2 " << grid3D.getNeighbors(Index(1.0,1.0,0.0))[1] << endl;
    //cout << "3 " << grid3D.getNeighbors(Index(1.0,1.0,0.0))[2] << endl;
    //cout << "4 " << grid3D.getNeighbors(Index(1.0,1.0,0.0))[3] << endl;
    //cout << "5 " << grid3D.getNeighbors(Index(1.0,1.0,0.0))[4] << endl;
    //cout << "6 " << grid3D.getNeighbors(Index(1.0,1.0,0.0))[5] << endl;
    //cout << grid.getNeighbors(Index(1.0,1.0))[1] << endl;
    //cout << grid.getNeighbors(Index(1.0,1.0))[2] << endl;
    //cout << grid.getNeighbors(Index(1.0,1.0))[3] << endl;



    result_vector = rlss::internal::firstSegmentFix<double, DIM3>(result_vector);
    cout << result_vector[0].norm() << endl;
    cout << result_vector[1] << endl;
    cout << result_vector[2] << endl;


    auto total_path_length = 0.0;
    for(std::size_t i = 0; i < result_vector.size() - 1; i++) {
        total_path_length
                += (result_vector[i+1] - result_vector[i]).norm(); // norm refers to magnitude
        //cout << (result_vector[i+1] - result_vector[i]).norm() << endl;
        //cout << total_path_length << endl; //2.705
    }

    //cout << result_vector.size() << endl; // discete search soln. = 3
    //cout << total_path_length << endl; // total path length norm = 2.7074
    //cout << result_vector[0] << endl;
    //cout << result_vector[1] << endl;
    //cout << result_vector[2] << endl;
    vector<int> num_pieces(4);
    num_pieces[0] = 2;
    //cout << num_pieces[0] << endl;    

    //cout << total_path_length;

    //REQUIRE(result_vector.size() == 3);
    //REQUIRE((result_vector[0] - VectorDIM(0.74, 0.75)).squaredNorm() < 1e-9);
    //REQUIRE((result_vector[1] - VectorDIM(1.25, 0.75)).squaredNorm() < 1e-9);
    //REQUIRE((result_vector[2] - VectorDIM(3.75, 0.75)).squaredNorm() < 1e-9);
    //REQUIRE((result_vector[3] - VectorDIM(3.76, 3.75)).squaredNorm() < 1e-9);
}
