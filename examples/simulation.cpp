#include <splx/curve/PiecewiseCurve.hpp>
#include <boost/filesystem/operations.hpp>
#include <ios>
#include <rlss/CollisionShapes/AlignedBoxCollisionShape.hpp>
#include <rlss/OccupancyGrid.hpp>
#include <rlss/RLSS.hpp>
#include <iostream>
#include "../third_party/cxxopts.hpp"
#include "../third_party/json.hpp"
#include <fstream>
#include <boost/filesystem.hpp>
#include <splx/opt/PiecewiseCurveQPGenerator.hpp>
#include <splx/curve/Bezier.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <memory>
//#include <rlss/internal/LegacyJSONBuilder.hpp>
#include <rlss/internal/JSONBuilder.hpp>
#include <rlss/TrajectoryOptimizers/RLSSHardOptimizer.hpp>
#include <rlss/TrajectoryOptimizers/RLSSSoftOptimizer.hpp>
#include <rlss/TrajectoryOptimizers/RLSSHardSoftOptimizer.hpp>
#include <rlss/DiscretePathSearchers/RLSSDiscretePathSearcher.hpp>
#include <rlss/ValidityCheckers/RLSSValidityChecker.hpp>
#include <rlss/GoalSelectors/RLSSGoalSelector.hpp>
#include <random>
#include <rlss/internal/Util.hpp>

constexpr unsigned int DIM = SIMULATION_DIMENSION; // must be provided as define

namespace fs = boost::filesystem;
using RLSS = rlss::RLSS<double, DIM>;
using OccupancyGrid = rlss::OccupancyGrid<double, DIM>;
using AlignedBoxCollisionShape = rlss::AlignedBoxCollisionShape<double, DIM>;
using CollisionShape = rlss::CollisionShape<double, DIM>;
using StdVectorVectorDIM = OccupancyGrid::StdVectorVectorDIM;
using VectorDIM = OccupancyGrid::VectorDIM;
using MatrixDIMDIM = rlss::internal::MatrixRC<double, DIM, DIM>;
using PiecewiseCurveQPGenerator
= splx::PiecewiseCurveQPGenerator<double, DIM>;
using PiecewiseCurve = splx::PiecewiseCurve<double, DIM>;
using Bezier = splx::Bezier<double, DIM>;
using AlignedBox = OccupancyGrid::AlignedBox;
using Ellipsoid = rlss::Ellipsoid<double, DIM>;
using RLSSHardOptimizer = rlss::RLSSHardOptimizer<double, DIM>;
using RLSSSoftOptimizer = rlss::RLSSSoftOptimizer<double, DIM>;
using RLSSHardSoftOptimizer = rlss::RLSSHardSoftOptimizer<double, DIM>;
using RLSSDiscretePathSearcher = rlss::RLSSDiscretePathSearcher<double, DIM>;
using RLSSValidityChecker = rlss::RLSSValidityChecker<double, DIM>;
using RLSSGoalSelector = rlss::RLSSGoalSelector<double, DIM>;
using TrajectoryOptimizer = rlss::TrajectoryOptimizer<double, DIM>;
using DiscretePathSearcher = rlss::DiscretePathSearcher<double, DIM>;
using ValidityChecker = rlss::ValidityChecker<double, DIM>;
using GoalSelector = rlss::GoalSelector<double, DIM>;
using JSONBuilder = rlss::internal::JSONBuilder<double, DIM>;
//using firstsegmentfixer = rlss::internal::firstSegmentFix<double, DIM>;


bool allRobotsReachedFinalStates(
        const std::vector<RLSS>& planners,
        const std::vector<PiecewiseCurve>& original_trajectories,
        const std::vector<StdVectorVectorDIM>& states,
        double reach_distance,
        const double max_parameter,
        const double current_time
) {     
    std::vector<double> nett_euclidean_dist(DIM);
    auto v = 0.1;
    for(std::size_t i = 0; i < planners.size(); i++) {
        VectorDIM goal_position
                = original_trajectories[i].eval(
                        original_trajectories[i].maxParameter(), 0);
    
        double dist = (goal_position - states[i][0]).norm();

        if(dist > reach_distance) {
                //std::cout << "robot " << i << " distance: " << dist << " drone 0 current location: " << states[0][0][0] << " drone 0 max parameter: " << max_parameter << std::endl;
                //typeid(current_time).name() 
                return false; // havent reach location yet
        }
        
    }
    return true;
}

int main(int argc, char* argv[]) {

    std::random_device rd;
    std::mt19937 gen{rd()};

    cxxopts::Options options("RLSS", "");
    options.add_options()
            (
                    "c,config",
                    "Config file path",
                    cxxopts::value<std::string>()
                            ->default_value(
                                    absl::StrCat(
                                            "../examples/",
                                            DIM,
                                            "d_config.json"
                                    )
                            )
            )
            (
                    "h, help",
                    "help"
            )
            ; //config.json refers to the 3D/2D json, whereas robot.json refers to the file under robot directory

    auto parsed_options = options.parse(argc, argv);

    if(parsed_options.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    std::fstream json_fs(
            parsed_options["config"].as<std::string>(), std::ios_base::in);
    nlohmann::json config_json = nlohmann::json::parse(json_fs);
    json_fs.close();

    std::string base_path = config_json["base_path"];
    std::string obstacle_directory = config_json["obstacle_directory"];
    std::string robot_parameters_directory
            = config_json["robot_parameters_directory"];
    double replanning_period = config_json["replanning_period"];
    double reach_distance = config_json["reach_distance"];
    std::vector<double> occupancy_grid_step_size
            = config_json["occupancy_grid_step_size"];

    OccupancyGrid::Coordinate step_size;
    for(unsigned int i = 0; i < DIM; i++) {
        step_size(i) = occupancy_grid_step_size[i];
    }
    OccupancyGrid occupancy_grid(step_size);





    // occupany grid developed from obstacle directory 
    for(auto& p: fs::directory_iterator(base_path + obstacle_directory)) {
        std::fstream obstacle_file(p.path().string(), std::ios_base::in);
        std::string type;
        obstacle_file >> type;
        if(type == "cvxhull") {
            StdVectorVectorDIM obstacle_pts;
            VectorDIM vec;
            while (obstacle_file >> vec(0)) {
                for(unsigned int d = 1; d < DIM; d++)
                    obstacle_file >> vec(d);
                obstacle_pts.push_back(vec);
            }
            occupancy_grid.addObstacle(obstacle_pts);
        } else if(type == "ellipsoid") {
            VectorDIM center;
            MatrixDIMDIM mtr;

            for(unsigned int i = 0; i < DIM; i++) {
                obstacle_file >> center(i);
            }

            for(unsigned int r = 0; r < DIM; r++) {
                for(unsigned int c = 0; c < DIM; c++) {
                    obstacle_file >> mtr(r, c);
                }
            }
            Ellipsoid ell(center, mtr);
            occupancy_grid.addObstacle(ell);

        }
    }

    std::cout << "num obstacles: " << occupancy_grid.size() << std::endl;
    //std::cout << 



    // initialises the relevant vectors for the planner
    std::vector<RLSS> planners;
    std::vector<std::shared_ptr<CollisionShape>> collision_shapes;
    std::vector<PiecewiseCurve> original_trajectories;
    std::vector<unsigned int> contUpto;

    // for each robot there is a vector of pairs where each pair
    // is (degree, distribution)
    std::vector<
            std::vector<
                    std::pair<unsigned int, std::normal_distribution<>
                    >
            >
    > noise;



            

    // robot params and path constraints begin here 
    for(auto& p:
            fs::directory_iterator(base_path + robot_parameters_directory)
            ) {
        std::cout << p.path().string() << std::endl;
        std::fstream robot_description(p.path().string(), std::ios_base::in);
        nlohmann::json robot_json = nlohmann::json::parse(robot_description);

        PiecewiseCurve original_trajectory;
        for(nlohmann::json piece_json
                : robot_json["original_trajectory"]["pieces"]) {
            if(piece_json["type"] == "BEZIER") {
                Bezier piece(piece_json["duration"]);
                for(const std::vector<double>& cpt
                        : piece_json["control_points"]) {
                    assert(cpt.size() == DIM);
                    VectorDIM cpt_vec;
                    for(unsigned int d = 0; d < DIM; d++)
                        cpt_vec(d) = cpt[d];

                    piece.appendControlPoint(cpt_vec); // 1 pc, 2 ctrl points
                }
                original_trajectory.addPiece(piece);// rmb this damn shit 
            }
        }
        original_trajectories.push_back(original_trajectory);

        PiecewiseCurveQPGenerator qp_generator;
        nlohmann::json pieces_json;
        if(config_json.contains("plan_for_trajectory")) {
            // override robot json
            pieces_json = config_json["plan_for_trajectory"]["pieces"];
        } else {
            pieces_json = robot_json["plan_for_trajectory"]["pieces"];
        } // 8 8 8 8


        for(const nlohmann::json& plan_for_piece: pieces_json) {
            if(plan_for_piece["type"] == "BEZIER") {
                qp_generator.addBezier(plan_for_piece["num_control_points"], 0); // [8,0]
            } else {
                throw std::domain_error (
                        absl::StrCat(
                                "curve type not handled"
                        )
                );
            }
        }



        double search_step = robot_json["search_step"];//
        double rescaling_multiplier = robot_json["rescaling_multiplier"];//
        unsigned int max_rescaling_count
                = config_json.contains("max_rescaling_count")//
                  ? config_json["max_rescaling_count"]
                  : robot_json["max_rescaling_count"];
        double desired_time_horizon = robot_json["desired_time_horizon"];//
        unsigned int continuity_upto_degree
                = config_json.contains("continuity_upto_degree")
                  ? config_json["continuity_upto_degree"]
                  : robot_json["continuity_upto_degree"];
        contUpto.push_back(continuity_upto_degree);
        std::vector<std::pair<unsigned int, double>>
                maximum_derivative_magnitudes;
        for(const std::pair<unsigned int, double>& der_mag
                : (config_json.contains("maximum_derivative_magnitudes") ?
                   config_json["maximum_derivative_magnitudes"]
                 : robot_json["maximum_derivative_magnitudes"])
                ) {
            maximum_derivative_magnitudes.push_back(der_mag);
        }
        std::vector<std::pair<unsigned int, double>>
                integrated_squared_derivative_weights;
        for(const std::pair<unsigned int, double>& deg_weight
                : robot_json["integrated_squared_derivative_weights"]) {
            integrated_squared_derivative_weights.push_back(deg_weight);
        }
        std::vector<double> piece_endpoint_cost_weights;
        for(const double weight: robot_json["piece_endpoint_cost_weights"]) {
            piece_endpoint_cost_weights.push_back(weight);
        }

        nlohmann::json colshapeatzero;
        if(config_json.contains("collision_shape_at_zero")) {
            colshapeatzero = config_json["collision_shape_at_zero"]; // collision shape taken from config_json
        } else {
            colshapeatzero = robot_json["collision_shape_at_zero"];
        }

        VectorDIM colshapemin, colshapemax;
        for(unsigned int d = 0; d < DIM; d++) {
            colshapemin(d) = colshapeatzero[0][d];
            colshapemax(d) = colshapeatzero[1][d];
        }

        auto albox_col_shape = std::make_shared<AlignedBoxCollisionShape>(
                AlignedBox(colshapemin, colshapemax));
        auto collision_shape = std::static_pointer_cast<CollisionShape>(
                albox_col_shape);

        collision_shapes.push_back(collision_shape);

        nlohmann::json workspace_json;
        if(config_json.contains("workspace")) {
            workspace_json = config_json["workspace"];
        } else {
            workspace_json = robot_json["workspace"];
        }

        VectorDIM workspacemin, workspacemax;

        for(unsigned int d = 0; d < DIM; d++) {
            workspacemin(d) = workspace_json[0][d];
            workspacemax(d) = workspace_json[1][d];
        }

        AlignedBox workspace(workspacemin, workspacemax);

        std::string optimizer = config_json.contains("optimizer")
                                ? config_json["optimizer"] : robot_json["optimizer"];

        // parameter -> [enabled, weight]
        std::unordered_map<std::string, std::pair<bool, double>>
                soft_optimization_parameters;
        nlohmann::json sop_json
                = config_json.contains("soft_optimization_parameters") ?
                  config_json["soft_optimization_parameters"] :
                  (robot_json.contains("soft_optimization_parameters") ?
                   robot_json["soft_optimization_parameters"]
                   : nlohmann::json());


        for(auto it = sop_json.begin(); it != sop_json.end(); it++) {
            soft_optimization_parameters[it.key()]
                    = std::make_pair(it.value()["enable"], it.value()["weight"]);
        }

        double optimization_obstacle_check_distance
                = config_json.contains("optimization_obstacle_check_distance") ? // taken from config_json
                  config_json["optimization_obstacle_check_distance"] :
                  robot_json["optimization_obstacle_check_distance"];

        nlohmann::json noise_json =
                config_json.contains("noise") ?
                config_json["noise"] :
                (robot_json.contains("noise") ?
                 robot_json["noise"] :
                 nlohmann::json()
                );

        noise.push_back(
                std::vector<
                        std::pair<unsigned int, std::normal_distribution<>>
                >()
        );
        for(const auto& n : noise_json) {
            noise.back().emplace_back(
                    n[0],
                    std::normal_distribution<>(n[1], n[2])
            );
        }

        auto rlss_goal_selector = std::make_shared<RLSSGoalSelector>
                (
                        desired_time_horizon,
                        original_trajectory,
                        workspace,
                        collision_shape,
                        search_step
                );
        auto goal_selector
                = std::static_pointer_cast<GoalSelector>(rlss_goal_selector);

        double maximum_velocity = std::numeric_limits<double>::max();
        for(const auto& [d, v]: maximum_derivative_magnitudes) {
            if(d == 1) {
                maximum_velocity = v;
                break;
            }
        }
        auto rlss_discrete_path_searcher
                = std::make_shared<RLSSDiscretePathSearcher>
                        (
                                replanning_period, 
                                workspace,
                                collision_shape,
                                maximum_velocity, // should max_vel be minimally greater than the required vel (eucl distance/duration)?
                                qp_generator.numPieces()
                        );
        auto discrete_path_searcher
                = std::static_pointer_cast<DiscretePathSearcher>(
                        rlss_discrete_path_searcher);

        std::shared_ptr<TrajectoryOptimizer> trajectory_optimizer;
        if(optimizer == "rlss-hard-soft") {
            auto rlss_hard_soft_optimizer = std::make_shared<RLSSHardSoftOptimizer>
                    (
                            collision_shape,
                            qp_generator,
                            workspace,
                            continuity_upto_degree,
                            integrated_squared_derivative_weights,
                            piece_endpoint_cost_weights,
                            soft_optimization_parameters,
                            optimization_obstacle_check_distance
                    );
            trajectory_optimizer
                    = std::static_pointer_cast<TrajectoryOptimizer>(
                    rlss_hard_soft_optimizer);
        } else if (optimizer == "rlss-soft") {
            auto rlss_soft_optimizer = std::make_shared<RLSSSoftOptimizer>
                    (
                            collision_shape,
                            qp_generator,
                            workspace,
                            continuity_upto_degree,
                            integrated_squared_derivative_weights,
                            piece_endpoint_cost_weights,
                            soft_optimization_parameters,
                            optimization_obstacle_check_distance
                    );
            trajectory_optimizer
                    = std::static_pointer_cast<TrajectoryOptimizer>(
                    rlss_soft_optimizer);
        } else if (optimizer == "rlss-hard") {
            auto rlss_hard_optimizer = std::make_shared<RLSSHardOptimizer>
                    (
                            collision_shape,
                            qp_generator,
                            workspace,
                            continuity_upto_degree,
                            integrated_squared_derivative_weights,
                            piece_endpoint_cost_weights,
                            optimization_obstacle_check_distance
                    );
            trajectory_optimizer
                    = std::static_pointer_cast<TrajectoryOptimizer>(
                    rlss_hard_optimizer);
        } else {
            throw std::domain_error(
                    absl::StrCat(
                            "optimizer ",
                            optimizer,
                            " not recognized."
                    )
            );
        }


        auto rlss_validity_checker = std::make_shared<RLSSValidityChecker>
                (
                        maximum_derivative_magnitudes,
                        search_step
                );
        auto validity_checker
                = std::static_pointer_cast<ValidityChecker>(
                        rlss_validity_checker);

        planners.emplace_back(
                goal_selector,
                trajectory_optimizer,
                discrete_path_searcher,
                validity_checker,
                max_rescaling_count,
                rescaling_multiplier
        );

    }

    unsigned int num_robots = planners.size();

    std::cout << "num robots: " << num_robots << std::endl;

    std::vector<StdVectorVectorDIM> states(num_robots);
    for(unsigned int i = 0; i < num_robots; i++) {
        for(unsigned int j = 0; j <= contUpto[i]; j++) {
            states[i].push_back(original_trajectories[i].eval(0, j)); // states initial condition == original_trajectory.eval(0,0) for position at time 0 at derivative 0
        }
    }


    JSONBuilder json_builder;
    json_builder.setRobotCount(num_robots);
    for(std::size_t i = 0; i < num_robots; i++) {
        json_builder.setRobotShape(i, collision_shapes[i]->boundingBox(VectorDIM::Zero()));
    }

    for(unsigned int r_id = 0; r_id < num_robots; r_id++) {
        json_builder.setOriginalTrajectory(
                r_id,
                original_trajectories[r_id]
        );
    }
    json_builder.setFrameDt(0.01); // json description begin here
    json_builder.addOccupancyGridToCurrentFrame(occupancy_grid);

    double current_time = 0.0;
    std::vector<PiecewiseCurve> trajectories(num_robots);
    std::vector<double> trajectory_current_times(num_robots, 0);
    double max_parameter;
    StdVectorVectorDIM old_position_state(DIM); //vector containing vectors of DIM and Type
    double total_displacement_0 = 0.0;
    double total_displacement_1 = 0.0;

    while(!allRobotsReachedFinalStates( // bool function at the top, while all robots have not reached their final state
            planners, original_trajectories, states, reach_distance, max_parameter, current_time)) //ADD IN MAX PARAM
             
        {
        rlss::debug_message("starting the loop for time ", current_time, "...");

        rlss::debug_message("collecting robot shapes...");

        
        std::vector<AlignedBox> robot_collision_boxes(num_robots); // specifiy that within robot_collision_boxes, theres 2 since 2 robots for this example
        for(std::size_t i = 0; i < num_robots; i++) {
            robot_collision_boxes[i]
                    = collision_shapes[i]->boundingBox(states[i][0]);// this is where the current states are extracted for the bounding boxes for all the drones 
            //std::cout << "aligned_box: " << robot_collision_boxes[i].BottomLeft << std::endl;
            //std::cout << "Drone " << i << " last known position: " << states[i][0] << std::endl;
            old_position_state[i] = states[i][0];
        
        }


        for(std::size_t i = 0; i < planners.size(); i++) {
            rlss::debug_message("planning for robot ", i, "...");
            std::vector<AlignedBox> other_robot_collision_boxes
                    = robot_collision_boxes;
            other_robot_collision_boxes.erase(
                    other_robot_collision_boxes.begin() + i);

//            double cutoff_duration = 100000; // us
//            auto plan_start_time = std::chrono::steady_clock::now();
            std::optional<PiecewiseCurve> curve
                    = planners[i].plan(
                            current_time, // current time would reflect where the drone needs to go to in the current traj planned, each time stamp is acquainted (current time + horizon)
                            states[i], // drone's current position
                            other_robot_collision_boxes, ///other robot's position covered w collision boxes
                            occupancy_grid
                    );
//            auto plan_end_time = std::chrono::steady_clock::now();
//            auto planning_duration = std::chrono::duration_cast<std::chrono::microseconds>(
//                        plan_end_time - plan_start_time
//                ).count();

            if(curve /*&& planning_duration < cutoff_duration*/) {    //curve begins at 0(current position) and ends at max parameter aka time horizon or lesser if below duration limit (goal position)
                rlss::debug_message(
                        rlss::internal::debug::colors::GREEN,
                        "planning successful.",
                        rlss::internal::debug::colors::RESET
                );
                trajectories[i] = *curve;
                trajectory_current_times[i] = 0; // traj_current time will always restart at 0, the only things that are constantly being updated would be state, as well as current time
                json_builder.addTrajectoryToCurrentFrame(i, trajectories[i]);
            } else {
                rlss::debug_message(
                        rlss::internal::debug::colors::RED,
                        "planning failed.",
                        rlss::internal::debug::colors::RESET
                );
                trajectory_current_times[i] += replanning_period;
            }

            for(std::size_t j = 0; j < states[i].size(); j++) { // read this ***************** states[i].size() = 2 (continuity up to degree param)
                if(j == 0) {
                    json_builder.setRobotPositionInCurrentFrame(
                            i, states[i][j]);
                }
//                std::cout << "robot " << i << " eval" << std::endl;
                states[i][j] = trajectories[i].eval( // this is the position it needs to actually go to
                        std::min(
                                trajectory_current_times[i] + replanning_period,   // updated position after replanning period, for planning of next curve only thats why u dun see the robot json updater
                                trajectories[i].maxParameter()
                        ),
                        j
                );

                for(auto& n: noise[i]) {
                    if(n.first == j) {
                        VectorDIM noise_addition;
                        for(unsigned int d = 0; d < DIM; d++) {
                            noise_addition(d) = n.second(gen);
                        }
                        states[i][j] += noise_addition;
                    }
                }
            }
             
            
            //old_state = rlss::internal::firstSegmentFix<double, DIM>(old_state); 
            std::cout << "Drone " << i << " nett euclid distance " << (original_trajectories[i].eval(original_trajectories[i].maxParameter(), 0)).norm() << std::endl;            
            std::cout << "Drone " << i << " required velocity " << (original_trajectories[i].eval(original_trajectories[i].maxParameter(), 0)).norm()/original_trajectories[i].maxParameter() << std::endl;
            std::cout << "Drone " << i << " velocity: " << (states[i][0] - old_position_state[i]).norm()/replanning_period << std::endl;
            std::cout << "Drone " << i << " current position: " << old_position_state[i] << std::endl; //current position
            std::cout << "Drone " << i << " goal position: " << trajectories[i].eval(trajectories[i].maxParameter(), 0) << std::endl; //goal
            std::cout << "Drone " << i << " updated position after 0.1s replanning: " << states[i][0] << std::endl; // after 0.1s movement
            std::cout << "\n" << std::endl;       
        }



        json_builder.nextFrame();
        for(double t = 0.01; t < replanning_period - 0.005; t += 0.01) { //for 0.05, the number of steps taken was half (5) as compared to 10 for 0.1
            for(std::size_t i = 0; i < num_robots; i++) {
                json_builder.setRobotPositionInCurrentFrame(
                        i,
                        trajectories[i].eval(
                                std::min(
                                        trajectory_current_times[i] + t, // 10 positions leading up to the updated position after every replanning period
                                        trajectories[i].maxParameter()
                                ),
                                0
                        )                  
                );
               // std::cout << "trajectories max param: " << trajectories[0].maxParameter() << std::endl;
            }
            json_builder.nextFrame(); // this function includes the steps 
        }

        json_builder.save("vis.json");
        rlss::internal::StatisticsStorage<double> all_stats;
        for(const auto& planner: planners) {
//        planner.statisticsStorage().save();
            all_stats += planner.statisticsStorage();
        }

        all_stats.save("all_stats.json");
        current_time += replanning_period;  //current time at the end equivalent to the duration
        //std::cout << trajectories[0].maxParameter() << std::endl;
        max_parameter = trajectories[0].maxParameter();
        std::cout << "Current_time: " << current_time << " Drone 0 Final Position: " << states[0][0] << " Drone 1 Final Position: "<< states [1][0] << std::endl;
        //<< " traj_current_time: " << trajectory_current_times[0] << std::endl;
        //std::cout << "max_param: " << max_parameter << " current_position: " << states[0][0] << std::endl;
        //std::cout << qp_generator.numPieces() << endl;
        //std::cout << "original_traj[0]: " << original_trajectories[0].maxParameter() << std::endl; // 15 aka the duration
        total_displacement_0 += ((states[0][0] - old_position_state[0]).norm());
        total_displacement_1 += ((states[1][0] - old_position_state[1]).norm());
        //std::cout << "Drone 0 Final Pt: " << original_trajectories[0].eval(original_trajectories[0].maxParameter(), 0) << std::endl;
        //std::cout << states[0][0] << std::endl;
        //std::cout << old_position_state[0] << std::endl;
        std::cout << "Total displacement of Drone 0: " << total_displacement_0 << std::endl;
        std::cout << "Actual Required Velocity of Drone 0: " << total_displacement_0/original_trajectories[0].maxParameter() << std::endl;
        std::cout << "Total displacement of Drone 1: " << total_displacement_1 << std::endl;
        std::cout << "Actual Required Velocity of Drone 1: " << total_displacement_1/original_trajectories[1].maxParameter() << std::endl;
        std::cout << "num obstacles: " << occupancy_grid.size() << std::endl;
        std::cout << "\n";
    }


    rlss::internal::StatisticsStorage<double> all_stats;
    for(const auto& planner: planners) {
//        planner.statisticsStorage().save();
        all_stats += planner.statisticsStorage();
    }

    all_stats.save("all_stats.json");

    json_builder.save("vis.json");

    return 0;
}