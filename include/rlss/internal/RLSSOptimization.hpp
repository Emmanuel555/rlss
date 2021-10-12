#ifndef RLSS_INTERNAL_RLSS_OPTIMIZATION_HPP
#define RLSS_INTERNAL_RLSS_OPTIMIZATION_HPP

#include <rlss/CollisionShapes/CollisionShape.hpp>
#include <splx/opt/PiecewiseCurveQPGenerator.hpp>
#include <rlss/internal/Util.hpp>
#include <rlss/internal/SVM.hpp>
#include <rlss/internal/MathematicaWriter.hpp>

namespace rlss {
namespace internal {

template<typename T, unsigned int DIM>
std::vector<Hyperplane<T, DIM>> robot_safety_hyperplanes(
        const VectorDIM<T, DIM>& robot_position,
        const std::vector<AlignedBox<T, DIM>>&
        other_robot_collision_shape_bounding_boxes,
        std::shared_ptr<CollisionShape<T, DIM>> colshape) {

    using Hyperplane = internal::Hyperplane<T, DIM>;
    using AlignedBox = internal::AlignedBox<T, DIM>;
    using StdVectorVectorDIM = internal::StdVectorVectorDIM<T, DIM>;
     
    //std::cout << "hyper_plane check_1..." << std::endl;

    std::vector<Hyperplane> hyperplanes;

    AlignedBox robot_box
            = colshape->boundingBox(robot_position);

    StdVectorVectorDIM robot_points
            = rlss::internal::cornerPoints<T, DIM>(robot_box);

    //std::cout << "hyper_plane check_2..." << std::endl;

    for(const auto& oth_collision_shape_bbox:
            other_robot_collision_shape_bounding_boxes) {

        std::cout << "hyper_plane for loop init..." << std::endl;
        std::cout << "collision boxes center..." << (oth_collision_shape_bbox.center()) << std::endl;
        std::cout << "collision boxes min..." << (oth_collision_shape_bbox.min()) << std::endl;
        std::cout << "collision boxes max..." << (oth_collision_shape_bbox.max()) << std::endl;

        StdVectorVectorDIM oth_points
                = rlss::internal::cornerPoints<T, DIM>(oth_collision_shape_bbox);


        Hyperplane svm_hp = rlss::internal::svm<T, DIM>(robot_points, oth_points); // (current robot box and other robot's box)


        Hyperplane svm_shifted = rlss::internal::shiftHyperplane<T, DIM>(
                robot_position,
                robot_box,
                svm_hp
        );
        hyperplanes.push_back(svm_shifted);
    }

    //std::cout << "hyper_plane check..." << std::endl;

    return hyperplanes;
}

template<typename T, unsigned int DIM>
void generate_optimization_problem(
    splx::PiecewiseCurveQPGenerator<T, DIM>& qpgen,
    std::shared_ptr<CollisionShape<T, DIM>> colshape,
    const AlignedBox<T, DIM>& wss,
    unsigned int contupto,
    const std::vector<std::pair<unsigned int, T>>& lambdas,
    const std::vector<T>& thetas,
    T obstacle_check_distance,
    const StdVectorVectorDIM<T, DIM>& segments,
    const std::vector<T>& durations,
    const std::vector<AlignedBox<T, DIM>>& oth_rbt_col_shape_bboxes,
    const OccupancyGrid<T, DIM>& occupancy_grid,
    const StdVectorVectorDIM<T, DIM>& current_robot_state,
    MathematicaWriter<T, DIM>& mathematica,
    const std::unordered_map<std::string, std::pair<bool, T>>& soft_parameters
            = std::unordered_map<std::string, std::pair<bool, T>>()
) {
    using VectorDIM = internal::VectorDIM<T, DIM>;
    using AlignedBox = internal::AlignedBox<T, DIM>;
    using Hyperplane = internal::Hyperplane<T, DIM>;
    using StdVectorVectorDIM = internal::StdVectorVectorDIM<T, DIM>;

    if(segments.size() != qpgen.numPieces() + 1) {
        throw std::domain_error(
            absl::StrCat(
                "number of segments is not equal to number of pieces",
                ", number of segments: ",
                segments.size() - 1,
                ", number of pieces: ",
                qpgen.numPieces()
            )
        );
    }

    if(durations.size() + 1 != segments.size()) {
        throw std::domain_error(
            absl::StrCat(
                "number of piece durations is not equal to the number of ",
                "segments, number of durations: ",
                durations.size(),
                ", number of segments: ",
                segments.size()
            )
        );
    }

    //std::cout << "why isnt this triggered? " << current_robot_state.size() << std::endl;
    //std::cout << "why isnt this triggered? " << contupto << std::endl;

    if(current_robot_state.size() <= contupto) {
        std::cout << "robot_state_size_triggered " << std::endl;
        throw std::domain_error(
            absl::StrCat(
                "robot state size does not contain some required derivatives",
                ", max derivative degree contained in the state: ",
                current_robot_state.size() - 1,
                ", continuity upto: ",
                contupto
            )
        );
    }

    //std::cout << "checkpoint_1..." << std::endl;

    if(thetas.size() != qpgen.numPieces()) {
        std::cout << "no.of pieces_triggered " << std::endl;
        throw std::domain_error(
            absl::StrCat(
               "number of piece endpoint parameters is not equal to ",
               "number of pieces.",
               " number of pieces: ",
               qpgen.numPieces(),
               ", number of parameters: ",
               thetas.size()
            )
        );
    }

    //std::cout << "checkpoint_2..." << oth_rbt_col_shape_bboxes.size() << std::endl;

    for(const auto& bbox: oth_rbt_col_shape_bboxes) {
        debug_message(
        "other robot collision shape bounding box is [min: ",
        bbox.min().transpose(),
        ", max: ",
        bbox.max().transpose(),
        "]"
        );
        mathematica.otherRobotCollisionBox(bbox);
    }

    //std::cout << "checkpoint_3..." << std::endl;

    qpgen.resetProblem();
    qpgen.setPieceMaxParameters(durations);

    std::cout << "checkpoint_4..." << std::endl;

    mathematica.discretePath(segments);

    std::cout << "checkpoint_5..." << std::endl;

    // workspace constraint
    AlignedBox ws = rlss::internal::bufferAlignedBox<T, DIM>(
            VectorDIM::Zero(),
            colshape->boundingBox(VectorDIM::Zero()),
            wss
    );

    //std::cout << "checkpoint_6..." << std::endl;
 
    qpgen.addBoundingBoxConstraint(ws);

    //std::cout << "checkpoint_7..." << std::endl;

    debug_message(
        "buffered workspace is [min: ",
        ws.min().transpose(),
        ", max: ",
        ws.max().transpose(),
        "]"
    );

    //std::cout << "checkpoint_8..." << std::endl;

    mathematica.selfCollisionBox(
            colshape->boundingBox(current_robot_state[0]));

    // robot to robot avoidance constraints for the first piece - 1
    
    if (soft_parameters.at("robot_to_robot_hyperplane_constraints").first == 1)
    {
    std::vector<Hyperplane> robot_to_robot_hps
            = robot_safety_hyperplanes<T, DIM>(
                    current_robot_state[0],
                    oth_rbt_col_shape_bboxes,
                    colshape
            );

    std::cout << "checkpoint_10..." << std::endl;        
    std::cout << "Hyperplane vector size..." << std::endl;
//    robot_to_robot_hps = internal::pruneHyperplanes<T, DIM>(
//            robot_to_robot_hps, ws);

    for(const auto& hp: robot_to_robot_hps) { //vector containing hyperplanes
        bool r2r_hyperplane_constraints_soft_enabled 
            = soft_parameters.find("robot_to_robot_hyperplane_constraints")
                    != soft_parameters.end()
              && soft_parameters.at("robot_to_robot_hyperplane_constraints").first; // enable : true
        T r2r_hyperplane_constraints_soft_weight = 
                r2r_hyperplane_constraints_soft_enabled
                ? soft_parameters.at("robot_to_robot_hyperplane_constraints").second // weight : 10000
                : 0;
        qpgen.addHyperplaneConstraintForPiece( // 1
                0,
                hp,
                r2r_hyperplane_constraints_soft_enabled,
                r2r_hyperplane_constraints_soft_weight
        );
        if(hp.signedDistance(current_robot_state[0]) > 0) {
            debug_message(
                "distance of current point to a first piece hyperplane: ",
                hp.signedDistance(current_robot_state[0])
            );
        }
    }


    assert(robot_to_robot_hps.size() == oth_rbt_col_shape_bboxes.size());

    for(const auto& hp: robot_to_robot_hps) {
        mathematica.robotCollisionAvoidanceHyperplane(hp);
        debug_message(
            "robot to robot collision avoidance hyperplane [n: ",
            hp.normal().transpose(),
            ", d: ",
            hp.offset(),
            "]"
        );
    }

    } // reject this hyperplane contraint for r2r for now


    std::cout << "r2r condition cleared..." << std::endl;

    std::cout << "find r2o weights..." << soft_parameters.at("robot_to_obstacle_hyperplane_constraints").second << std::endl;

    //std::cout << obstacle_check_distance << std::endl;

    
    if (soft_parameters.at("robot_to_obstacle_hyperplane_constraints").first == 1)
    {
    // robot to obstacle avoidance constraints for all pieces -2
    for (
        std::size_t p_idx = 0;
        p_idx < qpgen.numPieces();
        p_idx++
    ) {
        AlignedBox from_box
                = colshape->boundingBox(segments[p_idx]);
        AlignedBox to_box
                = colshape->boundingBox(segments[p_idx+1]);
        to_box.extend(from_box);

        StdVectorVectorDIM segments_corners
                = rlss::internal::cornerPoints<T, DIM>(to_box);

        std::vector<Hyperplane> piece_obstacle_hyperplanes;

        for(
            auto it = occupancy_grid.begin(to_box, obstacle_check_distance);
            it != occupancy_grid.end(to_box, obstacle_check_distance);
            ++it
        ) {

            AlignedBox grid_box = *it;

            StdVectorVectorDIM grid_box_corners
                    = rlss::internal::cornerPoints<T, DIM>(grid_box);

            Hyperplane shp = rlss::internal::svm<T, DIM>
                    (
                            segments_corners,
                            grid_box_corners
                    );


            shp = rlss::internal::shiftHyperplane<T, DIM>(
                    VectorDIM::Zero(),
                    colshape->boundingBox(VectorDIM::Zero()),
                    shp
            );


            mathematica.obstacleCollisionBox(grid_box);

            piece_obstacle_hyperplanes.push_back(shp);
        }

        debug_message("before prune num hyperplanes: "
                , piece_obstacle_hyperplanes.size());
//        piece_obstacle_hyperplanes
//            = internal::pruneHyperplanes<T, DIM>(
//                                piece_obstacle_hyperplanes,
//                                ws
//         );
        debug_message("after prune num hyperplanes: "
                , piece_obstacle_hyperplanes.size());

        for(const auto& shp: piece_obstacle_hyperplanes) {
            bool r2o_hyperplane_constraints_soft_enabled
                = soft_parameters.find(
                        "robot_to_obstacle_hyperplane_constraints"
                  )
                  != soft_parameters.end()
                  && soft_parameters.at("robot_to_obstacle_hyperplane_constraints")
                                        .first;

            T r2o_hyperplane_constraints_soft_weight =
                r2o_hyperplane_constraints_soft_enabled
                ? soft_parameters.at("robot_to_obstacle_hyperplane_constraints")
                            .second
                : 0;
            qpgen.addHyperplaneConstraintForPiece( // 2
                    p_idx,
                    shp,
                    r2o_hyperplane_constraints_soft_enabled,
                    r2o_hyperplane_constraints_soft_weight
            );
        }
    }
    }


    // continuity constraints - 3
    for(
        std::size_t p_idx = 0;
        p_idx < qpgen.numPieces() - 1;
        p_idx++
    ) {
        for(unsigned int k = 0; k <= contupto; k++) {
            debug_message(
                "adding continuity constraint between piece ",
                p_idx,
                " and ",
                p_idx+1,
                " for degree ",
                k
            );
            bool continuity_constraints_soft_enabled
                = soft_parameters.find(
                        "continuity_constraints"
                )
                  != soft_parameters.end()
                  && soft_parameters.at("continuity_constraints")
                          .first;

            T continuity_constraints_soft_weight =
                continuity_constraints_soft_enabled
                ? soft_parameters.at("continuity_constraints")
                        .second
                : 0;
            /*qpgen.addContinuityConstraint( // 3
                p_idx,
                k,
                continuity_constraints_soft_enabled,
                continuity_constraints_soft_weight
            );*/
        }
    }

    // initial point constraints - 4
    for(unsigned int k = 0; k <= contupto; k++) {
        debug_message(
            "adding initial point constraint for degree ",
            k,
            ". It should be ",
            current_robot_state[k].transpose()
        );
        bool initial_point_constraints_soft_enabled
                = soft_parameters.find(
                        "initial_point_constraints"
                )
                  != soft_parameters.end()
                  && soft_parameters.at("initial_point_constraints")
                          .first;

        T initial_point_constraints_soft_weight =
                initial_point_constraints_soft_enabled
                ? soft_parameters.at("initial_point_constraints")
                        .second
                : 0;
        /*qpgen.addEvalConstraint( // 4
                0,
                k,
                current_robot_state[k],
                initial_point_constraints_soft_enabled,
                initial_point_constraints_soft_weight
        );*/
    }



    // energy cost
    for(const auto& [d, l]: lambdas) {
        debug_message("adding integrated squared derivative cost for",
            "degree ", d, " with lambda ", l
        );
        qpgen.addIntegratedSquaredDerivativeCost(d, l);
    }



    // eval cost taking into acc the above constraints
    T duration_sum_before = 0;
    for(
        std::size_t p_idx = 0;
        p_idx < qpgen.numPieces();
        duration_sum_before += durations[p_idx], p_idx++
    ) {
        qpgen.addEvalCost(
                std::min(
                    duration_sum_before + durations[p_idx],
                    qpgen.maxParameter()
                ),
                0,
                segments[p_idx+1],
                thetas[p_idx]
        );
    }


}

} // namespace internal
} // namespace rlss

#endif // RLSS_INTERNAL_RLSS_OPTIMIZATION_HPP