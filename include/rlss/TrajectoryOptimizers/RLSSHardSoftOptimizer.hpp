#ifndef RLSS_RLSS_HARD_SOFT_OPTIMIZER_HPP
#define RLSS_RLSS_HARD_SOFT_OPTIMIZER_HPP

#include <rlss/internal/Util.hpp>
#include <rlss/TrajectoryOptimizers/TrajectoryOptimizer.hpp>
#include <splx/opt/PiecewiseCurveQPGenerator.hpp>
#include <rlss/CollisionShapes/CollisionShape.hpp>
#include <rlss/internal/SVM.hpp>
#include <rlss/internal/MathematicaWriter.hpp>
#include <chrono>

namespace rlss {

    template<typename T, unsigned int DIM>
    class RLSSHardSoftOptimizer: public TrajectoryOptimizer<T, DIM> {
    public:
        using Base = TrajectoryOptimizer<T, DIM>;
        using StdVectorVectorDIM = typename Base::StdVectorVectorDIM;
        using AlignedBox = typename Base::AlignedBox;
        using OccupancyGrid = typename Base::OccupancyGrid;
        using PiecewiseCurve = typename Base::PiecewiseCurve;
        using PiecewiseCurveQPGenerator = splx::PiecewiseCurveQPGenerator<T, DIM>;
        using VectorDIM = rlss::internal::VectorDIM<T, DIM>;
        using CollisionShape = rlss::CollisionShape<T, DIM>;
        using Hyperplane = rlss::internal::Hyperplane<T, DIM>;
        using Vector = typename PiecewiseCurveQPGenerator::Vector;

        RLSSHardSoftOptimizer(
            std::shared_ptr<CollisionShape> colshape,
            const PiecewiseCurveQPGenerator& qpgen, // PiecewiseCurveQPGenerator qp_generator
            const AlignedBox& ws,
            unsigned int contupto,
            const std::vector<std::pair<unsigned int, T>>& lambdas, // integrated_squared_derivative_weights
            const std::vector<T>& thetas, // piece_endpoint_cost_weights
            const std::unordered_map<std::string, std::pair<bool, T>>&
                    soft_parameters, // soft_optimization_parameters
            T obstacle_check_distance
        ): m_collision_shape(colshape),
           m_qp_generator(qpgen),
           m_workspace(ws),
           m_continuity_upto(contupto),
           m_lambda_integrated_squared_derivatives(lambdas),
           m_theta_position_at(thetas),
           m_soft_parameters(soft_parameters),
           m_obstacle_check_distance(obstacle_check_distance)
        {

        }

        // returns std::nullopt when optimization fails
        std::optional<PiecewiseCurve> optimize(
                const StdVectorVectorDIM& segments,
                const std::vector<T>& durations,
                const std::vector<AlignedBox>& oth_rbt_col_shape_bboxes,
                const OccupancyGrid& occupancy_grid,
                const StdVectorVectorDIM& current_robot_state
        )  override {

            /*std::cout << "Params..." << std::endl;
            std::cout << "r2r true?..." << m_soft_parameters["robot_to_robot_hyperplane_constraints"].first << std::endl;
            std::cout << m_obstacle_check_distance << std::endl;
            std::cout << segments.size() << std::endl;
            std::cout << occupancy_grid.size() << std::endl;
            std::cout << current_robot_state[0][0] << std::endl;
            std::cout << oth_rbt_col_shape_bboxes.size() << std::endl;*/

            internal::MathematicaWriter<T, DIM> mathematica;

            try {
                internal::generate_optimization_problem<T, DIM>(
                        m_qp_generator, // append constraints to qpgen
                        m_collision_shape,
                        m_workspace,
                        m_continuity_upto,
                        m_lambda_integrated_squared_derivatives,
                        m_theta_position_at,
                        m_obstacle_check_distance,
                        segments,
                        durations,
                        oth_rbt_col_shape_bboxes,
                        occupancy_grid,
                        current_robot_state,
                        mathematica,
                        m_soft_parameters
                );
            } catch(...) {
                std::cout << "Caught here..." << std::endl;
                return std::nullopt;
            }

         
            QPWrappers::RLSS_HARD_QP_SOLVER::Engine<T> solver; // defined solver
            solver.setFeasibilityTolerance(1e-9); // solver tolerance
            auto initial_guess = m_qp_generator.getDVarsForSegments(segments); //initial guess wud be from a star
            Vector soln; // initialise soln type
            QPWrappers::OptReturnType ret = QPWrappers::OptReturnType::Unknown;

            try {
                ret = solver.next(
                        m_qp_generator.getProblem(), soln, initial_guess); // the segments r the initial guesses rmb!!!!!!
            } catch (...) {
            std::cout << "initial solution failed..." << std::endl;
            }
            debug_message("hard optimization return value: ", ret);

            std::cout << ret << std::endl;
            
            if(ret == QPWrappers::OptReturnType::Optimal) { // hard optimisation soln worked
                auto result = m_qp_generator.extractCurve(soln);
                mathematica.piecewiseCurve(result);
                std::cout << "optimal for first try..." << std::endl;
                return result;
            } else {
                std::cout << "non-optimal..." << std::endl;
                auto soft_problem = m_qp_generator.getProblem().convert_to_soft();
                Vector soft_initial_guess(soft_problem.num_vars());
                soft_initial_guess.setZero();
                soft_initial_guess.block(0, 0, initial_guess.rows(), 1) = initial_guess;
                Vector soft_solution;
                QPWrappers::RLSS_SOFT_QP_SOLVER::Engine<T> solver;
                solver.setFeasibilityTolerance(1e-9);
                try {
                    ret = solver.next(soft_problem, soft_solution,
                                      soft_initial_guess);
                } catch(...) {
                }
                debug_message("soft optimization return value: ", ret);
                if(ret == QPWrappers::OptReturnType::Optimal) {
                    Vector soft_solution_primary
                        = soft_solution.block(0, 0, initial_guess.rows(), 1);
                    auto result = m_qp_generator.extractCurve(soft_solution_primary);
                    mathematica.piecewiseCurve(result);
                    std::cout << "optimal second try with soft constraints..." << std::endl;
                    return result;
                } else {
                    std::cout << "caught at the end..." << std::endl;
                    return std::nullopt;
                }
            }
        }
    private:
        std::shared_ptr<CollisionShape> m_collision_shape;
        PiecewiseCurveQPGenerator m_qp_generator;
        AlignedBox m_workspace;
        unsigned int m_continuity_upto;
        std::vector<std::pair<unsigned int, T>>
                m_lambda_integrated_squared_derivatives;
        std::vector<T> m_theta_position_at;
        std::unordered_map<std::string, std::pair<bool, T>> m_soft_parameters;
        T m_obstacle_check_distance;
    }; // class TrajectoryOptimizer

}

#endif // RLSS_RLSS_HARD_SOFT_OPTIMIZER_HPP