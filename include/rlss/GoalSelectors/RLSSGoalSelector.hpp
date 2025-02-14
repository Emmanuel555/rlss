#ifndef RLSS_RLSS_GOAL_SELECTOR_HPP
#define RLSS_RLSS_GOAL_SELECTOR_HPP
#include <rlss/GoalSelectors/GoalSelector.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <rlss/CollisionShapes/CollisionShape.hpp>


namespace rlss {

template<typename T, unsigned int DIM>
class RLSSGoalSelector: public GoalSelector<T, DIM> {
public:
    using Base = GoalSelector<T, DIM>;
    using VectorDIM = typename Base::VectorDIM;
    using OccupancyGrid = typename Base::OccupancyGrid;
    using PiecewiseCurve = splx::PiecewiseCurve<T, DIM>;
    using Index = typename OccupancyGrid::Index;
    using AlignedBox = rlss::internal::AlignedBox<T, DIM>;
    using CollisionShape = rlss::CollisionShape<T,DIM>;

    RLSSGoalSelector(
        T deshor,
        const PiecewiseCurve& origtraj,
        const AlignedBox& ws,
        std::shared_ptr<CollisionShape> colsha,
        T search_step
    ) : m_desired_horizon(deshor),
        m_original_trajectory(origtraj),
        m_workspace(ws),
        m_collision_shape(colsha),
        m_search_step(search_step) /* time allocated for searching to occur */
    {}

    std::optional<std::pair<VectorDIM, T>> select
    (
            const VectorDIM& current_position,
            const OccupancyGrid& occupancy_grid,
            T current_time  // current time = 0.0, current time + replaning period(0.01) per cycle; 
    ) override {
//        UnorderedIndexSet reachable_indices
//                = rlss::internal::BFS<T, DIM>(
//                        current_position,
//                        occupancy_grid,
//                        m_workspace,
//                        m_collision_shape
//        );

        T target_time = std::min(
                current_time + m_desired_horizon, // cannot assume we start from zero, clock waits for no man
                m_original_trajectory.maxParameter()
        );
        VectorDIM target_position = m_original_trajectory.eval(target_time, 0); // null derivative value (position) at target time (current time + horizon), max time it will reach will be its duration
        bool target_time_valid = false;
        std::vector<Index> neighbors = occupancy_grid.getNeighbors(
                target_position
        );
        neighbors.push_back(occupancy_grid.getIndex(target_position));
        /* this part n below is causing all this shit */
        for(const Index& neigh_idx: neighbors) {
            if(
                /*reachable_indices.find(neigh_idx) != reachable_indices.end()
                &&*/ rlss::internal::segmentValid<T, DIM>(
                    occupancy_grid,
                    m_workspace,
                    target_position,
                    neigh_idx,
                    m_collision_shape
            )
                    ) {
                target_time_valid = true;
                break;
            }
        }

        bool forward = true;
        long long int step_count = 1;
        while(
                !target_time_valid
                && (target_time + step_count * m_search_step
                    <= m_original_trajectory.maxParameter()
                    || target_time - step_count * m_search_step >= 0)
                ) {
            T candidate_target_time = target_time;
            if(forward) {
                candidate_target_time += step_count * m_search_step;
            } else {
                candidate_target_time -= step_count * m_search_step;
            }

            if(
                    candidate_target_time >= 0
                    && candidate_target_time <= m_original_trajectory.maxParameter()
                    ) {
                VectorDIM candidate_target_position
                        = m_original_trajectory.eval(candidate_target_time, 0);
                std::vector<Index> neighbors = occupancy_grid.getNeighbors(
                        candidate_target_position
                );
                neighbors.push_back(
                        occupancy_grid.getIndex(candidate_target_position)
                );

                for(const Index& neigh_idx: neighbors) {
                    if(
                        /*reachable_indices.find(neigh_idx)
                            != reachable_indices.end()
                        &&*/ rlss::internal::segmentValid<T, DIM>(
                            occupancy_grid,
                            m_workspace,
                            candidate_target_position,
                            neigh_idx,
                            m_collision_shape
                    )
                            ) {
                        target_time = candidate_target_time;
                        target_position = candidate_target_position;
                        target_time_valid = true;
                        break;
                    }
                }
            }

            if(forward) {
                forward = false;
            } else {
                forward = true;
                step_count++;
            }
        }

        if(!target_time_valid) {
            return std::nullopt;
        }

        T actual_horizon = target_time - current_time; // ends up having negative horizon as max_parameter is a scalar whereas current time has direction
        
        return std::make_pair(target_position, actual_horizon); //horizon shud be 8s, and target position = m_original_trajectory.eval(current time + horizon, 0)
        // for now, use target_time, means at every point, the horizon would be increasing as the drone 
        // isnt moving in current time, all the way up to max.param of curve so effectively, it would be 
        // 5 seconds horizon at each point to the end until it reaches close enuff distance
        // in reality, once the distance is close enough, traj will stop sending and planner activation
        // would be false, thus, the drone will jus hover there. This applies also when the user sends the drone to fly to the current spot 
        
    }

    void setOriginalTrajectory(const PiecewiseCurve& origtraj) {
        m_original_trajectory = origtraj;
    }
private:
    T m_desired_horizon;
    PiecewiseCurve m_original_trajectory;
    AlignedBox m_workspace;
    std::shared_ptr<CollisionShape> m_collision_shape;
    T m_search_step;
};

} // namespace rlss

#endif // RLSS_RLSS_GOAL_SELECTOR_HPP