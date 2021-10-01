#ifndef RLSS_RLSS_DISCRETE_PATH_SEARCHER_HPP
#define RLSS_RLSS_DISCRETE_PATH_SEARCHER_HPP

#include <rlss/internal/Util.hpp>
#include <rlss/DiscretePathSearchers/DiscretePathSearcher.hpp>
#include <rlss/CollisionShapes/CollisionShape.hpp>
#include <rlss/internal/DiscreteSearch.hpp>

namespace rlss {
template<typename T, unsigned int DIM>
class RLSSDiscretePathSearcher: public DiscretePathSearcher<T, DIM> {
public:

    using Base = DiscretePathSearcher<T, DIM>;
    using OccupancyGrid = typename Base::OccupancyGrid;
    using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<T,DIM>;
    using VectorDIM = typename Base::VectorDIM;
    using AlignedBox = rlss::internal::AlignedBox<T, DIM>;
    using CollisionShape = rlss::CollisionShape<T, DIM>;

    RLSSDiscretePathSearcher(
        T safe_upto,
        const AlignedBox& ws,
        std::shared_ptr<CollisionShape> colsha,
        T maxvel,
        std::size_t num_pieces
    ) : m_safe_upto(safe_upto), // replanning period 
        m_workspace(ws),
        m_collision_shape(colsha),
        m_maximum_velocity(maxvel),
        m_num_pieces(num_pieces) // 4
    {

    }


    /* auto discrete_path_test =
                rlss::internal::discreteSearch<T, DIM>(
                        current_position, // if current pos is stuck, current time is just gonna drift further resulting in a further target pos which means the drone needs to fly there faster ( increased vel)
                        goal_position,
                        occupancy_grid,
                        m_workspace,
                        m_collision_shape
                );

        StdVectorVectorDIM result = *discrete_path_test;
        std::cout << result.size() << std::endl; */


    std::optional<std::pair<StdVectorVectorDIM, std::vector<T>>>
    search(
            const VectorDIM& current_position,
            const VectorDIM& goal_position,
            T time_horizon, // probably taken from desired time horizon in RLSS::planner
            const OccupancyGrid& occupancy_grid
    ) override {
        time_horizon = std::max(time_horizon, m_safe_upto);

        std::optional<StdVectorVectorDIM> discrete_path_opt =
                rlss::internal::discreteSearch<T, DIM>(
                        current_position, // if current pos is stuck, current time is just gonna drift further resulting in a further target pos which means the drone needs to fly there faster ( increased vel)
                        goal_position,
                        occupancy_grid,
                        m_workspace,
                        m_collision_shape
                );

        std::cout << "?????" << std::endl;
        std::cout << current_position[0] << std::endl;
        std::cout << current_position[1] << std::endl;
        std::cout << current_position[2] << std::endl;
        std::cout << goal_position[0] << std::endl;
        std::cout << goal_position[1] << std::endl;
        std::cout << goal_position[2] << std::endl;
        std::cout << time_horizon << std::endl;


        if(!discrete_path_opt) {
            std::cout << "?????" << std::endl;
            std::cout << current_position[0] << std::endl;
            std::cout << current_position[1] << std::endl;
            std::cout << current_position[2] << std::endl;
            std::cout << typeid(goal_position[0]).name() << std::endl;
            std::cout << goal_position[1] << std::endl;
            std::cout << goal_position[2] << std::endl;
            std::cout << time_horizon << std::endl;
            //std::cout << m_collision_shape. << std::endl;
            std::cout << (m_collision_shape->boundingBox(current_position).center())[0] << std::endl;
            return std::nullopt;
        }

        StdVectorVectorDIM discrete_path = std::move(*discrete_path_opt);

        discrete_path = rlss::internal::firstSegmentFix<T, DIM>(discrete_path); // taken from discrete_path_opt

        T total_path_length = 0;
        for(std::size_t i = 0; i < discrete_path.size() - 1; i++) {
            total_path_length
                    += (discrete_path[i+1] - discrete_path[i]).norm(); // norm refers to magnitude
        }
        time_horizon = std::max(
                time_horizon,
                total_path_length/m_maximum_velocity
        );

        debug_message("discrete path length returned from the solver is ",
                      discrete_path.size());

        if(discrete_path.size() > m_num_pieces + 1) {
            debug_message("however it has more pieces than required.");
            discrete_path.resize(m_num_pieces + 1);
            debug_message("resized. new discrete path length is ",
                          discrete_path.size());
        } else if(discrete_path.size()  < m_num_pieces + 1) {
            debug_message("however it has less pieces than required.");
            discrete_path = rlss::internal::bestSplitSegments<T, DIM>(
                    discrete_path,
                    m_num_pieces
            );
            debug_message("splitted. new discrete path length is ",
                          discrete_path.size());
        }

        std::vector<T> segment_lengths(m_num_pieces); /// num_pieces = 4 for most of the time, need to test from here all the way to the bottom tmr
        total_path_length = 0;
        for(std::size_t i = 0; i + 1 < discrete_path.size(); i++) {
            segment_lengths[i]
                    = (discrete_path[i+1] - discrete_path[i]).norm();
            total_path_length
                    += segment_lengths[i];
        }


        std::vector<T> segment_durations(m_num_pieces, 0);
        for(std::size_t i = 0; i < segment_lengths.size(); i++) {
            segment_durations[i]
                    = time_horizon * (segment_lengths[i] / total_path_length); // the fraction of the time horizon dedicated for each segment length
        }

        if(!segment_durations.empty() && segment_durations[0] < m_safe_upto) {
            T multiplier = m_safe_upto / segment_durations[0];
            segment_durations[0] = m_safe_upto;
            for(std::size_t i = 1; i < segment_durations.size(); i++) {
                segment_durations[i] *= multiplier;
            }
        }

        return std::make_pair(discrete_path, segment_durations); // figure a way to extract this 
    };
private:
    T m_safe_upto;
    AlignedBox m_workspace;
    std::shared_ptr<CollisionShape> m_collision_shape;
    T m_maximum_velocity;
    std::size_t m_num_pieces;
}; // class RLSSDiscretePathSearcher
} // namespace rlss

#endif // RLSS_RLSS_DISCRETE_PATH_SEARCHER_HPP