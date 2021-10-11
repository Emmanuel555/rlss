#ifndef RLSS_DISCRETESEARCH_HPP
#define RLSS_DISCRETESEARCH_HPP

#include <rlss/internal/Util.hpp>
#include <rlss/OccupancyGrid.hpp>
#include <rlss/CollisionShapes/CollisionShape.hpp>
#include <libMultiRobotPlanning/a_star.hpp>
#include <boost/functional/hash/hash.hpp>
#include <optional>
#include <iostream>


namespace rlss {

namespace internal {



template<typename T, unsigned int DIM>
std::optional<StdVectorVectorDIM<T, DIM>> discreteSearch(
            const typename OccupancyGrid<T, DIM>::Coordinate& start_coordinate,
            const typename OccupancyGrid<T, DIM>::Coordinate& goal_coordinate,
            const OccupancyGrid<T, DIM>& occupancy_grid,
            const AlignedBox<T, DIM>& workspace,
            std::shared_ptr<rlss::CollisionShape<T,DIM>> collision_shape
) {
    using VectorDIM = VectorDIM<T, DIM>;
    using OccupancyGrid = OccupancyGrid<T, DIM>;
    using AlignedBox = AlignedBox<T, DIM>;
    using StdVectorVectorDIM = StdVectorVectorDIM<T, DIM>;
    using Index = typename OccupancyGrid::Index;
    using Coordinate = typename OccupancyGrid::Coordinate;
    using CollisionShape = rlss::CollisionShape<T, DIM>;


    Index starter_idx = occupancy_grid.getIndex(start_coordinate);
    OccupancyGrid trial_occupancy = occupancy_grid;
    //trial_occupancy.setOccupancy(Coordinate(0.034,0.045,-0.067));
    //trial_occupancy.removeOccupancy(start_coordinate);
    //trial_occupancy.clearTemporaryObstacles(); // this works 
    //std::cout << "Hello_1" << "  " << trial_occupancy.isOccupied(start_coordinate) << std::endl; // even tho it says clear temp obs, start coordinate is still occupied somehow 

    struct State {
        Coordinate position;
        Index dir;
        explicit State(Coordinate c) : position(c), dir(Index::Zero()) {}
        State(Coordinate c, Index d) : position(c), dir(d) {}
        bool operator==(const State& rhs) const {
            return position == rhs.position && dir == rhs.dir;
        }
    };

    struct StateHasher {
        std::size_t operator()(const State& s) const {
            std::size_t seed = 0;
            for(unsigned int d = 0; d < DIM; d++) {
                boost::hash_combine(seed, s.position(d));
                boost::hash_combine(seed, s.dir(d));
            }
            return seed;
        }
    };

    enum class Action {
        FORWARD,
        ROTATE,
        ROTATEFORWARD
    };

    class Environment {
    public:

        Environment(
            const OccupancyGrid& occ, 
            const AlignedBox& works, 
            std::shared_ptr<CollisionShape> cols,
            const Coordinate& goal
            )
            : m_occupancy_grid(occ),
              m_workspace(works),
              m_collision_shape(cols),
              m_goal(goal)
        {}

        int admissibleHeuristic(const State& s) {
            int h = 0;
            Index state_idx = m_occupancy_grid.getIndex(s.position);
            Index goal_idx = m_occupancy_grid.getIndex(m_goal);
            //std::cout << "occupancy bool in admissible heuristics" << "  " << m_occupancy_grid.isOccupied(s.position) << std::endl;
            return (state_idx - goal_idx).norm(); // 18
//            for(unsigned int d = 0; d < DIM; d++) {
//                h += std::abs(state_idx(d) - goal_idx(d));
//            }
//            return h;
        }

        bool isSolution(const State& s) { return s.position == m_goal; }

        void getNeighbors(
            const State& s,
            std::vector<libMultiRobotPlanning::Neighbor<State, Action, int> >& 
                neighbors) {

            neighbors.clear();

            Coordinate s_center = m_occupancy_grid.getCenter(s.position);
            Index s_idx = m_occupancy_grid.getIndex(s_center);

            if(this->actionValid(s.position, m_goal)) {
                neighbors.emplace_back(
                    State(m_goal, Index::Zero()), 
                    Action::ROTATEFORWARD, 
                    1 + (s_idx - m_occupancy_grid.getIndex(m_goal))
                                  .norm()
//                                .cwiseAbs().sum()
                );
            }

            if(s_center != s.position) { // get into grid
                if(this->actionValid(s.position, s_center)) {
                    neighbors.emplace_back(
                        State(s_center, Index::Zero()), 
                        Action::ROTATEFORWARD,
                        2
                    );
                }

                std::vector<Index> neigh_indexes
                        = m_occupancy_grid.getNeighbors(s_idx);

                for(const auto& neigh_idx: neigh_indexes) {
                    auto neigh_center = m_occupancy_grid.getCenter(neigh_idx);
                    if(this->actionValid(s.position, neigh_center)) {
                        neighbors.emplace_back(
                                State(neigh_center, Index::Zero()),
                                Action::ROTATEFORWARD,
                                2
                        );
                    }
                }
            } else {
                if(s.dir == Index::Zero()) {
                    std::vector<Index> neigh_indexes
                            = m_occupancy_grid.getNeighbors(s_idx);
                    for(const auto& neigh_idx : neigh_indexes) {
                        if (this->actionValid(s_idx, neigh_idx)) {
                            neighbors.emplace_back(
                                State(
                                    m_occupancy_grid.getCenter(neigh_idx),
                                    neigh_idx - s_idx
                                ),
                                Action::FORWARD,
                                1
                            );
                        }
                    }
                } else {
                    Index idx = s_idx + s.dir;
                    if(this->actionValid(s_idx, idx)) {
                        neighbors.emplace_back(
                            State(m_occupancy_grid.getCenter(idx), s.dir), 
                            Action::FORWARD, 
                            1
                        );
                    }

                    for(unsigned int d = 0; d < DIM; d++) {
                        if(s.dir(d) == 0) {
                            Index dir = Index::Zero();
                            dir(d) = 1;
                            neighbors.emplace_back(
                                State(m_occupancy_grid.getCenter(s_idx), dir), 
                                Action::ROTATE, 
                                1
                            );
                            dir(d) = -1;
                            neighbors.emplace_back(
                                State(m_occupancy_grid.getCenter(s_idx), dir), 
                                Action::ROTATE, 
                                1
                            );
                        }
                    }
                }
            }

            
        }

        void onExpandNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

        void onDiscover(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

    public:

        bool actionValid(const Coordinate& start, const Coordinate& end) {
            return rlss::internal::segmentValid<T, DIM>(
                    m_occupancy_grid,
                    m_workspace,
                    start,
                    end,
                    m_collision_shape
            );
        }


        bool actionValid(const Index& from_idx, const Index& to_idx) {
            Coordinate from_center = m_occupancy_grid.getCenter(from_idx);
            Coordinate to_center = m_occupancy_grid.getCenter(to_idx);

            return this->actionValid(from_center, to_center);
        }


        bool positionValid(const Coordinate& pos) {
            AlignedBox robot_box = m_collision_shape->boundingBox(pos);
            return m_workspace.contains(robot_box) // the robot's location must remain 
                    && !m_occupancy_grid.isOccupied(robot_box); // true if occupancy grid isnt occupied at where the robot currently is
        }

        bool indexValid(const Index& idx) {
            Coordinate center = m_occupancy_grid.getCenter(idx);
            return this->positionValid(center);
        }

    private:
        const OccupancyGrid& m_occupancy_grid;
        const AlignedBox& m_workspace;
        std::shared_ptr<CollisionShape> m_collision_shape;
        const Coordinate& m_goal;
    };
    
    
    Environment env(
            occupancy_grid,
            //trial_occupancy,
            workspace,
            collision_shape,
            goal_coordinate
    );

    
    libMultiRobotPlanning::AStar<State, Action, int, Environment, StateHasher>
            astar(env);
    libMultiRobotPlanning::PlanResult<State, Action, int> solution;

    /*std::cout << "Coordinates and test boxes" << std::endl;
    std::cout << start_coordinate[0] << std::endl;
    std::cout << start_coordinate[1] << std::endl;
    std::cout << start_coordinate[2] << std::endl;
    std::cout << env.positionValid(start_coordinate) << std::endl;
    AlignedBox test_box = collision_shape->boundingBox(start_coordinate);
    std::cout << trial_occupancy.isOccupied(start_coordinate) << std::endl;
    std::cout << workspace.contains(test_box) << std::endl;
    std::cout << (test_box.max())[0] << std::endl;
    std::cout << (test_box.min())[0] << std::endl;
    std::cout << (test_box.center())[0] << std::endl;*/
    

    if(env.positionValid(start_coordinate)) {
        State start_state(start_coordinate);
        //std::cout << "testing heuristics" << std::endl;
        //std::cout << env.admissibleHeuristic(start_state) << std::endl;
        bool success = astar.search(start_state, solution);
        if(!success) {
            std::cout << "failed astar" << std::endl;
            return std::nullopt;
        }
        
        StdVectorVectorDIM segments;
        segments.push_back(solution.states[0].first.position);
        for(std::size_t i = 0; i < solution.actions.size(); i++) {
            std::string action_text;
            if(solution.actions[i].first == Action::ROTATE) {
                action_text = "ROTATE";
            } else if(solution.actions[i].first == Action::FORWARD) {
                action_text = "FORWARD";
            } else if(solution.actions[i].first == Action::ROTATEFORWARD) {
                action_text = "ROTATEFORWARD";
            }
            debug_message(solution.states[i].first.position.transpose(),
                    ",", solution.states[i].first.dir.transpose(), " > ",
                    solution.states[i+1].first.position.transpose(), ",",
                    solution.states[i+1].first.dir.transpose(), ", action: ",
                      action_text);
            if(solution.actions[i].first == Action::ROTATE) {
                segments.push_back(solution.states[i+1].first.position);
            } else if (solution.actions[i].first == Action::ROTATEFORWARD) {
                if(i != 0
                    && segments.back() != solution.states[i].first.position)
                    segments.push_back(solution.states[i].first.position);
                if(i != solution.actions.size() - 1
                    && segments.back() != solution.states[i+1].first.position)
                    segments.push_back(solution.states[i+1].first.position);
            }
        }
        segments.push_back(solution.states.back().first.position);

        return segments;
    }

    return std::nullopt;
}


} // namespace internal
} // namespace rlss

#endif // RLSS_DISCRETESEARCH_HPP