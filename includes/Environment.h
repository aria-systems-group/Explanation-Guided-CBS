#include <unordered_set>


typedef int Cost; // easily differentiate between Cost and other data

// define action
enum class Action {
  Up,
  Down,
  Left,
  Right,
  Wait,
};

class Environment {
    public:
        Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
                std::vector<Location> goals):
            m_dimx(dimx),
            m_dimy(dimy),
            m_obstacles(std::move(obstacles)),
            m_goals(std::move(goals)),
            m_agentIdx(0)
            {}

        // Environment(const Environment&) = delete;
        // Environment& operator=(const Environment&) = delete;
        std::vector<Location> getGoals()
        {
            return m_goals;
        }

        Cost heuristicFunc(const State& st) const
        {
            std::cout << "I am here now -- heuristicFunc" << std::endl;
            return 0;
        }

        void expandState(const State st, std::vector<Neighbor<State, Action, Cost>> neighbors)
        {
            // Cost is the price to be at that state -- usually 1 for all
            neighbors.clear();

            // init and check "up" state
            State up(st.time + 1, st.x, st.y + 1);
            // init and check "down" state
            State down(st.time + 1, st.x, st.y - 1);
            // init and check "right" state
            State right(st.time + 1, st.x + 1, st.y);
            // init and check "left" state
            State left(st.time + 1, st.x - 1, st.y);

            if (isStateValid(up))
            {
                neighbors.emplace_back(Neighbor<State, Action, Cost>(up, Action::Up, 1));
            }
            if (isStateValid(down))
            {
                neighbors.emplace_back(Neighbor<State, Action, Cost>(up, Action::Down, 1));
            }
            if (isStateValid(right))
            {
                neighbors.emplace_back(Neighbor<State, Action, Cost>(up, Action::Right, 1));
            }
            if (isStateValid(left))
            {
                neighbors.emplace_back(Neighbor<State, Action, Cost>(up, Action::Left, 1));
            }
        }

        bool isStateValid(const State st) const
        {
            // is in env bounds
            if (st.x > m_dimx || st.y > m_dimy)
                return false;
            // is in obstacles
            for (auto& obs : m_obstacles)
            {
                if (st.x == obs.x && st.y == obs.y)
                    return false;
            }
            return true;
        }

        bool isStateGoal(const State st) const
        {
            return (st.x == m_goals[m_agentIdx].x && st.y == m_goals[m_agentIdx].y);
        }

    private:
        const int m_dimx;
        const int m_dimy;
        const std::unordered_set<Location> m_obstacles;
        const std::vector<Location> m_goals;
        int m_agentIdx;  // this cycles through the agents so that A* does not need to worry about it
};

