#pragma once
#include <unordered_set>
#include "../includes/State.h"
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>


// define action
enum class Action {
  Up,
  Down,
  Left,
  Right,
  Wait,
};

struct Location 
{
  Location(int x, int y) : x(x), y(y) {}
  const int x;
  const int y;

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }
};

// default constructor
namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

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

        double heuristicFunc(const State& st) const
        {
            // manhattan dist. from state to goal
            // use this heuristic when we can only move in 4 cardinal directions
            return abs(st.x - m_goals[m_agentIdx].x) + abs(st.y - m_goals[m_agentIdx].y);
        }

        void expandState(const State st, std::vector<State>& neighbors)
        {
            // clear previous data
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
                neighbors.push_back(up);
            }
            if (isStateValid(down))
            {
                neighbors.push_back(down);
            }
            if (isStateValid(right))
            {
                neighbors.push_back(right);
            }
            if (isStateValid(left))
            {
                neighbors.push_back(left);
            }
        }

        bool isStateValid(const State st) const
        {
            // is in env bounds
            if ( 0 > st.x || st.x > m_dimx || 0 > st.y || st.y > m_dimy)
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

        void updateAgent()
        {
            if (m_agentIdx < m_goals.size())
                m_agentIdx ++;
            else
                m_agentIdx = 0;
        }

        // std::vector<Location> getGoals() {return m_goals;}

        int getAgent() {return m_agentIdx;}

    private:
        const int m_dimx;
        const int m_dimy;
        const std::unordered_set<Location> m_obstacles;
        const std::vector<Location> m_goals;
        int m_agentIdx;  // this cycles through the agents so that A* does not need to worry about it
};