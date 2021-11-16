#pragma once
// my includes
#include "../includes/State.h"
#include "../includes/Conflict.h"
// standard includes
#include <unordered_set>
#include <set>
#include <map>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

typedef std::pair<const int, const int> vertex;
typedef std::set<vertex> cell;


// Lowest-level Datatype used by evnironment
struct Location 
{
  Location(int x, int y) : x(x), y(y) {}
  const int x;
  const int y;

  bool operator==(const Location *other) const {
    return std::tie(x, y) == std::tie(other->x, other->y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location *l) {
    return os << "(" << l->x << "," << l->y << ")";
  }
};

// default hash constructor
namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location *s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s->x);
    boost::hash_combine(seed, s->y);
    return seed;
  }
};
}  // namespace std

// Main Env class -- contains useful info regarding the graph
class Environment {
    public:
        Environment(const int dimx, const int dimy, std::unordered_set<Location*> obstacles,
                std::vector<State*> starts, std::vector<Location*> goals, std::vector<std::string> names):
            m_dimx(dimx),
            m_dimy(dimy),
            m_obstacles(std::move(obstacles)),
            m_starts(std::move(starts)),
            m_goals(std::move(goals)),
            m_agentIdx(0),
            m_agentNames(names)
            {}

        // typical heuristic score when given a state
        double heuristicFunc(const State *st) const
        {
            // manhattan dist. from state to goal
            // use this heuristic when we can only move in 4 cardinal directions
            return abs(st->x - (m_goals[m_agentIdx])->x) + abs(st->y - (m_goals[m_agentIdx])->y);
        }

        // find valid neighbors of a state
        void expandState(const State *st, std::vector<State*>& neighbors, 
            std::vector<Constraint*> constraints, bool isNodeWaiting);

        // Find if state is valid, given constraints
        bool isStateValid(const State *curr, const State *nxt, const std::vector<Constraint*> constraints) const;

        // abstract graph to (sz x sz) <= (m_dimx x m_dimy)
        void abstractGraph(const int sz);

        // macros function -- if using only low-level planner, can include collision checks
        void includeCollisionChecks(const std::vector<std::vector<State*>> parentSol)
        {
            useCollisionChecking = true;
            m_existingSol = parentSol;
        }

        // check if a node is goal 
        bool isStateGoal(const State *st) const
        {
            return (st->x == m_goals[m_agentIdx]->x && st->y == m_goals[m_agentIdx]->y);
        }

        // CBS usually plans for agents out of order -- this makes sure we plan for correct one
        void updateAgent()
        {
            if (m_agentIdx < m_goals.size())
                m_agentIdx ++;
            else
                m_agentIdx = 0;
        }

        // returns agent number
        int getAgent() {return m_agentIdx;};
        // returns list of starts
        const std::vector<State*> getStarts() {return m_starts;};
        // returns list of goals
        const std::vector<Location*> getGoals() {return m_goals;};
        // get size of space
        const int getXdim() {return m_dimx;};
        const int getYdim() {return m_dimy;};
        // get agent names
        const std::vector<std::string> getAgentNames() {return m_agentNames;};

        void setMapName(const std::string str) {m_mapName = str;};
        const std::string getMapName() {return m_mapName;};

    private:
        bool useCollisionChecking = false;  // saves boolean for using collision checks
        const int m_dimx;  // saves x-value of space
        const int m_dimy;  // saves y-value of space
        const std::unordered_set<Location*> m_obstacles;  // saves list of obstacles
        const std::vector<State*> m_starts;  // saves list of goals
        const std::vector<Location*> m_goals;  // saves list of goals
        std::vector<std::vector<State*>> m_existingSol;  // saves the parent solution
        int m_agentIdx; // agent index that tracks which agent we plan for
        std::vector<std::string> m_agentNames; // saves names of agents
        std::string m_mapName;
};
