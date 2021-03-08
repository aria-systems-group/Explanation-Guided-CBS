#pragma once
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include "../includes/State.h"
#include "../includes/Conflict.h"


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

// default constructor
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

class Environment {
    public:
        Environment(const int dimx, const int dimy, std::unordered_set<Location*> obstacles,
                std::vector<Location*> goals):
            m_dimx(dimx),
            m_dimy(dimy),
            m_obstacles(std::move(obstacles)),
            m_goals(std::move(goals)),
            m_agentIdx(0)
            {}

        double heuristicFunc(const State *st) const
        {
            // manhattan dist. from state to goal
            // use this heuristic when we can only move in 4 cardinal directions
            return abs(st->x - (m_goals[m_agentIdx])->x) + abs(st->y - (m_goals[m_agentIdx])->y);
        }

        void expandState(const State *st, std::vector<State*>& neighbors, 
            std::vector<Constraint*> constraints)
        {
            // clear previous data
            neighbors.clear();

            // init and check "up" state
            State *up = new State(st->time + 1, st->x, st->y + 1);
            // init and check "down" state
            State *down = new State(st->time + 1, st->x, st->y - 1);
            // init and check "right" state
            State *right = new State(st->time + 1, st->x + 1, st->y);
            // init and check "left" state
            State *left = new State(st->time + 1, st->x - 1, st->y);

            if (isStateValid(st, up, constraints))
            {
                neighbors.push_back(up);
            }
            if (isStateValid(st, down, constraints))
            {
                neighbors.push_back(down);
            }
            if (isStateValid(st, right, constraints))
            {
                neighbors.push_back(right);
            }
            if (isStateValid(st, left, constraints))
            {
                neighbors.push_back(left);
            }
        }

        bool isStateValid(const State *curr, const State *nxt, const std::vector<Constraint*> constraints) const
        {
            // is in env bounds
            if ( 0 > nxt->x || nxt->x > m_dimx || 0 > nxt->y || nxt->y > m_dimy)
                return false;
            // is in obstacles
            for (Location *obs : m_obstacles)
            {
                if (nxt->x == obs->x && nxt->y == obs->y)
                    return false;
            }

            if (useCollisionChecking)
            {
                // from exising solutions, get a list of states to check
                std::vector<State*> needCheck;
                for (std::vector<State*> sol: m_existingSol)
                {
                    if (nxt->time <= sol.back()->time)
                        needCheck.push_back(sol[nxt->time]);
                }
                // next, check that needCheck and curr are not the same state
                for (State *st: needCheck)
                {
                    if (st->isSameLocation(nxt))
                        return false;
                }
            }
            // need to also account for constraints
            // iterate through constraints and see if state matches any, 
            // if so, return false
            for (Constraint *c: constraints)
            {
                // note that State == is overloaded and we already provide 
                // agent relevant constraints
                VertexConstraint *v = c->getVertexConstraint();
                EdgeConstraint *e = c->getEdgeConstraint();
                if (v != nullptr)
                {
                    if ( (v->x == nxt->x) && (v->y == nxt->y) && (v->time == nxt->time))
                        return false;
                }

                if (e != nullptr)
                {
                    if ((e->x1 == curr->x) && (e->y1 == curr->y) && (e->time1 == curr->time))
                    {
                        if ((e->x2 == nxt->x) && (e->y2 == nxt->y) && (e->time2 == nxt->time))
                        {
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        void includeCollisionChecks(const std::vector<std::vector<State*>> parentSol)
        {
            useCollisionChecking = true;
            m_existingSol = parentSol;
        }

        bool isStateGoal(const State *st) const
        {
            return (st->x == m_goals[m_agentIdx]->x && st->y == m_goals[m_agentIdx]->y);
        }

        void updateAgent()
        {
            if (m_agentIdx < m_goals.size())
                m_agentIdx ++;
            else
                m_agentIdx = 0;
        }

        int getAgent() {return m_agentIdx;}

        const std::vector<Location*> getGoals() {return m_goals;};

        const int getXdim() {return m_dimx;};
        const int getYdim() {return m_dimy;};

    private:
        bool useCollisionChecking = false;
        const int m_dimx;
        const int m_dimy;
        const std::unordered_set<Location*> m_obstacles;
        const std::vector<Location*> m_goals;
        std::vector<std::vector<State*>> m_existingSol;
        int m_agentIdx;  // this cycles through the agents so that A* does not need to worry about it
};