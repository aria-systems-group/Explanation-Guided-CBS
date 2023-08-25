#pragma once
#include <vector>
#include <queue>
#include <unordered_set>
#include <thread>
#include "../includes/Timer.h"
#include "../includes/Conflict.h"
#include "../includes/Astar.h"


// XG - A* with heuristics
class S_Astar
{
public:
    // constructor
	S_Astar(Environment *env, const bool useCBS = true);

	// main planning function
    bool plan(State *startState, std::vector<State*> &solution, 
        const std::vector<Constraint*> relevantConstraints, 
        std::vector<std::vector<State*>>& parent, bool &done);

    // check if two path segments are disjoint
    bool is_disjoint(const std::vector<State*> v1, 
        const std::vector<State*> v2) const;

    int segmentSolution(const std::vector<State*> path, const std::vector<std::vector<State*>>& otherSols);

    const std::vector<timedObstacle> plan2obs(const State* start, std::vector<std::vector<State*>>& existingPlan);

    // get the environment object
	Environment* getEnv() {return m_env;};
private:
	Environment *m_env;  // saves the object
    const bool useCBS;  // saves the answer of using CBS
    A_star m_Astar;  // A* for heuristic planning
};