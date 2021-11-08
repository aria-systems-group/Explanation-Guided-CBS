#pragma once
// my conflict
#include "../includes/Astar.h"
#include "../includes/Conflict.h"
#include "../includes/Timer.h"
#include "Environment.h"
// standard includes
#include <unordered_set>
#include <thread>


// shorthand for a Plan 
typedef std::vector<std::vector<State*>> Solution;

// CBS
class CBS
{
public:
	// constructor
	CBS(Environment *env);
	// main planning function -- returns plan
	bool plan(const std::vector<State*>& startStates, Solution& solution);

	// set the solve time
    void setSolveTime(const double t) {solveTime_ = t;};
	
	// High-level node of Conflict Tree
	struct conflictNode
	{
	public:

		conflictNode(Solution solution): m_solution(solution)
		{
			// constructor
			m_cost = calcCost();
		};

		// Calculates the SoC value of a conflict Node
		int calcCost()
		{
			int cost  = 0;
			for (std::vector<State*> sol: m_solution)
			{
				cost = cost + sol.size();
			}
			return cost;
		};

		Solution m_solution;  // saves the plan
		Constraint m_constraint;  // saves the constraint of a conflict Node
		int m_cost{std::numeric_limits<int>::infinity()};  // saves the cost of a conflict Node
		conflictNode *parent{nullptr};  // saves parent of a conflict Node
	};

	// main low-level graph search function
	Solution lowLevelSearch(const std::vector<State*>& startStates, 
		std::vector<Constraint*> constriants, Solution& parent, bool &isDone);
	
	// Minimal Disjoint Segmentation Alg.
	Conflict* validateSolution(conflictNode *n);

	// Minimal Disjoint Segmentation Alg. -- run only at end of planning loop
	int segmentSolution(Solution sol);

	// Check if to path segments are disjoint
	bool is_disjoint(const std::vector<State*> v1, const std::vector<State*> v2) const;

	// get the total number of agents in the problem
	int getAgents() {return m_numAgents;};

	// priority queue -- compare two conflict nodes
	class myconflictComparator
	{
	public:
    	int operator() (const conflictNode *n1, const conflictNode *n2)
    	{
        	return n1->m_cost > n2->m_cost;
    	}
	};

protected:
	Environment *m_env;  // saves the environment object
	int m_numAgents;  // saves the total number of agents
	A_star *m_planner{nullptr};  // saves the A^* planner
	double solveTime_{std::numeric_limits<double>::infinity()};
};
