#pragma once
#include <unordered_set>
#include "Environment.h"
#include "../includes/Astar.h"
#include "../includes/Conflict.h"

typedef std::vector<std::vector<State*>> Solution;

// EXP - CBS
class CBS
{
public:
	CBS(Environment *env, const int bound);

	bool plan(const std::vector<State*>& startStates, Solution& solution);

	struct conflictNode
	{
	public:

		conflictNode(Solution solution): m_solution(solution)
		{
			m_cost = calcCost();
			// m_constraints = new Constraints();

		};

		int calcCost()
		{
			int cost = 0;
			for (std::vector<State*> sol: m_solution)
			{
				cost = cost + sol.size();
			}
			return cost;
		};

		int getSegCost() const
		{
			return m_solution[0].back()->cost;
		}

		Solution m_solution;
		Constraint m_constraint;
		int m_cost{std::numeric_limits<int>::infinity()};
		conflictNode *parent{nullptr};
	};

	Solution lowLevelSearch(const std::vector<State*>& startStates, 
		std::vector<Constraint*> constriants, Solution& parent);

	Conflict* validateSolution(conflictNode *n);

	int getAgents() {return m_numAgents;};

	// for open heap
	class myconflictComparator
	{
	public:
    	int operator() (const conflictNode *n1, const conflictNode *n2)
    	{
        	if (n1->getSegCost() == n2->getSegCost())
            {
                // which is shorter SOC
                return n1->m_cost > n2->m_cost;
            }
            else
            {
                // which segments are smaller
                return n1->getSegCost() > n2->getSegCost();
            }
    	}
	};

protected:
	Environment *m_env;
	const int m_bound;
	int m_numAgents;
	A_star *m_planner{nullptr};
};
