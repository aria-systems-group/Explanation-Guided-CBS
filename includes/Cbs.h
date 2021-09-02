#pragma once
#include <unordered_set>
#include "Environment.h"
#include "../includes/Astar.h"
#include "../includes/Conflict.h"

typedef std::vector<std::vector<State*>> Solution;


class CBS
{
public:
	CBS(Environment *env);

	bool plan(const std::vector<State*>& startStates, Solution& solution);

	struct conflictNode
	{
	public:

		conflictNode(Solution solution): m_solution(solution)
		{
			m_cost = calcCost();
			// m_constraints = new Constraints();

		};

		// bool is_disjoint(const std::vector<State*> v1, const std::vector<State*> v2) const;

		// int segmentSolution(Solution &sol);

		// int segmentSolution();

		int calcCost()
		{
			// return segmentSolution();
			int cost  = 0;
			for (std::vector<State*> sol: m_solution)
			{
				cost = cost + sol.size();
			}
			return cost;
		};

		Solution m_solution;
		Constraint m_constraint;
		int m_cost{std::numeric_limits<int>::infinity()};
		conflictNode *parent{nullptr};
	};

	Solution lowLevelSearch(const std::vector<State*>& startStates, 
		std::vector<Constraint*> constriants, Solution& parent);

	Conflict* validateSolution(conflictNode *n);

	int segmentSolution(Solution sol);

	bool is_disjoint(const std::vector<State*> v1, const std::vector<State*> v2) const;

	int getAgents() {return m_numAgents;};

	// for open heap
	class myconflictComparator
	{
	public:
    	int operator() (const conflictNode *n1, const conflictNode *n2)
    	{
        	return n1->m_cost > n2->m_cost;
    	}
	};

protected:
	Environment *m_env;
	int m_numAgents;
	A_star *m_planner{nullptr};
};
