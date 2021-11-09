#pragma once
// my includes
#include "Environment.h"
#include "../includes/XG-Astar-H.h"
#include "../includes/XG-Astar.h"
#include "../includes/Astar.h"
#include "../includes/Conflict.h"
#include "../includes/Timer.h"
// standard includes
#include <chrono>
#include <unordered_set>
#include <iostream>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <sstream>
#include <functional>
#include <thread>


// shorthand for a Plan 
typedef std::vector<std::vector<State*>> Solution;

// EG - CBS
class XG_CBS
{
public:
	// constructor
	XG_CBS(Environment *env, const int bound);

	// main plan function -- returns plan
	bool plan(const std::vector<State*>& startStates, Solution& solution, const bool useEG, const bool useHeuristic);

	// clear as much data as possible
	void clear();

	// set the solve time
    void setSolveTime(const double t) {solveTime_ = t;};

    Environment* getEnv() {return m_env;};

	// High-level node of Conflict Tree
	struct conflictNode
	{
	public:
		// constructor
		conflictNode(Solution solution): m_solution(solution)
		{
			m_cost = calcCost();
		};

		// Calculates the SoC value of a conflict Node
		int calcCost()
		{
			int cost = 0;
			for (std::vector<State*> sol: m_solution)
			{
				cost = cost + sol.size();
			}
			return cost;
		};

		// Calculates the Segmentation Cost of a conflict Node
		int getSegCost() const
		{
			int cost = 0;
			for (std::vector<State*> sol: m_solution)
			{
				if (cost < sol.back()->cost)
					cost = sol.back()->cost;
			}
			return cost;
		};

		// print function for debugging purposes ONLY
		void print(std::ostream& os, int level, const conflictNode *curr, std::vector<Conflict*> cnf) 
		{

    		for (int i = 1; i < level; i++)
    		{
    	   		os << "\t\t ";
    		}

    		std::string text = std::to_string(this->m_idx) + "_" + std::to_string((this->children).size());

    		if (this->m_eval == true)
    			text = text + "_eval";
    		if (this == curr)
    		{
    			text = text + "_par";
    			// also print conflict vector
    			this->printConflict(cnf);
    		}
    		if (this->m_solFlag == true)
    			text = text + "_sol";

    		os << text << std::endl;

    		for (conflictNode *child : this->children)
    		{
    	   		child->print(os, level + 1, curr, cnf);
    	   	}
    	   	// print node information to different file
    	   	this->printInformation();
    	};
    	// print function for debugging purposes ONLY
    	void printConflict(std::vector<Conflict*> cnf)
    	{
    		std::string dirName = ("txt/nodes/" + std::to_string(this->m_idx));
			int check = mkdir(dirName.c_str(), 0777);
			// output solution
			std::string fileName = dirName + "/conflict.txt";
			std::ofstream os(fileName);

			for (Conflict *c: cnf)
			{
				os << *c << std::endl;
			}
    	}

    	// print function for debugging purposes ONLY
    	void printInformation()
    	{
    		// set up node directory
			std::string dirName = ("txt/nodes/" + std::to_string(this->m_idx));
			int check = mkdir(dirName.c_str(), 0777);
			// output solution
			std::string fileName1 = dirName + "/solution.txt";
			std::ofstream os1(fileName1);
			int a_idx = 0;
			for (std::vector<State*> path: this->m_solution)
			{
				os1 << "agent" << a_idx << std::endl;
				for (State* st: path)
				{
					os1 << *st << std::endl;
				}
				a_idx++;
			}
			// get solution constraints
			std::vector<Constraint> constraints;
			conflictNode *currNode = this;
			while (currNode != nullptr)
			{
				constraints.insert(constraints.begin(), currNode->m_constraint);
				currNode = currNode->parent;
			}

			// output constraints
			std::string fileName2 = dirName + "/constraints.txt";
			std::ofstream os2(fileName2);
			for (Constraint c : constraints)
			{
				if (c.getVertexConstraint() != nullptr || c.getEdgeConstraint() != nullptr)
				{
					if (c.getVertexConstraint() == nullptr)
					{
						os2 << *(c.getEdgeConstraint()) << std::endl;
					}
					else
					{
						os2 << *(c.getVertexConstraint()) << std::endl;
					}
				}
			}
    	}

    	// updates index to match that of lower planner
    	void updateIdx(const int num)
    	{
    		m_idx = num;
    	}

		Solution m_solution;  // saves the plan
		Constraint m_constraint;  // saves the constraint of a conflict Node
		int m_cost{std::numeric_limits<int>::infinity()};  // saves the cost of a conflict Node
		conflictNode *parent{nullptr};  // saves parent of a conflict Node
		std::vector<conflictNode*> children;  // saves the childeren of a conflict Nodes (only used for debugging)
		int m_idx = 0;  // index of a conflict Node
		bool m_solFlag = false;  // true if node is solution
		bool m_eval = false;  // true if node has been checked for conflicts
	};

	// save/get CompTime
	void updateCompTime(const double t) {compTime_=  t;};
	const double getCompTime() {return compTime_;};
	
	// check to make sure no explanation conflict is added twice
	bool isConflictRepeat(Conflict *curr, std::vector<Conflict*> vec);

	// Minimal Disjoint Segmentation Alg.
	int segmentSolution(Solution sol);

	// Check if to path segments are disjoint
	bool is_disjoint(const std::vector<State*> v1, 
	const std::vector<State*> v2) const;

	// main low-level graph search function
	Solution lowLevelSearch(const std::vector<State*>& startStates, 
		std::vector<Constraint*> constriants, Solution& parent, const bool useEG, const bool useHeuristic, bool &isDone);

	// evaluate a conflict node for conflicts
	std::vector<Conflict*> validateSolution(conflictNode *n);

	// get the total number of agents in the problem
	int getAgents() {return m_numAgents;};

	// get the explanation bound (r)
	int getBound() {return m_bound;};

	// priority queue -- compare two conflict nodes
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

	// get solution node
	conflictNode* getSolutionNode() {return m_solution;};

	// open min-heap (must be public)
	std::priority_queue < conflictNode*, std::vector<conflictNode*>, myconflictComparator > open_heap_;
	// closed set
	std::unordered_set <conflictNode*> closed_set_;

protected:
	Environment *m_env;  // saves the environment object
	const int m_bound;  // saves the explanation bound (r)
	int m_numAgents;  // saves the total number of agents
	conflictNode* m_solution = nullptr;  // saves the solution node
	conflictNode* m_root = nullptr;  // saves the root node
	XG_Astar_H *m_planner_H{nullptr};  // saves the XG-A^* planner w/ heuristics
	XG_Astar *m_planner{nullptr};  // saves the XG-A^* planner w/o heuristics
	A_star *m_planner_A{nullptr};  // saves the A^* planner
	double solveTime_{std::numeric_limits<double>::infinity()};  // total solving time
	double compTime_{std::numeric_limits<double>::infinity()}; //
};
