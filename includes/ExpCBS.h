#pragma once
#include "Environment.h"
#include "../includes/Astar.h"
#include "../includes/Conflict.h"
#include <chrono>
#include <unordered_set>
#include <iostream>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <sstream> //for std::stringstream 


typedef std::vector<std::vector<State*>> Solution;

// EXP - CBS
class ExpCBS
{
public:
	ExpCBS(Environment *env, const int bound);

	bool plan(const std::vector<State*>& startStates, Solution& solution, bool verbose = false);

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
			int cost = 0;
			for (std::vector<State*> sol: m_solution)
			{
				if (cost < sol.back()->cost)
					cost = sol.back()->cost;
			}
			return cost;
		};

		void print(std::ostream& os, int level, const conflictNode *curr, std::vector<Conflict*> cnf) 
		{

    		for (int i = 1; i < level; i++)
    		{
    	   		os << "\t\t ";
    		}

    		// include address in tree
   			// const void * address = static_cast<const void*>(this);
			// std::stringstream ss;
			// ss << address;  
			// std::string name = ss.str();

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
						// std::cout << *(c.getEdgeConstraint()) << std::endl;
						// exit(1);
						os2 << *(c.getEdgeConstraint()) << std::endl;
					}
					else
					{
						// std::cout << *(c.getVertexConstraint()) << std::endl;
					// exit(1);
						os2 << *(c.getVertexConstraint()) << std::endl;
					}
				}
			}
    	}



    	void updateIdx(const int num)
    	{
    		m_idx = num;
    	}

		Solution m_solution;
		Constraint m_constraint;
		int m_cost{std::numeric_limits<int>::infinity()};
		conflictNode *parent{nullptr};
		std::vector<conflictNode*> children;
		int m_idx = 0;
		bool m_solFlag = false;
		bool m_eval = false;
		// bool m_isWaiting = true;
	};

	void showTree(const conflictNode *curr, std::vector<Conflict*> cnf);

	bool isConflictRepeat(Conflict *curr, std::vector<Conflict*> vec);

	Solution lowLevelSearch(const std::vector<State*>& startStates, 
		std::vector<Constraint*> constriants, Solution& parent);

	std::vector<Conflict*> validateSolution(conflictNode *n);

	int getAgents() {return m_numAgents;};

	int getBound() {return m_bound;};

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
	conflictNode* m_root = nullptr;
	A_star *m_planner{nullptr};
};
