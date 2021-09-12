
#pragma once
// my includes
#include "../includes/Conflict.h"
// standard includes
#include <vector>
#include <queue>
#include <unordered_set>


// XG - A* w/o heuristics
class EG_Astar
{
public:
    // constructor
	EG_Astar(Environment *env, const bool useCBS = true);

	bool plan(State *startState, std::vector<State*> &solution, 
        std::vector<Constraint*> relevantConstraints, 
        std::vector<std::vector<State*>>& parent);

	// a node in planner saves relevent information in a nice object
    struct Node
	{
        // constructor
    	Node(State *state, double hScore = std::numeric_limits<double>::infinity(), 
    		double gScore = std::numeric_limits<double>::infinity())
        	: state(state), gScore(gScore), hScore(hScore), parent{nullptr} {}

    	// print nicely
        friend std::ostream& operator<<(std::ostream& os, const Node& node)
    	{
    	  os << "state: " << *node.state << " fScore: " << (node.gScore + node.hScore);
    	  return os;
    	}

        // current state
    	State *state;
        // parent (to go back to s_i)
    	Node *parent;
        // gScore = is the cost of the path from the start node
    	double gScore;
        // hScore = heuristic cost function (Equclidean dist from node to goal)
    	double hScore;
        // segment cost
        int segCost{1};
        // boolean that enables certain transitions
        bool isWaiting = true;
	};
    // get longest path of existing plan
    int getLongestPath(const std::vector<std::vector<State*>>& parSol) const;
    // check for self loops and paths greater than B
    bool crossCheck(const Node *n, const int longTime) const;
    // Calc. index of a path segment from s_i to n
    int SegHeuristic(Node *n, std::vector<std::vector<State*>>& otherSols);
    // check if two path segments are disjoint
    bool is_disjoint(const std::vector<State*> v1, 
        const std::vector<State*> v2) const;

	// calculate the best next-node from open heap
	class myComparator
	{
	public:
    	int operator() (const Node *n1, const Node *n2)
    	{
    		if (n1->segCost == n2->segCost)
            {
                // which is smaller f-cost
                double fScore1 = n1->gScore + n1->hScore;
                double fScore2 = n2->gScore + n2->hScore;
                return fScore1 > fScore2;
            }
            else
            {
                // which index is smaller
                return n1->segCost > n2->segCost;
            }  
    	}
	};
    // grab environment object
	Environment* getEnv() {return m_env;};
private:
	Environment *m_env;  // saves the environment object
    const bool useCBS;  // saves the answer of using CBS
};