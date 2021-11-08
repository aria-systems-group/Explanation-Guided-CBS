#pragma once
//  my includes
#include "../includes/Conflict.h"
#include "../includes/Environment.h"
// standard includes
#include <vector>
#include <queue>
#include <unordered_set>


// A* Planner
class A_star
{
public:
	// constructor
	A_star(Environment *env);

	// main planning function  -- returns path
	bool plan(State *startState, std::vector<State*> &solution, 
        std::vector<Constraint*> relevantConstraints, bool &done);

	// a node in planner saves relevent information in a nice object
	struct Node
	{
		// constructor
    	Node(State *state, double hScore, 
    		double gScore = std::numeric_limits<double>::infinity())
        	: state(state), gScore(gScore), hScore(hScore), parent{nullptr} {}

    	friend std::ostream& operator<<(std::ostream& os, const Node& node)
    	{
    	  os << "state: " << node.state << " fScore: " << (node.gScore + node.hScore);
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
	};

	// calculate the best next-node from open heap
	class myComparator
	{
	public:
    	int operator() (const Node *n1, const Node *n2)
    	{
    		double fScore1 = n1->gScore + n1->hScore;
    		double fScore2 = n2->gScore + n2->hScore;
        	return fScore1 > fScore2;
    	}
	};
	// get the environment object
	Environment* getEnv() {return m_env;};
private:
	Environment *m_env;  // saves the object
};