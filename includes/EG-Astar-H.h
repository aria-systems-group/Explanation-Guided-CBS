
#pragma once
#include <vector>
#include <queue>
#include <unordered_set>
#include "../includes/Conflict.h"

// EG - A* with heuristics
class EG_Astar_H
{
public:
	EG_Astar_H(Environment *env, const bool useCBS = true);

	bool plan(State *startState, std::vector<State*> &solution, 
        std::vector<Constraint*> relevantConstraints, 
        std::vector<std::vector<State*>>& parent);

	struct Node
	{
    	Node(State *state, double hScore = std::numeric_limits<double>::infinity(), 
    		double gScore = std::numeric_limits<double>::infinity())
        	: state(state), gScore(gScore), hScore(hScore), parent{nullptr} {}

    	friend std::ostream& operator<<(std::ostream& os, const Node& node)
    	{
    	  os << "state: " << *node.state << " fScore: " << (node.gScore + node.hScore);
    	  return os;
    	}

    	// gScore = is the cost of the path from the start node
    	// hScore = heuristic cost function (Equclidean dist from node to goal)
    	State *state;
    	Node *parent;
    	double gScore;
    	double hScore;
        int segCost{1};
        bool isWaiting = true;
	};

    int noIntersectCheck(Node* curr, std::vector<std::vector<State*>> existingSol);

    int getMaxCost(std::vector<std::vector<State*>> existSol);

    int getLongestPath(const std::vector<std::vector<State*>>& parSol) const;

    bool crossCheck(const Node *n, const int longTime) const;

    int SegHeuristic(Node *n, std::vector<std::vector<State*>>& otherSols);

    int SegHeuristic_combinedAgent(Node *n, std::vector<std::vector<State*>>& otherSols);


    bool is_disjoint(const std::vector<State*> v1, 
        const std::vector<State*> v2) const;

    bool checkPathValidity(Node *curr);

	// To compare two Nodes -- f(n) = g(n) + h(n)
	// calculate the best next-node from open heap
	class myComparator
	{
	public:
    	int operator() (const Node *n1, const Node *n2)
    	{
    		if (n1->segCost == n2->segCost)
            {
                // which is smaller cost
                double fScore1 = n1->gScore + n1->hScore;
                double fScore2 = n2->gScore + n2->hScore;
                // std::cout << fScore1 << "vs. " << fScore2 << std::endl;
                return fScore1 > fScore2;
                // return n1->hScore > n2->hScore;
                // return n1->gScore > n2->gScore;
            }
            else
            {
                // which segments are smaller
                return n1->segCost > n2->segCost;
            }  
    	}
	};

	Environment* getEnv() {return m_env;};
private:
	Environment *m_env;
    const bool useCBS;
};