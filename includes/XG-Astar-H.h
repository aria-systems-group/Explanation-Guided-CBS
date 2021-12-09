#pragma once
#include <vector>
#include <queue>
#include <unordered_set>
#include <thread>
#include "../includes/Timer.h"
#include "../includes/Conflict.h"
// #include "../includes/Astar.h"


// XG - A* with heuristics
class XG_Astar_H
{
public:
    // constructor
	XG_Astar_H(Environment *env, const double percent_exp, const bool useCBS = true);

	// main planning function
    bool plan(State *startState, std::vector<State*> &solution, 
        const std::vector<Constraint*> relevantConstraints, 
        std::vector<std::vector<State*>>& parent, bool &done);

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
    // quick check for intersection of existing solution
    int noIntersectCheck(Node* curr, std::vector<std::vector<State*>> existingSol);
    // get index of existing plan
    int getMaxCost(std::vector<std::vector<State*>> existSol);
    // get longest path of existing plan
    int getLongestPath(const std::vector<std::vector<State*>>& parSol) const;
    // check for self loops and paths greater than B
    bool crossCheck(const Node *n, const int longTime) const;
    // Calc. index of a path segment from s_i to n
    int SegHeuristic(Node *n, std::vector<std::vector<State*>>& otherSols, const int max);
    // check if two path segments are disjoint
    bool is_disjoint(const std::vector<State*> v1, 
        const std::vector<State*> v2) const;
    // check if path is in normal form
    bool checkPathValidity(Node *curr);

    const double getPercExp() const {return m_perc_exp;};

    int segmentSolution(const std::vector<State*> path, const std::vector<std::vector<State*>>& otherSols);

    const std::vector<timedObstacle> plan2obs(const State* start, std::vector<std::vector<State*>>& existingPlan);

	// calculate the best next-node from open heap
	class myComparator
	{
	public:
        myComparator(const double p_exp): 
        m_p_exp(p_exp) {}
    	
        int operator() (const Node *n1, const Node *n2)
    	{
            if (m_p_exp < 1.0)
            {
                // we use weighting scheme for A* and XG-A*
                double fScore1 = ( (1 - m_p_exp) * (n1->gScore + n1->hScore) )
                 + ((m_p_exp) * (n1->segCost));
                double fScore2 = ( (1 - m_p_exp) * (n2->gScore + n2->hScore) )
                     + ((m_p_exp) * (n2->segCost));
                // when weighting, use which has lower segment as tie breaker
                if (fScore1 == fScore2)
                    return n1->segCost > n2->segCost;
                else
                    return fScore1 > fScore2;
            }
            else
            {
                // pure XG-A*
                // tie breaker is A* F-score
                if (n1->segCost == n2->segCost)
                {
                    // which is smaller cost
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
        }
    private:
        const double m_p_exp;
	};
    // get the environment object
	Environment* getEnv() {return m_env;};
private:
	Environment *m_env;  // saves the object
    const bool useCBS;  // saves the answer of using CBS
    const double m_perc_exp;
    // A_star m_Astar;  // A* for heuristic planning
};