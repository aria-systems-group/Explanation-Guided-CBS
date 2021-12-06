
#pragma once
#include <vector>
#include <queue>
#include <unordered_set>
#include <thread>
#include "../includes/Timer.h"
#include "../includes/Conflict.h"
#include "../includes/Astar.h"


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
        myComparator(const double p_exp, Environment *env, 
            const std::vector<Constraint*> cosntraints, 
            const std::vector<timedObstacle> timedObs,
            const std::vector<std::vector<State*>> existingSol,
            bool timerBool): 
        m_p_exp(p_exp), 
        m_env(env), 
        m_cosntraints(cosntraints), 
        m_Astar(A_star(m_env)),
        m_isOverTime(timerBool),
        m_existingSol(existingSol),
        m_timedObs(timedObs) {}

        int segmentation(const std::vector<State*> path);

        bool is_disjoint(const std::vector<State*> v1, 
            const std::vector<State*> v2) const;
    	
        int operator() (const Node *n1, const Node *n2)
    	{
            // CURRENT
            // double fScore1 = ( (1 - m_p_exp) * (n1->gScore + n1->hScore) )
            //      + ((m_p_exp) * (n1->segCost));
            // double fScore2 = ( (1 - m_p_exp) * (n2->gScore + n2->hScore) )
            //      + ((m_p_exp) * (n2->segCost));
            // if (fScore1 == fScore2)
                // return n1->segCost > n2->segCost;
            // else
            //     return fScore1 > fScore2;

            // EXPERIMENTAL
            
            /**********************************/
            // // get G score == seg cost
            const int gScore1 = n1->segCost;
            const int gScore2 = n2->segCost;

            // get H score
            // plan using A* that satisfies time-dependent obs.
            // provide time dependent obstacles to environment

            // // init params to be filled
            bool isAstarOverTime1 = false;
            bool isAstarSolved1 = false;
            bool isAstarOverTime2 = false;
            bool isAstarSolved2 = false;


            // // 1. get the path from current state back to root. 
            // std::vector<State*> path1;
            // std::vector<State*> path2;
            // const Node *copy1 = n1;
            // const Node *copy2 = n2;
            // while (copy1 != nullptr)
            // {
            //     path1.insert(path1.begin(), copy1->state);
            //     copy1 = copy1->parent;
            // }
            // while (copy2 != nullptr)
            // {
            //     path2.insert(path2.begin(), copy2->state);
            //     copy2 = copy2->parent;
            // }

            std::vector<State*> solution1;
            std::vector<State*> solution2;

            // // provide timed obs to env
            // m_env->addTimedObs(m_timedObs);

            // // std::thread timeThread (Timer(), start, 0.5, 
            // //     std::ref(isAstarOverTime), std::ref(isAstarSolved));
            // // printf("HERE \n");
            // bool success1 = m_Astar.plan(n1->state, solution1, 
            //     m_cosntraints, m_isOverTime);

            // bool success2 = m_Astar.plan(n2->state, solution2, 
            //     m_cosntraints, m_isOverTime);

            // if (solution1.size() == 0 || solution2.size() == 0)
            // {
            //     printf("More undefined behaviour.");
            //     exit(1);
            // }

            // // done planning, remove them
            // m_env->clearTimedObs();

            // path1.insert(path1.end(), solution1.begin(), solution1.end());
            // path2.insert(path2.end(), solution2.begin(), solution2.end());

            // const int hScore1 = segmentation(path1);
            // const int hScore2 = segmentation(path2);

            /**********************************/

            int hScore1 = 1;
            int hScore2 = 1;

            // fill tmp obs with new existing solution
            for (auto path: m_existingSol)
            {
                // printf("Path Size: %lu \n", path.size());
                for (auto *st: path)
                {
                    // std::cout << *st << std::endl;
                    // do not add tmp obs if st == startState
                    // turn state into location
                    Location *loc = new Location(st->x, st->y);
                    m_env->addTmpObs(loc);
                }
            }
            auto start1 = std::chrono::high_resolution_clock::now();
            std::thread timeThread1 (Timer(), start1, 0.1, std::ref(isAstarOverTime1), std::ref(isAstarSolved1));
            bool success1 = m_Astar.plan(n1->state, solution1, m_cosntraints, isAstarOverTime1);
            
            auto start2 = std::chrono::high_resolution_clock::now();
            std::thread timeThread2 (Timer(), start2, 0.1, std::ref(isAstarOverTime2), std::ref(isAstarSolved2));
            bool success2 = m_Astar.plan(n2->state, solution2, m_cosntraints, isAstarOverTime2);

            m_env->clearTmpObs();

            timeThread1.join();
            timeThread2.join();

            if (success1)
                hScore1 = 0;

            if (success2)
                hScore2 = 0;


            // combine costs together
            const int fScore1 = gScore1 + hScore1;
            const int fScore2 = gScore2 + hScore2;

            // if (fScore1 != fScore2)
            return fScore1 > fScore2;
            // else
            //     return solution1.size() > solution2.size();
        }
    private:
        const double m_p_exp;
        Environment *m_env;
        A_star m_Astar;
        const std::vector<Constraint*> m_cosntraints;
        const std::vector<timedObstacle> m_timedObs;
        const std::vector<std::vector<State*>> m_existingSol; 
        bool m_isOverTime;
	};
    // get the environment object
	Environment* getEnv() {return m_env;};
private:
	Environment *m_env;  // saves the object
    const bool useCBS;  // saves the answer of using CBS
    const double m_perc_exp;
    A_star m_Astar;  // A* for heuristic planning
};