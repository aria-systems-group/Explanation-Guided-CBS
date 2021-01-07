#include <unordered_set>
#include "../includes/State.h"
#include "../includes/Location.h"
#include "../includes/Environment.h"
#include "../includes/A_star.h"


A_star::A_star(Environment& env): m_env(env) {};

bool A_star::plan(const State& startState, PlanResult<State, Action, Cost>& solution, Cost initCost)
{
	solution.states.clear();
   	solution.states.push_back(std::make_pair<>(startState, 0));
   	solution.actions.clear();
   	solution.cost = initCost;

   	// EXAMPLE SYNTAX
   	// generate a node with different 
   	// Node n1(startState, fcost, gcost);

   	// insert a state into a closed list
   	// m_closed_list.insert(n1.state);

   	// One by one extract items from min heap
   	// std::cout << m_open_heap.top() << std::endl;
    // while (m_open_heap.empty() == false)
    // {
    //     std::cout << m_open_heap.top() << std::endl;
    //     m_open_heap.pop();
    // }

    // begin algorithm
   	// Initialize node at start state -- can keep zero f-cost
   	Node startNode = Node(startState, 0, 0);

   	// init neighbors
   	std::vector<Neighbor<State, Action, Cost>> neighbors;

   	// put startState in open list -- can leave f-score at 0
   	m_open_heap.push(startNode);

   	while (!m_open_heap.empty())
   	{
   		// get lowest cost node
   		Node current = m_open_heap.top();

   		// remove current from open
   		m_open_heap.pop();

   		// generate neighbors of current node
   		m_env.expandState(current.state, neighbors);

   		// for each successor
   		for (auto& neighbor : neighbors)
        {
            // if neighbor is goal
            if (m_env.isStateGoal(neighbor.state))
            {
            	std::cout << "found solution" << std::endl;
          		return true;
            }

            // if a node same as neighbor in closed
            std::cout << m_closed_list.find(neighbor) << std::endl;



        }

   		





   		break;

   	}

	return 0;
};