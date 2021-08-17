#include <unordered_set>
#include <queue>
#include "../includes/State.h"
#include "../includes/Environment.h"
#include "../includes/Astar.h"

A_star::A_star(Environment *env): m_env(env) {};

bool A_star::plan(State *startState, std::vector<State*> &solution, 
	std::vector<Constraint*> relevantConstraints)
{

	// std::cout << "A* planning for agent: " << m_env->getAgent() << std::endl;
	// if (relevantConstraints.size() == 0)
	// {
	// 	std::cout << "No Constraints." << std::endl;
	// }
	// else
	// {
	// 	std::cout << "Current Constraint List: " << std::endl;
	// 	for (Constraint *c: relevantConstraints)
	// 	{
	// 		std::cout << "Agent: " << c->getVertexConstraint()->m_agent << " " <<
	// 		"State: " << c->getVertexConstraint()->m_state << std::endl;
	// 	}
	// }

	// clear all previous information
	solution.clear();

	// init open min-heap
	std::priority_queue <Node*, std::vector<Node*>, myComparator > open_heap;
	// used for seeing if a node is in the heap
	std::unordered_set<State, std::hash<State>> open_list;

	// init start node
	Node *startNode = new Node(startState, m_env->heuristicFunc(startState), 0);
	
	// std::cout << *startState << std::endl;

	open_heap.emplace(startNode);
	open_list.insert(*startState);

	// init neighbors
	std::vector<State*> neighbors;

	while (!open_heap.empty())
	{
		Node *current = open_heap.top();

		if (m_env->isStateGoal(current->state))
		{
			// std::cout << "found goal!" << std::endl;
			Node *solNode = current;

          	while (solNode != nullptr)
          	{
          	  solution.insert(solution.begin(), solNode->state);
          	  solNode = solNode->parent;
          	}
			return true;
		}

		// remove from list
		open_heap.pop();

		// get neighbors
		// with this list of constraints, we provide it to expandNode()
		// which consquentially uses it for isStateValide()
		// see Environment.h for details
		// std::cout << "entering expand" << std::endl;
		m_env->expandState(current->state, neighbors, relevantConstraints, false);
		// std::cout << "exited expand" << std::endl;


		// for all neighbors...
		for (State *st: neighbors)
		{
			// create node
			Node *n = new Node(st, m_env->heuristicFunc(startState));
			// the edge weight from current to neighbor assumed to always = 1
			int tentative_gScore = current->gScore + 1;
			if (tentative_gScore < n->gScore)
			{
				n->parent = current;
				n->gScore = tentative_gScore;
				// if neighbor not in open list
				if (open_list.find(*st) == open_list.end())
				{
					// add to heap and list
					open_heap.emplace(n);
					open_list.insert(*st);
				}

			}
			// no need for this node to take up memory
			else
				delete n;
		}
	}
	std::cout << "No Solution Found using A* using current constraints." << std::endl;
	return false;
}