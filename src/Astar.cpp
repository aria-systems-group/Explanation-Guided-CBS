#include <unordered_set>
#include <queue>
#include "../includes/State.h"
#include "../includes/Environment.h"
#include "../includes/Astar.h"

A_star::A_star(Environment *env): m_env(env) {};

bool A_star::is_disjoint(const std::vector<State*> v1, 
	const std::vector<State*> v2) const
{
    if(v1.empty() || v2.empty()) return true;

    typename std::vector<State*>::const_iterator 
        it1 = v1.begin(), 
        it1End = v1.end();

    // if(*it1 > *v2.rbegin() || *it2 > *v1.rbegin()) return true;

    while(it1 != it1End)  //  && it2 != it2End
    {
    	typename std::vector<State*>::const_iterator 
        it2 = v2.begin(), 
        it2End = v2.end();
    	while (it2 != it2End)
    	{
    		// std::cout << **it1 << std::endl;
    		// std::cout << *it2 << std::endl;
    		// std::cout << (*it1)->isSameLocation(*it2) << std::endl;
    		if((*it1)->isSameLocation(*it2))
    			return false;
    		else
    			it2++;
    	}
        it1++;
    }

    return true;
}

int A_star::SegHeuristic(Node *n, std::vector<std::vector<State*>>& otherSols)
{
	// std::string test;
	// std::cout << "Segmenting while planning for agent: " << m_env->getAgent() << std::endl;
	// given a node and other solutions, this function 
	// returns the number of segments from the start node to n->state
	// std::cout << "Segmenting Solution w.r.t. parent segments" << std::endl;

	// 1. get the path to state. 
	std::vector<State*> currPathSeg;
	Node *currCopy = n;
	while (currCopy != nullptr)
	{
		currPathSeg.insert(currPathSeg.begin(), currCopy->state);
		currCopy = currCopy->parent;
	}
	 
	// 2. find out how many steps we can go. 
	// i.e. what is this nodes state time point. 
	int longTime = 0;
	for (int a = 0; a < otherSols.size(); a++)
	{
		std::vector<State*> currSol;
		if (a == m_env->getAgent())
			currSol = currPathSeg;
		else
			currSol = otherSols[a];

		if (currSol.size() > longTime)
			longTime = currSol.size();
	}

	// 3. init a visited list for all agents and an indexing variable
	std::vector<std::vector<State*>> agentVisited(m_env->getGoals().size());
	int lastSegmentTime = 0;
	int currCost = 1;

	// 4. segment the solution
	for (int currTime = 0; currTime <= longTime; currTime++)
	{
		// 4a. add visited state for currTime
		for (int a = 0; a < otherSols.size(); a++)
		{
			if (a == m_env->getAgent())
			{
				if (currTime < currPathSeg.size())
					agentVisited[a].push_back(currPathSeg[currTime]);
			}
			else
			{
				std::vector<State*> currSol = otherSols[a];
				if (currTime < currSol.size())
					agentVisited[a].push_back(otherSols[a][currTime]);
			}
		}

		// 4b. see if agent visited is disjoint
		for (int a1 = 0; a1 < otherSols.size(); a1++)
		{
			for (int a2 = 0; a2 < otherSols.size(); a2++)
			{
				// if these are not the same agent
				if (a1 != a2)
				{
					// 4c. check disjoint
					bool disjoint = is_disjoint(agentVisited[a1], agentVisited[a2]);
					if (!disjoint)
					{
						// 4d. add cost for all states prior to currTime
						for (int a = 0; a < otherSols.size(); a++)
						{
							for (int t = lastSegmentTime; t < currTime; t++)
							{
								if (a == m_env->getAgent())
								{
									if (t < currPathSeg.size())
										currPathSeg[t]->cost = currCost;
								}
								else
								{
									std::vector<State*> currSol = otherSols[a];
									if (currTime < currSol.size())
										otherSols[a][t]->cost = currCost;
								}
							}
						}
						lastSegmentTime = currTime;

						// update cost for future
						currCost ++;

						// 4e. clear visited lists and re-init with currTime state
						for (int a = 0; a < otherSols.size(); a++)
						{
							agentVisited[a].clear();
							if (a == m_env->getAgent())
							{
								if (currTime < currPathSeg.size())
									agentVisited[a].push_back(currPathSeg[currTime]);
							}
							else
							{
								std::vector<State*> currSol = otherSols[a];
								if (currTime < currSol.size())
									agentVisited[a].push_back(otherSols[a][currTime]);
							}
						}
					}
				}
			}
		}
	}
	// std::cout << "here" << std::endl;
	// std::cout << longTime << std::endl;
	// std::cout << otherSols[0].size() << std::endl;
	// std::cout << otherSols[1].size() << std::endl;

	for (int a = 0; a < otherSols.size(); a++)
	{
		// std::cout << "Agent: " << a << std::endl;
		for (int t = lastSegmentTime; t <= longTime; t++)
		{
			if (a == m_env->getAgent())
			{
				if (t < currPathSeg.size())
				{
					currPathSeg[t]->cost = currCost;
					// std::cout << *currPathSeg[t] << std::endl;
				}
			}
			else
			{
				if (t < otherSols[a].size())
				{
					otherSols[a][t]->cost = currCost;
					// std::cout << *otherSols[a][t] << std::endl;
				}
			}
		}
	}
	
	return currCost;
}

bool A_star::plan(State *startState, std::vector<State*> &solution, 
	std::vector<Constraint*> relevantConstraints, std::vector<std::vector<State*>>& parentSol)
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
			// if (parentSol.size() != 0)
			SegHeuristic(current, parentSol);
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
		m_env->expandState(current->state, neighbors, relevantConstraints);
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
				// if a parent solution exists, lets calculate the segment cost
				// if (parentSol.size() != 0)
				SegHeuristic(n, parentSol);
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