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


bool A_star::testSeg(Node *n, std::vector<std::vector<State*>> other)
{
	// find all n->parents that share cost of n
	// Node *currNode = n;
	// std::vector<Node*> currAgentSeg;
	// while (currNode != nullptr)
	// {
	// 	currAgentSeg.push_back(currNode);
	// 	currNode = currNode->parent;
	// }

	// // get all agent path segments of same time interval
	// std::vector<std::vector<State*>>  allAgentSegs(getEnv()->getGoals().size());
	// for (int a = 0; a < other.size(); a++)
	// {
	// 	std::vector<State*> agentSol = other[a];
	// 	if (other[a].size() == 0)
	// 	{
	// 		for (Node *tmpNode: currAgentSeg)
	// 		{
	// 			allAgentSegs[a].push_back(tmpNode->state);
	// 		}
	// 	}
	// 	else
	// 		for (Node *tmp: currAgentSeg)
	// 		{
	// 			int tmpTime = tmp->state->time;
	// 			if (tmpTime < other[a].size())
	// 				allAgentSegs[a].push_back(other[a][tmpTime]);
	// 		}
	// }

	// for (std::vector<State*> segment: allAgentSegs)
	// {
	// 	std::cout << segment.size() << std::endl;
	// }


	
	// exit(1);
	return 0;
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

	// Clear all costs 
	for (int a = 0; a < otherSols.size(); a++)
	{
		// std::cout << "Agent: " << a << std::endl;
		for (int t = 0; t <= longTime; t++)
		{
			if (a == m_env->getAgent())
			{
				if (t < currPathSeg.size())
				{
					currPathSeg[t]->cost = 0;
					// std::cout << *currPathSeg[t] << std::endl;
				}
			}
			else
			{
				if (t < otherSols[a].size())
				{
					otherSols[a][t]->cost = 0;
					// std::cout << *otherSols[a][t] << std::endl;
				}
			}
		}
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
				if (currTime < otherSols[a].size())
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
					// std::cout << "Checking Agents: " << a1 << " , " << a2 << std::endl;
					bool disjoint = is_disjoint(agentVisited[a1], agentVisited[a2]);
					if (!disjoint)
					{
						// std::cout << "Segmenting at : " << currTime << std::endl;
						// 4d. add cost for all states prior to currTime
						

						// for (State *st: currPathSeg)
						// {
						// 	std::cout << "Address: " << st << std::endl;
						// 	std::cout << " Values: " << *st << std::endl;
						// }


						for (int a = 0; a < otherSols.size(); a++)
						{
							for (int t = lastSegmentTime; t < currTime; t++)
							{
								if (a == m_env->getAgent())
								{
									if (t < currPathSeg.size())
									{
										// std::cout << "Changing planning agent " << std::endl;
										currPathSeg[t]->cost = currCost;
										// std::cout << "Address: " << currPathSeg[t] << std::endl;
										// std::cout << " Values: " << *currPathSeg[t] << std::endl;
									}
								}
								else
								{
									if (t < otherSols[a].size())
									{
										// std::cout << "Changing other agent " << std::endl;
										otherSols[a][t]->cost = currCost;
									}
								}
							}
							
						}
						lastSegmentTime = currTime;

						// std::cout << "Not Disjoint" << std::endl;
						// std::cout << "old cost: " << currCost << std::endl;
						// update cost for future
						currCost ++;
						// std::cout << "new cost: " << currCost << std::endl;

						// for (int a = 0; a < otherSols.size(); a++)
						// {
						// 	std::cout << "Agent: " << a << std::endl;
						// 	for (int t = 0; t <= longTime; t++)
						// 	{
						// 		if (a == m_env->getAgent())
						// 		{
						// 			if (t < currPathSeg.size())
						// 			{
						// 				std::cout << *currPathSeg[t] << std::endl;
						// 			}
						// 		}
						// 		else
						// 		{
						// 			if (t < otherSols[a].size())
						// 			{
						// 				std::cout << *otherSols[a][t] << std::endl;
						// 			}
						// 		}
						// 	}
						// }


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

	// for (int a = 0; a < otherSols.size(); a++)
	// {
	// 	std::cout << "Agent: " << a << std::endl;
	// 	for (int t = 0; t <= longTime; t++)
	// 	{
	// 		if (a == m_env->getAgent())
	// 		{
	// 			if (t < currPathSeg.size())
	// 			{
	// 				// currPathSeg[t]->cost = currCost;
	// 				std::cout << *currPathSeg[t] << std::endl;
	// 			}
	// 		}
	// 		else
	// 		{
	// 			if (t < otherSols[a].size())
	// 			{
	// 				// otherSols[a][t]->cost = currCost;
	// 				std::cout << *otherSols[a][t] << std::endl;
	// 			}
	// 		}
	// 	}
	// }

	// std::cin >> test;
	
	return currCost;
}

bool A_star::plan(State *startState, std::vector<State*> &solution, 
	std::vector<Constraint*> relevantConstraints, std::vector<std::vector<State*>>& parentSol)
{
	// clear all previous information
	solution.clear();

	// debug string
	// std::string test;

	// init open min-heap
	std::priority_queue <Node*, std::vector<Node*>, myComparator > open_heap;
	// used for seeing if a node is in the heap
	std::unordered_set<State, std::hash<State>> open_list;

	// init start node
	Node *startNode = new Node(startState, m_env->heuristicFunc(startState), 0);
	
	open_heap.emplace(startNode);
	open_list.insert(*startState);

	// init neighbors
	std::vector<State*> neighbors;

	while (!open_heap.empty())
	{
		Node *current = open_heap.top();

		if (m_env->isStateGoal(current->state))
		{
			Node *solNode = current;

			solNode->segCost = SegHeuristic(solNode, parentSol);

          	while (solNode != nullptr)
          	{
          	  solution.insert(solution.begin(), solNode->state);
          	  solNode = solNode->parent;
          	}
          	// std::cout << "here" << std::endl;
			return true;
		}

		// remove from list
		open_heap.pop();

		// get neighbors
		// with this list of constraints, we provide it to expandNode()
		// which consquentially uses it for isStateValide()
		// see Environment.h for details
		m_env->expandState(current->state, neighbors, relevantConstraints);


		// for all neighbors...
		for (State *st: neighbors)
		{
			// create node
			Node *n = new Node(st, m_env->heuristicFunc(st));
			n->parent = current;

			if (parentSol.size() != 0)
			{
				// EXP-A* loop
				// THIS WORKS BUT IS SLOWWWWW
				n->segCost = SegHeuristic(n, parentSol);

				if (n->segCost <= getBound())
				{
					// satisfiable node, continue on
					if (n->segCost < n->parent->segCost)
					{
						// blindly add node to list
						// not checking if it is a node we found before
						// not checking if in open
						open_heap.emplace(n);
						open_list.insert(*st);
					}
					else
					{
						// two cases to consider
						// 1. ==
						// 2. >
						// either way, we should check to see if 
						// it is the same node and if same state
						// only if not either of those, do we add node
						int tentative_gScore = n->parent->gScore + 1;
						if (tentative_gScore < n->gScore)
						{
							n->gScore = tentative_gScore;
							if (open_list.find(*st) == open_list.end())
							{
								open_heap.emplace(n);
								open_list.insert(*st);
							}
							else
							{
								delete n;
							}

						}
						else
						{
							delete n;
						}
					}
				}
				else
					delete n;
			}
			else
			{
				// A* loop
				int tentative_gScore = n->parent->gScore + 1;
				if (tentative_gScore < n->gScore)
				{
					n->gScore = tentative_gScore;
					if (open_list.find(*st) == open_list.end())
					{
						open_heap.emplace(n);
						open_list.insert(*st);
					}
					else
					{
						delete n;
					}
				}
				else
				{
					delete n;
				}
			}
		}
	}
	std::cout << "No Solution Found using A* using current constraints." << std::endl;
	return false;
}