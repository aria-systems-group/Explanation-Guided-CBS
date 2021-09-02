
#include <unordered_set>
#include <queue>
#include "../includes/State.h"
#include "../includes/Environment.h"
#include "../includes/EG-Astar-H.h"
#include <chrono>

EG_Astar_H::EG_Astar_H(Environment *env, const bool useCBS): m_env(env), useCBS{useCBS} {};

bool EG_Astar_H::is_disjoint(const std::vector<State*> v1, 
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


int EG_Astar_H::SegHeuristic_combinedAgent(Node *n, std::vector<std::vector<State*>>& otherSols)
{
	if (otherSols.size() == 0)
		return 1;
	// given a node and other solutions, this function 
	// returns the number of segments from the start node to n->state

	// 1. get the path from current state back to root. 
	std::vector<State*> currPathSeg;
	Node *currCopy = n;
	while (currCopy != nullptr)
	{
		currPathSeg.insert(currPathSeg.begin(), currCopy->state);
		currCopy = currCopy->parent;
	}

	if (useCBS)
	{
		// 2. find out how many steps we can go. 
		// i.e. what is this nodes state time point. 
		// int longTime = currPathSeg.back()->time;
		// std::cout << "here" << std::endl;
		int longTime = 0;
		for (int a = 0; a < otherSols.size(); a++)
		{
			if (a == m_env->getAgent())
			{
				// std::cout << "in here" << std::endl;
				int tmp = currPathSeg.back()->time;
				if (tmp > longTime)
					longTime = tmp;
			}
			else
			{
				if (otherSols[a].size() > 0)
				{
					int tmp = otherSols[a].back()->time;
					if (tmp > longTime)
						longTime = tmp;
				}
			}
		}
		// Clear all costs 
		for (int a = 0; a < otherSols.size(); a++)
		{
			for (int t = 0; t <= longTime; t++)
			{
				if (a == m_env->getAgent())
				{
					if (t < currPathSeg.size())
					{
						currPathSeg[t]->cost = 0;
					}
				}
				else
				{
					if (t < otherSols[a].size())
					{
						otherSols[a][t]->cost = 0;
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
					if ((a1 != a2) && (a1 == m_env->getAgent() || a2 == m_env->getAgent()))
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
										{
											currPathSeg[t]->cost = currCost;
										}
									}
									else
									{
										if (t < otherSols[a].size())
										{
											otherSols[a][t]->cost = currCost;
										}
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
									if (currTime < otherSols[a].size())
										agentVisited[a].push_back(otherSols[a][currTime]);
								}
							}
						}
					}	
				}
			}
		}
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
					}
				}
				else
				{
					if (t < otherSols[a].size())
					{
						otherSols[a][t]->cost = currCost;
					}
				}
			}
		}
		return currCost;
	}
	else
	{
		std::cout << "ERROR: incorrect heuristic used in A*. Aborting..." << std::endl; 
		exit(1);
		return 0;
	}

}

int EG_Astar_H::SegHeuristic(Node *n, std::vector<std::vector<State*>>& otherSols)
{
	if (otherSols.size() == 0)
		return 1;
	// given a node and other solutions, this function 
	// returns the number of segments from the start node to n->state

	// 1. get the path from current state back to root. 
	std::vector<State*> currPathSeg;
	Node *currCopy = n;
	while (currCopy != nullptr)
	{
		currPathSeg.insert(currPathSeg.begin(), currCopy->state);
		currCopy = currCopy->parent;
	}

	if (useCBS)
	{
		// 2. find out how many steps we can go. 
		// i.e. what is this nodes state time point. 
		// int longTime = currPathSeg.back()->time;
		// std::cout << "here" << std::endl;
		int longTime = 0;
		for (int a = 0; a < otherSols.size(); a++)
		{
			if (a == m_env->getAgent())
			{
				// std::cout << "in here" << std::endl;
				int tmp = currPathSeg.back()->time;
				if (tmp > longTime)
					longTime = tmp;
			}
			else
			{
				if (otherSols[a].size() > 0)
				{
					int tmp = otherSols[a].back()->time;
					if (tmp > longTime)
						longTime = tmp;
				}
			}
		}

		// Clear all costs 
		for (int a = 0; a < otherSols.size(); a++)
		{
			for (int t = 0; t <= longTime; t++)
			{
				if (a == m_env->getAgent())
				{
					if (t < currPathSeg.size())
					{
						currPathSeg[t]->cost = 0;
					}
				}
				else
				{
					if (t < otherSols[a].size())
					{
						otherSols[a][t]->cost = 0;
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
										{
										currPathSeg[t]->cost = currCost;
										}
									}
									else
									{
										if (t < otherSols[a].size())
										{
											otherSols[a][t]->cost = currCost;
										}
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
									if (currTime < otherSols[a].size())
										agentVisited[a].push_back(otherSols[a][currTime]);
								}
							}
						}
					}
				}
			}
		}
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
					}
				}
				else
				{
					if (t < otherSols[a].size())
					{
						otherSols[a][t]->cost = currCost;
					}
				}
			}
		}
		return currCost;
	}
	else
	{
		// 2. find out how many steps we can go. 
		// i.e. what is this nodes state time point. 
		// int longTime = currPathSeg.back()->time;
		// std::cout << "here" << std::endl;
		int longTime = 0;
		for (int a = 0; a <= otherSols.size(); a++)
		{
			if (a == m_env->getAgent())
			{
				// std::cout << "in here" << std::endl;
				int tmp = currPathSeg.back()->time;
				if (tmp > longTime)
					longTime = tmp;
			}
			else
			{
				if (otherSols[a].size() > 0)
				{
					int tmp = otherSols[a].back()->time;
					if (tmp > longTime)
						longTime = tmp;
				}
			}
		}

		// Clear all costs 
		for (int a = 0; a <= otherSols.size(); a++)
		{
			for (int t = 0; t <= longTime; t++)
			{
				if (a == m_env->getAgent())
				{
					if (t < currPathSeg.size())
					{
						currPathSeg[t]->cost = 0;
					}
				}
				else
				{
					if (t < otherSols[a].size())
					{
						otherSols[a][t]->cost = 0;
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
			for (int a = 0; a <= otherSols.size(); a++)
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
			for (int a1 = 0; a1 <= otherSols.size(); a1++)
			{
				for (int a2 = 0; a2 <= otherSols.size(); a2++)
				{
					// if these are not the same agent
					if (a1 != a2)
					{
						// 4c. check disjoint
						bool disjoint = is_disjoint(agentVisited[a1], agentVisited[a2]);
						if (!disjoint)
						{
							// 4d. add cost for all states prior to currTime
	
							for (int a = 0; a <= otherSols.size(); a++)
							{
								for (int t = lastSegmentTime; t < currTime; t++)
								{
									if (a == m_env->getAgent())
									{
										if (t < currPathSeg.size())
										{
										currPathSeg[t]->cost = currCost;
										}
									}
									else
									{
										if (t < otherSols[a].size())
										{
											otherSols[a][t]->cost = currCost;
										}
									}
								}
							}	
						
							lastSegmentTime = currTime;

							// update cost for future
							currCost ++;

							// 4e. clear visited lists and re-init with currTime state
							for (int a = 0; a <= otherSols.size(); a++)
							{
								agentVisited[a].clear();
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
						}
					}
				}
			}
		}
		for (int a = 0; a <= otherSols.size(); a++)
		{
			// std::cout << "Agent: " << a << std::endl;
			for (int t = lastSegmentTime; t <= longTime; t++)
			{
				if (a == m_env->getAgent())
				{
					if (t < currPathSeg.size())
					{
						currPathSeg[t]->cost = currCost;
					}
				}
				else
				{
					if (t < otherSols[a].size())
					{
						otherSols[a][t]->cost = currCost;
					}
				}
			}
		}
		return currCost;
	}
}

int EG_Astar_H::getLongestPath(const std::vector<std::vector<State*>>& parSol) const
{
	int longTime = 0;
	for (std::vector<State*> sol: parSol)
	{
		// get last time
		if (sol.size() > 0)
		{
			int currTime = sol.back()->time;
			if (currTime > longTime)
				longTime = currTime;
		}
	}
	return longTime;
}

bool EG_Astar_H::crossCheck(const Node *n, const int longTime) const
{
	std::string test;
	//see if currState is same location as any other on path back to root
	State *currState = n->state;
	Node *cpy = n->parent;
	// can do whatever we want before longTime is hit
	// after longTime, currState cannot reach previous state in loop
	if (currState->time < longTime || longTime == 0)
		return false;
	else if (currState->time > (m_env->getXdim() * m_env->getYdim()))
		return true;
	else
	{
		while (cpy != nullptr)
		{
			if (currState->isSameLocation((cpy->state)))
			{
				// if (m_env->getAgent() == 1)
				// {
				// 	std::cout << "returning cross: " << std::endl;
				// 	std::cin >> test;
				// }
				return true;
			}
			cpy = cpy->parent;
		}
		// if (m_env->getAgent() == 1)
		// {
		// 	std::cout << "returning NO cross: " << std::endl;
		// 	std::cin >> test;
		// }
	}
	return false;
}

// void ExpA_star::canonicalPath(Node *curr)
// {
// 	// input: path with possibly repeated switching
// 	// return: path without switching and waits at beginning of each segment
// 	Node *currCopy = curr;
// 	std::string test;
// 	int currCost = curr->segCost;
// 	std::vector<State*> currSeg;
// 	while (currCopy != nullptr)
// 	{
// 		if (currCopy->segCost == currCost)
// 		{
// 			currSeg.push_back(currCopy->state);
// 			// check for revisitation
// 			State *init = currSeg[0];
// 			for (int i=0; i < currSeg.size(); i++)
// 			{
// 				if (i == 0)
// 					continue;
// 				else
// 				{
// 					// check for same location 
// 					if (init->isSameLocation(currSeg[i]))
// 					{
// 						// make all states up to currSeg[i] == init
// 						for (int j = 0; j < currSeg[i]; j++)
// 						{
// 							currSeg[j]->x = init->x;
// 							currSeg[j]->y = init->y;
// 						}
// 					}
// 				}
// 		}
// 		else
// 		{
// 			currSeg.clear();
// 			currSeg.push_back(currCopy->state);
// 			currCost = currCost - 1;
// 		}
// 		currCopy = currCopy->parent;
// 	}
// }

bool EG_Astar_H::checkPathValidity(Node *curr)
{
	// given a new node (and consequentially a path)
	// make sure it fits into canonical form
	std::string test;

	// make 2 copies of path
	Node *currCopy = curr;
	Node *cpy = curr;

	// make an empty segment -- make this vector of vectors
	std::vector<Node*> currSeg;

	// get current segment
	if (cpy->parent == nullptr)
		return true;
	
	while (cpy->parent != nullptr && cpy->segCost == curr->segCost)  // state->cost
	{
		cpy = cpy->parent;
		currSeg.push_back(cpy);
		
	}
	
	// for (Node *n: currSeg)
		// std::cout << *n << " " << n->isWaiting << std::endl;


	// while not at root node
	// std::cout << "entering loop" << std::endl;
	// while (currCopy->parent != nullptr)
	// {
		
	// std::cout << "here: " << *currCopy << " " << currCopy->isWaiting << std::endl; 
	// std::cin >> test;
	// std::cout << "now here: " << std::endl;
	if (currCopy->segCost == currCopy->parent->segCost)
	{
		// std::cout << "in this loop " << std::endl;
		// currCopy and parent the part of same segment
		if (currCopy->parent->isWaiting && (currCopy->state)->isSameLocation(currCopy->parent->state))
		{
			// waiting -- return true
			// std::cout << "in here: " << *(currCopy->parent) << std::endl;
			// std::cout << "return true here" << std::endl;
			// std::cin >> test;
			return true;
		}
		else
		{
			// std::cout << "i am here now" << std::endl;
			// two cases possible
			// n->parent was not waiting OR currCopy was not same location as parent
			// either way, need to make sure that currCopy did not revisit any state
			for (Node *n: currSeg)
			{
				if ((n->state)->isSameLocation(currCopy->state))
				{
					// n is not waiting but has revisited a state
					// std::cout << "returning false here" << std::endl;
					// std::cin >> test;
					return false;
				}
			}
			// std::cout << "checked segment without fail" << std::endl; 
		}
		// std::cout << "adding..." << std::endl;
		// currSeg.push_back(currCopy);
	}
	else
	{
		// no need for this
		// std::cout << "return true here" << std::endl;
		// std::cin >> test;
		return true;
		// no restrictions on actions
		// clear current segment and add currCopy to new segment
		// currSeg.clear();
		// currSeg.push_back(currCopy);
	}
		// std::cout << "i am here now" << std::endl;
		// currCopy = currCopy->parent;
		// std::cout << "out of logic -- going to parent" << std::endl;
		// std::cout << *currCopy << std::endl;
		// std::cout << "printing segment" << std::endl;
	
		
		// std::cin >> test;
	// }

	// std::cin >> test;
	// check initial state with current segment
	// for (Node *n: currSeg)
	// {
	// 	if ((n->state)->isSameLocation(currCopy->state))
	// 	{
	// 		// n is not waiting but has revisited a state
	// 		std::cout << "return false" << std::endl;
	// 		std::cin >> test;
	// 		return false;
	// 	}
	// }
	// std::cout << "exiting true" << std::endl;
	// std::cin >> test;
	return true;
}

int EG_Astar_H::getMaxCost(std::vector<std::vector<State*>> existSol)
{
	int max_seg = 0;
	int missing  = 0;

	if (existSol.size() == 0)
		return std::numeric_limits<int>::infinity();
	else
	{
		// check for a full existing solution
		for (std::vector<State*> a: existSol)
		{
			if (a.size() == 0)
			{
				missing++;
				if (missing > 1)
					return std::numeric_limits<int>::infinity();
			}
			else
			{
				if (a.back()->cost > max_seg)
					max_seg = a.back()->cost;
			}
		}
	}
	return max_seg;
}

int EG_Astar_H::noIntersectCheck(Node* curr, std::vector<std::vector<State*>> existingSol)
{
	// path is on the latest segment
	// any current intersection is a segment

	// get current segment for all agents.
	std::vector<std::vector<State*>>  currentSegment(m_env->getAgentNames().size());
	// std::cout << "here" << std::endl;
	for (int a = 0; a < existingSol.size(); a++) 
	{
		// std::cout << existingSol[a].size() << std::endl;
		for (State* st: existingSol[a])
		{
			// std::cout << *st << std::endl;
			// std::cout << "entering check" << std::endl;
			if (st->cost == curr->parent->segCost)
			{
				// std::cout << "checking" <<std::endl;
				currentSegment[a].push_back(st);
			}
				
			// std::cout << "done check" << std::endl;
		}
	}
	// std::cout << "made it here" << std::endl;
	// check current state against current segment
	for (std::vector<State*> a: currentSegment)
	{
		for (State* st: a)
		{
			if (st->isSameLocation(curr->state))
				return (curr->parent->segCost) + 1;
		}
	}
	// std::cout << "now here" << std::endl;
	return (curr->parent->segCost);
}

bool EG_Astar_H::plan(State *startState, std::vector<State*> &solution, 
	std::vector<Constraint*> relevantConstraints, std::vector<std::vector<State*>>& parentSol)
{
	std::string test;
	int maxCost = getMaxCost(parentSol);
	auto start = std::chrono::high_resolution_clock::now();
	// clear all previous information
	solution.clear();

	if (useCBS == false)
	{
		std::cout << "planning without CBS... using collision checking" << std::endl;
		m_env->includeCollisionChecks(parentSol);
	}
	
	const int longTime = getLongestPath(parentSol);

	// init open min-heap
	std::priority_queue <Node*, std::vector<Node*>, myComparator > open_heap;
	// used for seeing if a node is in the heap
	std::unordered_set<State, std::hash<State>> open_list;

	// init start node
	Node *startNode = new Node(startState, m_env->heuristicFunc(startState), 0);

	if (m_env->isStateValid(startState, startState, relevantConstraints) && checkPathValidity(startNode))
	{
		open_heap.emplace(startNode);
		open_list.insert(*startState);
	}
	
	// init neighbors
	std::vector<State*> neighbors;
	int total  = 1;
	int time = 1;
	int missing = 0;
	while (!open_heap.empty())
	{
		Node *current = open_heap.top();
		// std::cout << *(current->state) << std::endl;
		auto currStop = std::chrono::high_resolution_clock::now();
		auto currDuration = std::chrono::duration_cast<std::chrono::microseconds>(currStop - start);
		if ((currDuration.count() / 1000000.0) > (100 * time))
		{
			missing = 0;
			std::cout << "Planning in A* for greater than " << 
				(100 * time) << " seconds." << std::endl;
			time++;
			Node *currCopy = current;
			while (currCopy != nullptr)
			{
				std::cout << *currCopy << std::endl;
				currCopy = currCopy->parent;
			}
			// check for a full existing solution
			for (std::vector<State*> a: parentSol)
			{
				if (a.size() == 0)
				{
					missing++;
					
				}
			}
			if (missing > 1)
				std::cout << "not a full parent" << std::endl;
			else
				std::cout << "full parent" << std::endl;
		}
		
		if (m_env->isStateGoal(current->state))
		{
			Node *solNode = current;

			solNode->segCost = SegHeuristic(solNode, parentSol);
			int expCost = solNode->segCost;

          	while (solNode != nullptr)
          	{
          		// std::cout << *solNode->state << " " << solNode->isWaiting << std::endl;
          		solution.insert(solution.begin(), solNode->state);
          		solNode = solNode->parent;
          	}
          	// std::cin >> test;
          	// std::cout << "found goal for agent: " << m_env->getAgent() << std::endl;
			auto stop = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
			if (useCBS == false)
			{
				parentSol.push_back(solution);
				std::cout << "Found Solution using EG-A*-H" << std::endl;
				std::cout << "Solution Explanation cost: " << expCost << std::endl;
				std::cout << "Duration: " << duration.count() << " microseconds" << " or approx. " << 
  					(duration.count() / 1000000.0) << " seconds" << std::endl;

			}
			// notify user that solution was found after 100 seconds
			if (time > 1)
				std::cout << "Found Solution" << std::endl;
			return true;
		}

		// remove from list
		open_heap.pop();

		// get neighbors
		// with this list of constraints, we provide it to expandNode()
		// which consquentially uses it for isStateValide()
		// see Environment.h for details
		m_env->expandState(current->state, neighbors, relevantConstraints, current->isWaiting);

		// for all neighbors...
		for (State *st: neighbors)
		{
			// create node
			Node *n = new Node(st, m_env->heuristicFunc(st));
			n->parent = current;

			// // testing section

			if (n->parent->state->cost == maxCost)
			{
				// do some different seg check
				n->state->cost = noIntersectCheck(n, parentSol);
				n->segCost = n->state->cost;
			}
			else if (n->parent->state->cost > maxCost)
			{
				n->state->cost = n->parent->state->cost;
				n->segCost = n->parent->segCost;
			}
			else
			{
				n->segCost = SegHeuristic(n, parentSol);  // updates all state costs
				// n->segCost = SegHeuristic_combinedAgent(n, parentSol);
			}
			

			// we have updated segmentation information 
			// now time to update node waiting status
			if (n->parent->state->cost == n->state->cost)
			{
				// part of same segment ... 
				if ((n->parent->isWaiting) && ((n->state)->isSameLocation(n->parent->state)))
				{
					// took "stay" action n is waiting
					n->isWaiting = true;
					// n->parent->isWaiting = true;				
				}
				else
				{
					// did not "stay" n is not waiting
					n->isWaiting = false;
					
					
					// std::cin >> test;
					// n->parent->isWaiting = true;
				}
			}
			else
			{
				// new segment was created n isWaiting
				n->isWaiting = true;
				// if (n->parent->state->time == 0 && n->parent->state->x == 0 && n->parent->state->y == 4)
				// {
				// 	if (n->state->time == 1 && n->state->x == 1 && n->state->y == 4)
				// 	{
				// 		// std::cout << "here" << std::endl;

 			// 			std::cout << *(n->state) << " " << n->isWaiting << " " << n->segCost << std::endl;
				// 		std::cout << *(n->parent->state) << " " << n->parent->isWaiting << " " << n->parent->segCost << std::endl;				
				// 		std::cout << (n->state)->isSameLocation(n->parent->state) << std::endl;
				// 		std::cout << n->parent->isWaiting << std::endl;
				// 		// std::cin >> test;
				// 	}
				// }
				// n->parent->isWaiting = true;
			}



			// Now, check to see if node fits into connonical form
			// delete if current is not waiting but n is same location as current
			// delete if path is not in canonical form
			if (!checkPathValidity(n))
			{
				delete n;
			}
			// if (false)
				// std::cout << "ERROR" << std::endl;
			else
			{
			
				// plan as normal 
				// if (true)
				if (!crossCheck(n, longTime)) 
				// either longTime not reached
				// or there was no cross with iteself
			
				{
				// if new segment not created
				// greedily add node
					if (n->segCost <= n->parent->segCost)
					{
						// if (!(current->state->isSameLocation(n->state)))
						// 	std::cout << "adding node" << std::endl;
						n->gScore = n->parent->gScore + 1;
						open_heap.emplace(n);
						open_list.insert(*st);
						total++;
					}
					else
					{
						// new segment needed, proceed as normal
						int tentative_gScore = n->parent->gScore + 1;
						if (tentative_gScore < n->gScore)
						{
							// if (!(current->state->isSameLocation(n->state)))
							// 	std::cout << "adding node" << std::endl;
							n->gScore = tentative_gScore;
							open_heap.emplace(n);
							open_list.insert(*st);
							total++;
						}
					}
				}
			}
		}
	}
	// std::cout << "No Solution Found using A* using current constraints." << std::endl;
	// std::cout << "total nodes Explored by A*: " << total << std::endl;
	return false;
}