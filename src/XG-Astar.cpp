
// my includes
#include "../includes/State.h"
#include "../includes/Environment.h"
#include "../includes/XG-Astar.h"
// standard includes
#include <unordered_set>
#include <queue>
#include <chrono>


XG_Astar::XG_Astar(Environment *env, const bool useCBS): m_env(env), useCBS{useCBS} {};

bool XG_Astar::is_disjoint(const std::vector<State*> v1, 
	const std::vector<State*> v2) const
{
    if(v1.empty() || v2.empty()) return true;

    typename std::vector<State*>::const_iterator 
        it1 = v1.begin(), 
        it1End = v1.end();

    while(it1 != it1End)
    {
    	typename std::vector<State*>::const_iterator 
        it2 = v2.begin(), 
        it2End = v2.end();
    	while (it2 != it2End)
    	{
    		if((*it1)->isSameLocation(*it2))
    			return false;
    		else
    			it2++;
    	}
        it1++;
    }
    return true;
}


int XG_Astar::SegHeuristic(Node *n, std::vector<std::vector<State*>>& otherSols)
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
		int longTime = 0;
		for (int a = 0; a < otherSols.size(); a++)
		{
			if (a == m_env->getAgent())
			{
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

int XG_Astar::getLongestPath(const std::vector<std::vector<State*>>& parSol) const
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

bool XG_Astar::crossCheck(const Node *n, const int longTime) const
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
				return true;
			}
			cpy = cpy->parent;
		}
	}
	return false;
}

bool XG_Astar::plan(State *startState, std::vector<State*> &solution, 
	std::vector<Constraint*> relevantConstraints, std::vector<std::vector<State*>>& parentSol, bool &done)
{
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

	if (m_env->isStateValid(startState, startState, relevantConstraints))
	{
		open_heap.emplace(startNode);
		open_list.insert(*startState);
	}
	
	// init neighbors
	std::vector<State*> neighbors;
	int total  = 1;
	int time = 1;
	int missing = 0;
	while (!open_heap.empty() && !done)
	{
		if (done)
			std::cout << "updated done" << std::endl;
		Node *current = open_heap.top();
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
          		solution.insert(solution.begin(), solNode->state);
          		solNode = solNode->parent;
          	}
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
			// clear all data
			open_heap.pop();
          	open_list = std::unordered_set<State, std::hash<State>>();
          	// clear the data
          	while (!open_heap.empty())
          	{
          		Node* n = open_heap.top();
          		delete n->state;
          		open_heap.pop();
          		delete n;
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
			n->segCost = SegHeuristic(n, parentSol);  // updates all state costs
			
			// we have updated segmentation information 
			// plan as normal 
			if (!crossCheck(n, longTime)) 
			// either longTime not reached
			// or there was no cross with iteself
			{
			// if new segment not created
			// greedily add node
				if (n->segCost <= n->parent->segCost)
				{
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
						n->gScore = tentative_gScore;
						open_heap.emplace(n);
						open_list.insert(*st);
						total++;
					}
				}
			}
		}
	}
	// clear the data
	open_list = std::unordered_set<State, std::hash<State>>();
    while (!open_heap.empty())
    {
    	Node* n = open_heap.top();
    	delete n->state;
    	open_heap.pop();
    	delete n;
    }
	return false;
}