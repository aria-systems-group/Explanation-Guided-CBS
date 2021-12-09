#include "../includes/S-Astar.h"


S_Astar::S_Astar(Environment *env, const bool useCBS): 
	m_env(env), useCBS{useCBS}, m_Astar{A_star(m_env)} {};


// check if two path segments are disjoint
bool S_Astar::is_disjoint(const std::vector<State*> v1, const std::vector<State*> v2) const
{
    if(v1.empty() || v2.empty()) return true;

    typename std::vector<State*>::const_iterator 
        it1 = v1.begin(), 
        it1End = v1.end();

    while(it1 != it1End)  //  && it2 != it2End
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

int S_Astar::segmentSolution(const std::vector<State*> path, const std::vector<std::vector<State*>>& otherSols)
{
	if (otherSols.size() == 0)
		return 1;

	// find out how many steps we can go. 
	// i.e. what is this nodes state time point. 
	int longTime = 0;
	for (int a = 0; a < otherSols.size(); a++)
	{
		if (a == m_env->getAgent())
		{
			// std::cout << "in here" << std::endl;
			int tmp = path.back()->time;
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
				if (t < path.size())
				{
					path[t]->cost = 0;
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
				if (currTime < path.size())
					agentVisited[a].push_back(path[currTime]);
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
									if (t < path.size())
									{
										path[t]->cost = currCost;
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
								if (currTime < path.size())
									agentVisited[a].push_back(path[currTime]);
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
				if (t < path.size())
				{
					path[t]->cost = currCost;
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

const std::vector<timedObstacle> S_Astar::plan2obs(const State* start,
	std::vector<std::vector<State*>>& existingPlan)
{
	// init empty vector
	std::vector<timedObstacle> obst;

	// proceed through segmentation alg. 
	// 1. find out how many steps we can go. 
	// i.e. what is this nodes state time point. 
	int longTime = 0;
	for (auto path: existingPlan)
	{
		if (path.size() > 0)
		{
			const int tmp = path.back()->time;
			if (tmp > longTime)
				longTime = tmp;
		}
	}

	// 2. Clear all costs 
	for (auto path: existingPlan)
	{
		for (int t = 0; t <= longTime; t++)
		{
			if (t < path.size())
			{
				path[t]->cost = 0;
			}
		}
	}

	// 3. init a visited list for all agents and an indexing variable
	std::vector<std::vector<State*>> agentVisited(m_env->getGoals().size());
	int lastSegmentTime = 0;
	int currCost = 1;

	// 4. segment the solution
	// Note: need to do this because segmentation may change when removing an agent
	for (int currTime = 0; currTime <= longTime; currTime++)
	{
		// 4a. add visited state for currTime
		for (int a = 0; a < existingPlan.size(); a++)
		{
			if (currTime < existingPlan[a].size())
				agentVisited[a].push_back(existingPlan[a][currTime]);
		}

		// 4b. see if agent visited is disjoint
		for (int a1 = 0; a1 < existingPlan.size(); a1++)
		{
			for (int a2 = 0; a2 < existingPlan.size(); a2++)
			{
				// if these are not the same agent
				if (a1 != a2)
				{
					// 4c. check disjoint
					bool disjoint = is_disjoint(agentVisited[a1], agentVisited[a2]);
					if (!disjoint)
					{
						// 4d. add cost for all states prior to currTime
						for (int a = 0; a < existingPlan.size(); a++)
						{
							for (int t = lastSegmentTime; t < currTime; t++)
							{
								if (t < existingPlan[a].size())
								{
									existingPlan[a][t]->cost = currCost;
									// add obstacle here!!
									if ((existingPlan[a][t]->x == start->x) && (existingPlan[a][t]->y == start->y))
									{
										continue;
									}
									else
									{
										timedObstacle tO(existingPlan[a][t]->x, 
											existingPlan[a][t]->y, lastSegmentTime, currTime-1);
										obst.push_back(tO);
									}
								}
							}
						}	
						
						// update cost and last segment time
						lastSegmentTime = currTime;
						currCost ++;

						// 4e. clear visited lists and re-init with currTime state
						for (int a = 0; a < existingPlan.size(); a++)
						{
							agentVisited[a].clear();
							if (currTime < existingPlan[a].size())
								agentVisited[a].push_back(existingPlan[a][currTime]);
						}
					}
				}
			}
		}
	}
	// 5. finished iterating through plan, update last segment
	for (int a = 0; a < existingPlan.size(); a++)
	{
		for (int t = lastSegmentTime; t <= longTime; t++)
		{
			if (t < existingPlan[a].size())
			{
				existingPlan[a][t]->cost = currCost;
				if ((existingPlan[a][t]->x == start->x) && (existingPlan[a][t]->y == start->y))
				{
					continue;
				}
				else
				{
					timedObstacle tO(existingPlan[a][t]->x, 
						existingPlan[a][t]->y, lastSegmentTime, longTime);
					obst.push_back(tO);
				}
			}
		}
	}
	return obst;
}


// main planning function
bool plan(State *startState, std::vector<State*> &solution, 
    const std::vector<Constraint*> relevantConstraints, 
    std::vector<std::vector<State*>>& parentSol, bool &done)
{
	// const std::vector<timedObstacle> timedObs = plan2obs(startState, parentSol);
	// m_env->addTimedObs(timedObs);

	// // plan using A*
	// bool success = m_Astar.plan(startState, solution, relevantConstraints, done);
	// m_env->clearTmpObs();
	// m_env->clearTimedObs();
	// if (success)
	// {
	// 	// segment solution
	// 	int cost = segmentSolution(solution, parentSol);
	// 	return true;
	// }
	// else
	// 	return false;
	return false;
}

