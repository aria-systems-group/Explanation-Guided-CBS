// my includes
#include "../includes/Cbs.h"
// standard includes
#include <chrono>


CBS::CBS(Environment *env): m_env(env)
{
	m_planner = new A_star(m_env);
	m_numAgents = m_env->getGoals().size();
	Constraint m_constraint{Constraint()};
};

Solution CBS::lowLevelSearch(const std::vector<State*>& startStates, 
		std::vector<Constraint*> constraints, Solution& parent, bool &isDone)
{
	// we have a list of constraints for all agents from current node
	// to root node. 

	// low level planner should only be provided with the constraints
	// it needs to worry about.

	Solution sol;
	std::vector<State*> singleSol;
	bool cont = true;

	for (int a = 0; a < startStates.size(); a++) // (const State &st : startStates)
	{	
		// need to provide A* with constraints relevant to 
		// the agent. 
		// for each agent, cycle through c
		std::vector<Constraint*> agentRelevantCs;
		for (Constraint *c: constraints)
		{
			if (c->getVertexConstraint() != nullptr)
			{
				if (c->getVertexConstraint()->agent == a)
					agentRelevantCs.push_back(c);
			}
			if (c->getEdgeConstraint() != nullptr)
			{
				if (c->getEdgeConstraint()->agent == a)
					agentRelevantCs.push_back(c);
			}
		}
		if (cont)
		{
			
			if (parent.size() == 0)
			{
				bool success = m_planner->plan(startStates[a], singleSol, agentRelevantCs, isDone);
				if (success)
					sol.push_back(singleSol);
				else
					cont = false;
			}
			// if the most current constraint in the list of constraints is not 
			// for agent a, then the solution will not change
			// do this for both vertex of edge constraints
			else
			{
				VertexConstraint *currVC = constraints[0]->getVertexConstraint();
				EdgeConstraint *currEC = constraints[0]->getEdgeConstraint();
				// std::cout << currVC << std::endl;
				// std::cout << currEC << std::endl;
				if (currVC != nullptr)
				{
					// if the newest vertex constraint not with agent a
					if (currVC->agent != a)
					{
						// new sol = parent sol
						sol.push_back(parent[a]);
					}
					// newest vertex constraint is relevant to agent a
					// need to replan
					else
					{
						bool success = m_planner->plan(startStates[a], singleSol, agentRelevantCs, isDone);
						if (success)
							sol.push_back(singleSol);
						else
							cont = false;
					}
				}
				else if (currEC != nullptr)
				{
					// if the newest edge constraint not with agent a
					if (currEC->agent != a)
					{
						// new sol = parent sol
						sol.push_back(parent[a]);
					}
					// newest edge constraint is relevant to agent a
					// need to replan
					else
					{
						bool success = m_planner->plan(startStates[a], singleSol, agentRelevantCs, isDone);
						if (success)
							sol.push_back(singleSol);
						else
							cont = false;
					}
				}
			}
		}
		m_planner->getEnv()->updateAgent();
	}
	// wrap environment idx back to first agent
	m_planner->getEnv()->updateAgent();
	return sol;
}

// this is the computational bottle neck
Conflict* CBS::validateSolution(conflictNode *n)
{
	// this function will step through all solution and look for conflicts
	// if conflict is found, then we add it to the node
	Conflict *c = nullptr;

	std::string test;

	// check for vertex constriants
	// note that these loops can be quicker by not double checking 
	// i.e. a1 vs a2 and v2 vs a1
	for (int a1 = 0; a1 < getAgents(); a1++)
	{
		for (int a2 = 0; a2 < getAgents(); a2++)
		{
			// if these are not the same agent
			if (a1 != a2)
			{
				int t1 = n->m_solution[a1].size();
				int t2 = n->m_solution[a2].size();

				// if statement to check the shortest path
				if (t1 <= t2)
				{
					// iterate through t1 looking for vertex constraints
					for (int t = 0; t < t1; t++)
					{
						// fist, vertex conflicts
						// get both states at same time step
						State *a1Curr = n->m_solution[a1][t];
						State *a2Curr = n->m_solution[a2][t];

						if ((a1Curr->x == a2Curr->x) && (a1Curr->y == a2Curr->y) && (a1Curr->time == a2Curr->time))
						{
							// a1 and a2 are the indices of the agents
							// Conflict *c = new Conflict(a1, a2, st1);
							c = new Conflict();
							c->type = Conflict::Vertex;
							c->time1 = a1Curr->time;
							c->agent1 = a1; c->agent2 = a2;
							c->x1 = a1Curr->x; c->y1 = a1Curr->y;
							return c;
						}
						if ((t + 1) < t1)
						{
							// check for edge constraint with next time step
							State *a1Nxt = n->m_solution[a1][t + 1];
							State *a2Nxt = n->m_solution[a2][t + 1];

							// check to see if two agents flip
							if ((a1Curr->x == a2Nxt->x) && (a1Curr->y == a2Nxt->y) && (a2Curr->x == a1Nxt->x) && (a2Curr->y == a1Nxt->y) )
							{
								c = new Conflict();
								c->type = Conflict::Edge;
								c->time1 = a1Curr->time;
								c->time2 = a1Nxt->time;
								c->agent1 = a1;
								c->agent2 = a2;
								c->x1 = a1Curr->x ; c->y1 = a1Curr->y;
								c->x2 = a1Nxt->x ; c->y2 = a1Nxt->y;
								return c;
							}
						}
					}
				}
				else
				{
					// iterate through t2 looking for constraints
					for (int t = 0; t < t2; t++)
					{
						State *a1Curr = n->m_solution[a1][t];
						State *a2Curr = n->m_solution[a2][t];

						if ((a1Curr->x == a2Curr->x) && (a1Curr->y == a2Curr->y) && (a1Curr->time == a2Curr->time))
						{
							c = new Conflict();
							c->type = Conflict::Vertex;
							c->time1 = a1Curr->time;
							c->agent1 = a1; c->agent2 = a2;
							c->x1 = a1Curr->x ; c->y1 = a1Curr->y;
							return c;
						}
						if ((t + 1) < t2)
						{
							State *a1Nxt = n->m_solution[a1][t + 1];
							State *a2Nxt = n->m_solution[a2][t + 1];
							if ((a1Curr->x == a2Nxt->x) && (a1Curr->y == a2Nxt->y) && (a2Curr->x == a1Nxt->x) && (a2Curr->y == a1Nxt->y) )
							{
								c = new Conflict();
								c->type = Conflict::Edge;
								c->time1 = a1Curr->time;
								c->time2 = a1Nxt->time;
								c->agent1 = a1;
								c->agent2 = a2;
								c->x1 = a1Curr->x ; c->y1 = a1Curr->y;
								c->x2 = a1Nxt->x ; c->y2 = a1Nxt->y;
								return c;
							}
						}
					}
				}
			}
		}
	}
	return c;
}

bool CBS::is_disjoint(const std::vector<State*> v1, 
	const std::vector<State*> v2) const
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

int CBS::segmentSolution(Solution sol)
{
	int longTime = 0;
	for (int a = 0; a < sol.size(); a++)
	{
		int tmp = sol[a].back()->time;
		if (tmp > longTime)
			longTime = tmp;
	}
	// 3. init a visited list for all agents and an indexing variable
	std::vector<std::vector<State*>> agentVisited(m_env->getGoals().size());
	int lastSegmentTime = 0;
	int currCost = 1;

	for (int currTime = 0; currTime <= longTime; currTime++)
	{
		// add visited state for currTime
		for (int a = 0; a < sol.size(); a++)
			if (sol[a].back()->time >= currTime)
				agentVisited[a].push_back(sol[a][currTime]);

		// 4b. see if agent visited is disjoint
		for (int a1 = 0; a1 < sol.size(); a1++)
		{
			for (int a2 = 0; a2 < sol.size(); a2++)
			{
				// if these are not the same agent
				if (a1 != a2)
				{
					// 4c. check disjoint
					bool disjoint = is_disjoint(agentVisited[a1], agentVisited[a2]);
					if (!disjoint)
					{
						// 4d. add cost for all states prior to currTime
						for (int a = 0; a < sol.size(); a++)
						{
							for (int t = lastSegmentTime; t < currTime; t++)
							{
								if (sol[a].back()->time >= t)
									sol[a][t]->cost = currCost;
							}
						}	
						lastSegmentTime = currTime;

						// update cost for future
						currCost ++;

						// 4e. clear visited lists and re-init with currTime state
						for (int a = 0; a < sol.size(); a++)
						{
							agentVisited[a].clear();
							if (sol[a].back()->time >= currTime)
								agentVisited[a].push_back(sol[a][currTime]);
						}
					}
				}
			}
		}
	}
	for (int a = 0; a < sol.size(); a++)
	{
		for (int t = lastSegmentTime; t <= longTime; t++)
		{
			if (sol[a].back()->time >= t)
				sol[a][t]->cost = currCost;
		}
	}
	return currCost;
}

bool CBS::plan(const std::vector<State*>& startStates, Solution& solution)
{
	std::cout << "Planning with CBS... " << std::endl;
	bool isOverTime = false;
	bool isSolved = false;
	const auto start = std::chrono::high_resolution_clock::now();

	std::thread timeThread (Timer(), start, solveTime_, std::ref(isOverTime), std::ref(isSolved));

	solution.clear();

	// init open min-heap
	std::priority_queue <conflictNode*, std::vector<conflictNode*>, myconflictComparator > open_heap;
	
	// find solution with no constraints
	std::vector<Constraint*> constriants;
	Solution par;
	Solution rootSol = lowLevelSearch(startStates, constriants, par, isOverTime);

	conflictNode *rootNode;
	
	if (rootSol.size() != m_numAgents)
		return false;
	else
		rootNode = new conflictNode(rootSol);

	open_heap.emplace(rootNode);
	int tree_sz = 1;

	while (!open_heap.empty() && !isOverTime)
	{

		// get best node
		conflictNode *current = open_heap.top();

		// validate for conflicts
		Conflict *c = validateSolution(current);

		if (c == nullptr)
		{
			int expCost = segmentSolution(current->m_solution);
  			auto stop = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  			std::cout << "Successful planning using CBS" << std::endl;
  			std::cout << "Duration: " << duration.count() << " micro seconds" << " or approx. " << (duration.count() / 1000000.0) << " seconds" << std::endl;
  			std::cout << "Size of Conflict Tree: " << tree_sz << std::endl;
  			std::cout << "Solution Explanation Cost: " << expCost << std::endl;
  			solution = current->m_solution;
			isSolved = true;
			timeThread.join();
			return true;
		}
		else
		{
			// remove from list
			open_heap.pop();
			// for each agent in conflict (currently only two at a time)
			for (int a = 0; a < 2; a++)
			{
				if (c->type == Conflict::Vertex)
				{
					// as per paper, new node is initialized with parent solution
					conflictNode *n = new conflictNode(current->m_solution);
					// new node branches from current
					n->parent = current;
					// create constraint
					if (a == 0)
					{
						VertexConstraint *v = new VertexConstraint(c->agent1, c->time1, c->x1, c->y1);
						n->m_constraint.add(v);
					}
					else
					{
						VertexConstraint *v = new VertexConstraint(c->agent2, c->time1, c->x1, c->y1);
						n->m_constraint.add(v);
					}
					// add constraint to node
					// get vector of entire constraints
					
					std::vector<Constraint*> constriants;
					conflictNode *currNode = n;
					while (currNode != nullptr)
					{
						constriants.push_back(&(currNode->m_constraint));
						currNode = currNode->parent;
					}

					n->m_solution = lowLevelSearch(startStates, constriants, 
						n->parent->m_solution, isOverTime);
					// update cost of solution if it is a valid one
					if (n->m_solution.size() == m_numAgents)
					{
						n->m_cost = n->calcCost();
						open_heap.emplace(n);
						tree_sz ++;
					}
				}
				if (c->type == Conflict::Edge)
				{
					// as per paper, new node is initialized with parent solution
					conflictNode *n = new conflictNode(current->m_solution);
					// new node branches from current
					n->parent = current;
					// create constraint -- changes based on which agent we are talking about

					if (a == 0)
					{
						EdgeConstraint *e = new EdgeConstraint(c->agent1, c->time1, c->time2, c->x1, c->y1, c->x2, c->y2);
						n->m_constraint.add(e);
					}
					else
					{
						EdgeConstraint *e = new EdgeConstraint(c->agent2, c->time1, c->time2, c->x2, c->y2, c->x1, c->y1);
						n->m_constraint.add(e);
					}
					
					
					std::vector<Constraint*> constriants;
					conflictNode *currNode = n;
					while (currNode != nullptr)
					{
						constriants.push_back(&(currNode->m_constraint));
						currNode = currNode->parent;
					}

					n->m_solution = lowLevelSearch(startStates, constriants, 
						n->parent->m_solution, isOverTime);
					// update cost of solution if it is a valid one
					if (n->m_solution.size() == m_numAgents)
					{
						n->m_cost = n->calcCost();
						open_heap.emplace(n);
						tree_sz ++;
					}
				}
			}
		}
	}
	std::cout << "No solution" << std::endl;
	timeThread.join();
	return false;
}
