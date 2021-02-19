#include "../includes/Cbs.h"
#include <chrono>


// struct mycomparer {
//     bool operator()(string const& lhs, pair<string const, SomeCustomId> const& rhs) {
//         return lhs < rhs.first;
//     }
//     bool operator()(pair<string const, SomeCustomId> const& lhs, string const& rhs) {
//         return lhs.first < rhs;
//     }
// };

CBS::CBS(Environment *env, const int bound): m_env(env), m_bound{bound}
{
	m_planner = new A_star(m_env);
	m_planner->updateBound(m_bound);
	m_numAgents = m_env->getGoals().size();
	Constraint m_constraint{Constraint()};
};


bool CBS::conflictNode::is_disjoint(const std::vector<State*> v1, 
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


int CBS::conflictNode::segmentSolution()
{
	// this function segments a solution within a node
	// for (std::vector<State> sol: n->m_solution)
	// {
	// 	for (State st: sol)
	// 	{
	// 		std::cout << st << std::endl;
	// 	}
	// }

	// 1. find longest solution
	int longTime = 0;
	for (std::vector<State*> Asol: m_solution)
	{
		if (Asol.size() > longTime)
			longTime = Asol.size();
	}
	// std::cout << longTime << "\n";

	// 2. init visited list and a segment indexing variable
	std::vector<std::vector<State*>> agentVisited(m_solution.size());
	int lastSegmentTime = 0;
	int currCost = 1;

	// 3. segment the solution
	for (int currTime = 0; currTime <= longTime; currTime++)
	{
		// 3a. add visited state for currTime
		for (int a = 0; a < m_solution.size(); a++)
		{
			std::vector<State*> currSol = m_solution[a];
			if (currTime < currSol.size())
				agentVisited[a].push_back(currSol[currTime]);
		}

		// 3b. check if disjoint

		for (int a1 = 0; a1 < m_solution.size(); a1++)
		{
			for (int a2 = 0; a2 < m_solution.size(); a2++)
			{
				// if these are not the same agent
				if (a1 != a2)
				{
					// 3c check disjoint
					bool disjoint = is_disjoint(agentVisited[a1], agentVisited[a2]);
					if (!disjoint)
					{
						// 3d. add cost for all states prior to currTime
						// while (lastSegmentTime < (currTime - 1))
						// {
						// std::cout << "I am here" << std::endl;
						for (int a = 0; a < m_solution.size(); a++)
						{
							for (int t = lastSegmentTime; t <= (currTime - 1); t++)
							{
								// exit(1);
								if (t < m_solution[a].size())
									m_solution[a][t]->cost = currCost;
							}
						}
						lastSegmentTime = currTime;
						// std::cout << "now here" << std::endl;

						// update cost for future
						currCost ++;


						// 3e. clear visited lists and re-init with currTime state
						// std::cout << "last loop" << std::endl;
						for (int a = 0; a < m_solution.size(); a++)
						{
							agentVisited[a].clear();
							std::vector<State*> currSol = m_solution[a];
							if (currTime < currSol.size())
								// m_solution[a][currTime]->cost = currCost;
								agentVisited[a].push_back(currSol[currTime]);
						}
						// std::cout << "success segment" << std::endl;
					}
				}
			}
		}
	}
	// std::cout << "out of loop" << std::endl;
	for (int a = 0; a < m_solution.size(); a++)
	{
		for (int t = lastSegmentTime; t < longTime; t++)
		{
			if (t < m_solution[a].size())
			{
				m_solution[a][t]->cost = currCost;
			}
		}
	}
	return currCost;
	// std::cout << "done with function" << std::endl;

}


Solution CBS::lowLevelSearch(const std::vector<State*>& startStates, 
		std::vector<Constraint*> constraints, Solution& parent)
{
	// we have a list of constraints for all agents from current node
	// to root node. 

	// low level planner should only be provided with the constraints
	// it needs to worry about.

	while (m_planner->getEnv()->getAgent() != 0)
	{
		m_planner->getEnv()->updateAgent();
	}

	// std::cout << "In CBS finding MA solution" << std::endl;
	Solution sol(getAgents());
	std::vector<State*> singleSol;
	std::vector<Constraint*> agentRelevantCs;
	bool cont = true;

	if (parent.size() == 0)
	{
		// at root node, plan for all agents
		for (int a = 0; a < startStates.size(); a++)
		{
			if (cont)
			{
				bool success = m_planner->plan(startStates[a], singleSol, 
					agentRelevantCs, parent);
				if (success)
					sol[a] = singleSol;
				else
					cont = false;
			}
			m_planner->getEnv()->updateAgent();
		}
	}
	else
	{
		// not at root node, only replan for a single agent
		VertexConstraint *currVC = constraints[0]->getVertexConstraint();
		EdgeConstraint *currEC = constraints[0]->getEdgeConstraint();
		// if we added a vertex constaint, replan for that agent
		if (currVC != nullptr)
		{
			// get the most recent solutions
			for (int a = 0; a < startStates.size(); a++)
			{
				// new sol = parent sol
				if (currVC->agent != a)
				{
					for (State* st: parent[a])
					{
						State *st_new = new State(st);
						sol[a].push_back(st_new);
					}
				}
			}

			// get all relevant constriants
			for (Constraint *c: constraints)
			{
				if (c->getVertexConstraint() != nullptr)
				{
					if (c->getVertexConstraint()->agent == currVC->agent)
						agentRelevantCs.push_back(c);
				}
				if (c->getEdgeConstraint() != nullptr)
				{
					if (c->getEdgeConstraint()->agent == currVC->agent)
						agentRelevantCs.push_back(c);
				}
			}

			// update agent to the correct one
			while (m_planner->getEnv()->getAgent() != currVC->agent)
			{
				m_planner->getEnv()->updateAgent();
			}

			bool success = m_planner->plan(startStates[currVC->agent], singleSol, 
				agentRelevantCs, sol);
			if (success)
			{
				sol[currVC->agent] = singleSol;
			}
			else
				cont = false;
			m_planner->getEnv()->updateAgent();
		}
		else if (currEC != nullptr)
		{
			// get the most recent solutions
			for (int a = 0; a < startStates.size(); a++)
			{
				// new sol = parent sol
				if (currEC->agent != a)
				{
					for (State* st: parent[a])
					{
						State *st_new = new State(st);
						sol[a].push_back(st_new);
					}
				}
			}

			// get all relevant constriants
			std::vector<Constraint*> agentRelevantCs;
			for (Constraint *c: constraints)
			{
				if (c->getVertexConstraint() != nullptr)
				{
					if (c->getVertexConstraint()->agent == currEC->agent)
						agentRelevantCs.push_back(c);
				}
				if (c->getEdgeConstraint() != nullptr)
				{
					if (c->getEdgeConstraint()->agent == currEC->agent)
						agentRelevantCs.push_back(c);
				}
			}

			// update agent to the correct one
			while (m_planner->getEnv()->getAgent() != currEC->agent)
			{
				m_planner->getEnv()->updateAgent();
			}

			bool success = m_planner->plan(startStates[currEC->agent], singleSol, 
				agentRelevantCs, sol);
			if (success)
			{
				// sol[currEC->agent].clear();
				sol[currEC->agent] = singleSol;
			}
			else
				cont = false;

			m_planner->getEnv()->updateAgent();
		}
	}

	



	// for (int a = 0; a < startStates.size(); a++) // (const State &st : startStates)
	// {	
	// 	// need to provide A* with constraints relevant to 
	// 	// the agent. 
	// 	// for each agent, cycle through c
	// 	std::vector<Constraint*> agentRelevantCs;
	// 	for (Constraint *c: constraints)
	// 	{
	// 		if (c->getVertexConstraint() != nullptr)
	// 		{
	// 			if (c->getVertexConstraint()->agent == a)
	// 				agentRelevantCs.push_back(c);
	// 		}
	// 		if (c->getEdgeConstraint() != nullptr)
	// 		{
	// 			if (c->getEdgeConstraint()->agent == a)
	// 				agentRelevantCs.push_back(c);
	// 		}
	// 	}
	// 	// std::cout << "entering planner" << std::endl;
	// 	// std::cout << "Agent: " << a << ", Start: " << startStates[a] << ", Goal: " << m_planner->getEnv()->getGoals()[a] << std::endl;
	// 	if (cont)
	// 	{
			
	// 		if (parent.size() == 0)
	// 		{
	// 			bool success = m_planner->plan(startStates[a], singleSol, 
	// 				agentRelevantCs, parent);
	// 			if (success)
	// 				sol.push_back(singleSol);
	// 			else
	// 				cont = false;
	// 		}
	// 		// if the most current constraint in the list of constraints is not 
	// 		// for agent a, then the solution will not change
	// 		// do this for both vertex of edge constraints
	// 		else
	// 		{
	// 			VertexConstraint *currVC = constraints[0]->getVertexConstraint();
	// 			EdgeConstraint *currEC = constraints[0]->getEdgeConstraint();
	// 			// std::cout << currVC << std::endl;
	// 			// std::cout << currEC << std::endl;
	// 			if (currVC != nullptr)
	// 			{
	// 				// if the newest vertex constraint not with agent a
	// 				if (currVC->agent != a)
	// 				{
	// 					// new sol = parent sol
	// 					sol.push_back(parent[a]);
	// 				}
	// 				// newest vertex constraint is relevant to agent a
	// 				// need to replan
	// 				else
	// 				{
	// 					bool success = m_planner->plan(startStates[a], singleSol, agentRelevantCs, parent);
	// 					if (success)
	// 						sol.push_back(singleSol);
	// 					else
	// 						cont = false;
	// 				}
	// 			}
	// 			else if (currEC != nullptr)
	// 			{
	// 				// if the newest edge constraint not with agent a
	// 				if (currEC->agent != a)
	// 				{
	// 					// new sol = parent sol
	// 					sol.push_back(parent[a]);
	// 				}
	// 				// newest edge constraint is relevant to agent a
	// 				// need to replan
	// 				else
	// 				{
	// 					bool success = m_planner->plan(startStates[a], singleSol, 
	// 						agentRelevantCs, parent);
	// 					if (success)
	// 						sol.push_back(singleSol);
	// 					else
	// 						cont = false;
	// 				}
	// 			}
	// 		}
	// 	}
		
	// 	// std::cout << "exited planner" << std::endl;
	// 	// if sol found, add it --- else, return no solution
		
	// 	m_planner->getEnv()->updateAgent();
	// }
	// // wrap environment idx back to first agent
	// m_planner->getEnv()->updateAgent();
	// update agent to the correct one
	while (m_planner->getEnv()->getAgent() != 0)
	{
		m_planner->getEnv()->updateAgent();
	}


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
				// first, get time both solutions
				// std::cout << "getting times" << std::endl;
				int t1 = n->m_solution[a1].size();
				int t2 = n->m_solution[a2].size();
				// std::cout << "got times" << std::endl;
				// std::cout << "t1: " << t1 << " t2: " << t2 << std::endl;

				// if statement to check the shortest path
				if (t1 <= t2)
				{
					// iterate through t1 looking for vertex constraints
					for (int t = 0; t < t1; t++)
					{
						// fist, vertex conflicts
						// get both states at same time step

						// std::cout << "I am here now" << std::endl;
						State *a1Curr = n->m_solution[a1][t];
						State *a2Curr = n->m_solution[a2][t];

						// std::cout << "now here" << std::endl;


						if ((a1Curr->x == a2Curr->x) && (a1Curr->y == a2Curr->y) && (a1Curr->time == a2Curr->time))
						{
							// std::cout << "found v conflict" << std::endl;
							// std::cout << st2 << std::endl;
							// a1 and a2 are the indices of the agents
							// Conflict *c = new Conflict(a1, a2, st1);
							c = new Conflict();
							c->type = Conflict::Vertex;
							c->time1 = a1Curr->time;
							c->agent1 = a1; c->agent2 = a2;
							c->x1 = a1Curr->x; c->y1 = a1Curr->y;

							// if (a1 == 1 && a2 == 7)
							// {
							// 	std::cout << "conflict here" << std::endl;
							// 	if (a1Curr->time == 3 && a2Curr->time == 3)
							// 	{
							// 		std::cout << "In Conflict" << std::endl;
							// 		std::cout << a1Curr <<std::endl;
							// 		std::cout << a2Curr <<std::endl;
							// 		std::cin >> test;
							// 	}
							// }
							// std::cout << "returning true" << std::endl;
							return c;
						}

						// if (a1 == 1 && a2 == 7)
						// {
						// 	std::cout << "here" << std::endl;
						// 	if (a1Curr->time == 3 && a2Curr->time == 3)
						// 	{
						// 		std::cout << "No Conflict" << std::endl;
						// 		std::cout << a1Curr <<std::endl;
						// 		std::cout << a2Curr <<std::endl;
						// 		std::cin >> test;
						// 	}
						// }

						if ((t + 1) < t1)
						{
							// check for edge constraint with next time step
							State *a1Nxt = n->m_solution[a1][t + 1];
							State *a2Nxt = n->m_solution[a2][t + 1];

							// check to see if two agents flip
							if ((a1Curr->x == a2Nxt->x) && (a1Curr->y == a2Nxt->y) && (a2Curr->x == a1Nxt->x) && (a2Curr->y == a1Nxt->y) )
							{
								// std::cout << "found e conflict" << std::endl;
								// std::cout << "Found Edge Conflict: " << std::endl;
								// std::cout << st1 << std::endl;
								// std::cout << st3 << std::endl;
								c = new Conflict();
								c->type = Conflict::Edge;
								c->time1 = a1Curr->time;
								c->time2 = a1Nxt->time;
								c->agent1 = a1;
								c->agent2 = a2;
								c->x1 = a1Curr->x ; c->y1 = a1Curr->y;
								c->x2 = a1Nxt->x ; c->y2 = a1Nxt->y;
								// std::cout << "returning true" << std::endl;
								// std::cout << c->agent1 << " " << c->agent2 << std::endl;
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
						// fist, vertex conflicts
						// std::cout << "in loop time: " << t << std::endl;
						// get both states at same time step
						// std::cout << "down here" << std::endl;
						State *a1Curr = n->m_solution[a1][t];
						State *a2Curr = n->m_solution[a2][t];
						// std::cout << "now down here" << std::endl;

						if ((a1Curr->x == a2Curr->x) && (a1Curr->y == a2Curr->y) && (a1Curr->time == a2Curr->time))
						{
							// std::cout << "found v conflict" << std::endl;
							// std::cout << st2 << std::endl;
							// a1 and a2 are the indices of the agents
							// Conflict *c = new Conflict(a1, a2, st1);
							c = new Conflict();
							c->type = Conflict::Vertex;
							c->time1 = a1Curr->time;
							c->agent1 = a1; c->agent2 = a2;
							c->x1 = a1Curr->x ; c->y1 = a1Curr->y;
							// if (a1 == 1 && a2 == 7)
							// {
							// 	std::cout << "conflict here" << std::endl;
							// 	if (a1Curr->time == 3 && a2Curr->time == 3)
							// 	{
							// 		std::cout << "In Conflict" << std::endl;
							// 		std::cout << a1Curr <<std::endl;
							// 		std::cout << a2Curr <<std::endl;
							// 		std::cin >> test;
							// 	}
							// }
							// std::cout << "returning" << std::endl;
							return c;
						}

						// if (a1 == 1 && a2 == 7)
						// {
						// 	std::cout << "here" << std::endl;
						// 	if (a1Curr->time == 3 && a2Curr->time == 3)
						// 	{
						// 		std::cout << "No Conflict" << std::endl;
						// 		std::cout << a1Curr <<std::endl;
						// 		std::cout << a2Curr <<std::endl;
						// 		std::cin >> test;
						// 	}
						// }
						// check for edge constraint with next time step
						// std::cout << "here" << std::endl;
						if ((t + 1) < t2)
						{
							State *a1Nxt = n->m_solution[a1][t + 1];
							State *a2Nxt = n->m_solution[a2][t + 1];
							// std::cout << *a1Nxt << std::endl;
							// std::cout << *a2Nxt << std::endl;
						

							// check to see if two agents flip
							if ((a1Curr->x == a2Nxt->x) && (a1Curr->y == a2Nxt->y) && (a2Curr->x == a1Nxt->x) && (a2Curr->y == a1Nxt->y) )
							{
								// std::cout << "Found Edge Conflict: " << std::endl;
								// std::cout << st1 << std::endl;
								// std::cout << st3 << std::endl;
								c = new Conflict();
								c->type = Conflict::Edge;
								c->time1 = a1Curr->time;
								c->time2 = a1Nxt->time;
								c->agent1 = a1;
								c->agent2 = a2;
								c->x1 = a1Curr->x ; c->y1 = a1Curr->y;
								c->x2 = a1Nxt->x ; c->y2 = a1Nxt->y;
								// std::cout << c->agent1 << " " << c->agent2 << std::endl;
								// std::cout << "Current" << std::endl;
								// std::cout << "Agent: " << c->agent1 << " " << a1Curr << " " << " Agent: " << c->agent2 << " " << a2Curr << std::endl;
								// std::cout << "Next" << std::endl;
								// std::cout << "Agent: " << c->agent1 << " " << a1Nxt << " " << " Agent: " << c->agent2 << " " << a2Nxt << std::endl;
								// std::cout << "returning" << std::endl;
								return c;
							}
							// std::cout << "changing " << std::endl;
						}
					}
				}
			}
		}
	}
	return c;
}

bool CBS::plan(const std::vector<State*>& startStates, Solution& solution)
{
	std::cout << "Now Planning with CBS" << std::endl;
	int timeAstar = 0;
	auto start = std::chrono::high_resolution_clock::now();

	solution.clear();

	// init open min-heap
	std::priority_queue <conflictNode*, std::vector<conflictNode*>, myconflictComparator > open_heap;
	// used for seeing if a node is in the heap
	// std::unordered_set<State, std::hash<State>> m_open_list;
	
	// find solution with no constraints
	std::vector<Constraint*> constriants;
	// std::cout << "searching for init sol" << std::endl;
	Solution par;
	// std::cout << "Init low lowLevelSearch" << std::endl;
	auto t1 = std::chrono::high_resolution_clock::now();
	Solution rootSol = lowLevelSearch(startStates, constriants, par);
	auto t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
	timeAstar = timeAstar + duration.count();

	// std::cout << "done" << std::endl;
	// exit(1);

	// std::cout << "Initial Solution: " << std::endl;
	// for (int a = 0; a < rootSol.size(); a++)
	// {
	// 	std::cout << "Agent: " << a << std::endl;
	// 	for (State *st: rootSol[a])
	// 		std::cout << *st << std::endl;
	// }
	std::string test;

	conflictNode *rootNode;
	
	if (rootSol.size() != m_numAgents)
		return false;
	else
	{
		rootNode = new conflictNode(rootSol);
		open_heap.emplace(rootNode);
	}

	while (!open_heap.empty())
	{

		// get best node
		conflictNode *current = open_heap.top();

		// validate it for conflicts
		// std::cout << "Searching Conflicts " <<  std::endl;

		Conflict *c = validateSolution(current);

		// std::cout << std::endl;
		// std::cout << "Conflict Found: " << c << std::endl;
		// exit(1);
		// std::cin >> test;

		if (c == nullptr)
		{
			// if no conflicts occur, then we found a solution
			auto stop = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  			std::cout << "Duration: " << duration.count() << " microseconds" << " or approx. " << 
  				(duration.count() / 1000000.0) << " seconds" << std::endl;
  			
  			std::cout << "Time Spent in A*: " << timeAstar << " microseconds" << 
  				" or approx. " << (timeAstar / 1000000.0) << " seconds" << std::endl;

  			// int solCost = segmentSolution(current->m_solution);
  			// std::cout << "in CBS solution" << std::endl;
  			// for (int a = 0; a < getAgents(); a++)
  			// {
  			// 	for (State* st: current->m_solution[a])
  			// 	{
  			// 		std::cout << *st << std::endl;
  			// 	}
  			// }

  			// exit(1);
  			// std::cout << current << std::endl;
			// std::cin >>test;
  			int solCost = current->getSegCost();
  	// 		for (int a = 0; a < getAgents(); a++)
			// {
			// 	std::cout << "Agent: " << a << std::endl;
			// 	for (State* st: current->m_solution[a])
			// 	{
			// 		std::cout << *st << std::endl;
			// 	}
			// }
  			if (solCost <= m_bound)
  			{
  				std::cout << "Solution is Satisfiable!" << std::endl;
  				solution = current->m_solution;
				return true;
  			}
  			else
  			{
  				std::cout << "Solution is Not satisfiable." << std::endl;
  				open_heap.pop();
  				conflictNode *current = open_heap.top();
  			}
		}
		else
		{
			// remove from list
			open_heap.pop();
			// for each agent in conflict (currently only two at a time)
			for (int a = 0; a < 2; a++)
			{
				// std::cout << "found conflict. " << std::endl;
				if (c->type == Conflict::Vertex)
				{
					// std::cout << *c << std::endl;

					// vertex Conflict is defined as follows

					// c->type = Conflict::Vertex;
					// c->time1 = st1.time;
					// c->agent1 = a1; c->agent2 = a2;
					// c->x1 = st1.x ; c->y1 = st1.y;

					// as per paper, new node is initialized with parent solution
					std::vector<std::vector<State*>> empty;
					conflictNode *n = new conflictNode(empty);
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

					auto t1 = std::chrono::high_resolution_clock::now();
					n->m_solution = lowLevelSearch(startStates, constriants, 
						n->parent->m_solution);
					auto t2 = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
					timeAstar = timeAstar + duration.count();

					bool valid = true;
					for (std::vector<State*> sol: n->m_solution)
					{
						if (sol.size() == 0)
						{
							valid = false;
							break;
						}
					}
					// update cost of solution if it is a valid one
					// std::cout << n << std::endl;
					if (valid)
					{

						// for (int a = 0; a < getAgents(); a++)
						// {
						// 	std::cout << "Agent: " << a << std::endl;
						// 	for (State* st: n->m_solution[a])
						// 	{
						// 		std::cout << *st << std::endl;
						// 	}
						// }

						// std::cin >>test;


						n->m_cost = n->calcCost();
						open_heap.emplace(n);
					}
					// else
					// {
					// 	std::cout << open_heap.size() << std::endl;
					// }
					// std::cout << "New Solution: " << std::endl;
					// for (int a = 0; a < n->m_solution.size(); a++)
					// {
					// 	std::cout << "Agent: " << a << std::endl;
					// 	for (State st: n->m_solution[a])
					// 		std::cout << st << std::endl;
					// }


				}
				if (c->type == Conflict::Edge)
				{
					// std::cout << "Current Solution: " << std::endl;
					// for (int a = 0; a < current->m_solution.size(); a++)
					// {
					// 	std::cout << "Agent: " << a << std::endl;
					// 	for (State st: current->m_solution[a])
					// 		std::cout << st << std::endl;
					// }
					// std::cout << *c << std::endl;
					// Edge Conflict are defined as follows! 
					// c->type = Conflict::Edge;
					// c->time1 = a1Curr.time;
					// c->time2 = a1Nxt.time;
					// c->agent1 = a1;
					// c->agent2 = a2;
					// c->x1 = a1Curr.x ; c->y1 = a1Curr.y;
					// c->x2 = a1Nxt.x ; c->y2 = a1Nxt.y;
					std::vector<std::vector<State*>> empty;
					// as per paper, new node is initialized with parent solution
					conflictNode *n = new conflictNode(empty);
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

					auto t1 = std::chrono::high_resolution_clock::now();
					n->m_solution = lowLevelSearch(startStates, constriants, 
						n->parent->m_solution);
					auto t2 = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
					timeAstar = timeAstar + duration.count();

					bool valid = true;
					for (std::vector<State*> sol: n->m_solution)
					{
						if (sol.size() == 0)
						{
							valid = false;
							break;
						}
					}
					// update cost of solution if it is a valid one
					if (valid)
					{

						// for (int a = 0; a < getAgents(); a++)
						// {
						// 	std::cout << "Agent: " << a << std::endl;
						// 	for (State* st: n->m_solution[a])
						// 	{
						// 		std::cout << *st << std::endl;
						// 	}
						// }
						// std::cin >>test;
						n->m_cost = n->calcCost();
						open_heap.emplace(n);
					}
					// else
					// {
					// 	std::cout << open_heap.size() << std::endl;
					// }
					// std::cout << "New Solution: " << std::endl;
					// for (int a = 0; a < n->m_solution.size(); a++)
					// {
					// 	std::cout << "Agent: " << a << std::endl;
					// 	for (State st: n->m_solution[a])
					// 		std::cout << st << std::endl;
					// }
					// exit(1);
				}
			}
		}
	}
	std::cout << "No solution" << std::endl;
	return 0;
}
