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
	m_numAgents = m_env->getGoals().size();
	Constraint m_constraint{Constraint()};
};


bool CBS::conflictNode::is_disjoint(const std::vector<State> v1, const std::vector<State> v2) const
{
    if(v1.empty() || v2.empty()) return true;

    typename std::vector<State>::const_iterator 
        it1 = v1.begin(), 
        it1End = v1.end();

    // if(*it1 > *v2.rbegin() || *it2 > *v1.rbegin()) return true;

    while(it1 != it1End)  //  && it2 != it2End
    {
    	typename std::vector<State>::const_iterator 
        it2 = v2.begin(), 
        it2End = v2.end();
    	while (it2 != it2End)
    	{
    		if(it1->isSameLocation(*it2))
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
	// std::cout << "Segmenting Solution" << "\n";
	// for (std::vector<State> sol: n->m_solution)
	// {
	// 	for (State st: sol)
	// 	{
	// 		std::cout << st << std::endl;
	// 	}
	// }

	// 1. find longest solution
	int longTime = 0;
	for (std::vector<State> Asol: m_solution)
	{
		if (Asol.size() > longTime)
			longTime = Asol.size();
	}
	// std::cout << longTime << "\n";

	// 2. init visited list and a segment indexing variable
	std::vector<std::vector<State>> agentVisited(m_solution.size());
	int lastSegmentTime = 0;
	int currCost = 1;

	// 3. segment the solution
	for (int currTime = 0; currTime <= longTime; currTime++)
	{
		// 3a. add visited state for currTime
		for (int a = 0; a < m_solution.size(); a++)
		{
			std::vector<State> currSol = m_solution[a];
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
							for (int t = lastSegmentTime; t < (currTime); t++)
							{
								// exit(1);
								if (t < m_solution[a].size())
									m_solution[a][t].cost = currCost;
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
							std::vector<State> currSol = m_solution[a];
							if (currTime < currSol.size())
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
				m_solution[a][t].cost = currCost;
			}
		}
	}
	return currCost;
	// std::cout << "done with function" << std::endl;

}


Solution CBS::lowLevelSearch(const std::vector<State>& startStates, 
		std::vector<Constraint*> constraints)
{
	// we have a list of constraints for all agents from current node
	// to root node. 

	// low level planner should only be provided with the constraints
	// it needs to worry about.

	// std::cout << "In CBS finding MA solution" << std::endl;
	Solution sol;
	std::vector<State> singleSol;
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
		// std::cout << "entering planner" << std::endl;
		// std::cout << "Agent: " << a << ", Start: " << startStates[a] << ", Goal: " << m_planner->getEnv()->getGoals()[a] << std::endl;
		if (cont)
		{
			bool success = m_planner->plan(startStates[a], singleSol, agentRelevantCs);
			if (success)
				sol.push_back(singleSol);
			else
				cont = false;
		}
		
		// std::cout << "exited planner" << std::endl;
		// if sol found, add it --- else, return no solution
		
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
				int t1 = n->m_solution[a1].size();
				int t2 = n->m_solution[a2].size();

				// if statement to check the shortest path
				if (t1 < t2)
				{
					// iterate through t1 looking for vertex constraints
					for (int t = 0; t < t1; t++)
					{
						// fist, vertex conflicts
						// get both states at same time step
						State a1Curr = n->m_solution[a1][t];
						State a2Curr = n->m_solution[a2][t];

						if ((a1Curr.x == a2Curr.x) && (a1Curr.y == a2Curr.y) && (a1Curr.time == a2Curr.time))
						{
							// std::cout << "found a conflict" << std::endl;
							// std::cout << st2 << std::endl;
							// a1 and a2 are the indices of the agents
							// Conflict *c = new Conflict(a1, a2, st1);
							c = new Conflict();
							c->type = Conflict::Vertex;
							c->time1 = a1Curr.time;
							c->agent1 = a1; c->agent2 = a2;
							c->x1 = a1Curr.x ; c->y1 = a1Curr.y;
							return c;
						}
						// check for edge constraint with next time step
						State a1Nxt = n->m_solution[a1][t + 1];
						State a2Nxt = n->m_solution[a2][t + 1];

						// check to see if two agents flip
						if ((a1Curr.x == a2Nxt.x) && (a1Curr.y == a2Nxt.y) && (a2Curr.x == a1Nxt.x) && (a2Curr.y == a1Nxt.y) )
						{
							// std::cout << "Found Edge Conflict: " << std::endl;
							// std::cout << st1 << std::endl;
							// std::cout << st3 << std::endl;
							c = new Conflict();
							c->type = Conflict::Edge;
							c->time1 = a1Curr.time;
							c->time2 = a1Nxt.time;
							c->agent1 = a1;
							c->agent2 = a2;
							c->x1 = a1Curr.x ; c->y1 = a1Curr.y;
							c->x2 = a1Nxt.x ; c->y2 = a1Nxt.y;
							// std::cout << c->agent1 << " " << c->agent2 << std::endl;
							return c;
						}
					}
				}
				else
				{
					// iterate through t2 looking for constraints
					for (int t = 0; t < t2; t++)
					{
						// fist, vertex conflicts
						// get both states at same time step
						State a1Curr = n->m_solution[a1][t];
						State a2Curr = n->m_solution[a2][t];

						if ((a1Curr.x == a2Curr.x) && (a1Curr.y == a2Curr.y) && (a1Curr.time == a2Curr.time))
						{
							// std::cout << "found a conflict" << std::endl;
							// std::cout << st2 << std::endl;
							// a1 and a2 are the indices of the agents
							// Conflict *c = new Conflict(a1, a2, st1);
							c = new Conflict();
							c->type = Conflict::Vertex;
							c->time1 = a1Curr.time;
							c->agent1 = a1; c->agent2 = a2;
							c->x1 = a1Curr.x ; c->y1 = a1Curr.y;
							return c;
						}
						// check for edge constraint with next time step
						State a1Nxt = n->m_solution[a1][t + 1];
						State a2Nxt = n->m_solution[a2][t + 1];

						// check to see if two agents flip
						if ((a1Curr.x == a2Nxt.x) && (a1Curr.y == a2Nxt.y) && (a2Curr.x == a1Nxt.x) && (a2Curr.y == a1Nxt.y) )
						{
							// std::cout << "Found Edge Conflict: " << std::endl;
							// std::cout << st1 << std::endl;
							// std::cout << st3 << std::endl;
							c = new Conflict();
							c->type = Conflict::Edge;
							c->time1 = a1Curr.time;
							c->time2 = a1Nxt.time;
							c->agent1 = a1;
							c->agent2 = a2;
							c->x1 = a1Curr.x ; c->y1 = a1Curr.y;
							c->x2 = a1Nxt.x ; c->y2 = a1Nxt.y;
							// std::cout << c->agent1 << " " << c->agent2 << std::endl;
							// std::cout << "Current" << std::endl;
							// std::cout << "Agent: " << c->agent1 << " " << a1Curr << " " << " Agent: " << c->agent2 << " " << a2Curr << std::endl;
							// std::cout << "Next" << std::endl;
							// std::cout << "Agent: " << c->agent1 << " " << a1Nxt << " " << " Agent: " << c->agent2 << " " << a2Nxt << std::endl;
							return c;
						}
					}
				}
			}
		}
	}
	return c;
}

bool CBS::plan(const std::vector<State>& startStates, Solution& solution)
{
	auto start = std::chrono::high_resolution_clock::now();

	solution.clear();

	// init open min-heap
	std::priority_queue <conflictNode*, std::vector<conflictNode*>, myconflictComparator > open_heap;
	// used for seeing if a node is in the heap
	// std::unordered_set<State, std::hash<State>> m_open_list;
	
	// find solution with no constraints
	std::vector<Constraint*> constriants;
	// std::cout << "searching for init sol" << std::endl;
	Solution rootSol = lowLevelSearch(startStates, constriants);
	// std::cout << "Initial Solution: " << std::endl;
	// for (int a = 0; a < rootSol.size(); a++)
	// {
	// 	std::cout << "Agent: " << a << std::endl;
	// 	for (State st: rootSol[a])
	// 		std::cout << st << std::endl;
	// }


	conflictNode *rootNode;
	
	if (rootSol.size() != m_numAgents)
		return false;
	else
		rootNode = new conflictNode(rootSol);

	std::cout << "Root Node Cost: " << rootNode->m_cost << std::endl;

	open_heap.emplace(rootNode);

	while (!open_heap.empty())
	{

		// get best node
		conflictNode *current = open_heap.top();

		// validate it for conflicts
		Conflict *c = validateSolution(current);

		// std::cout << std::endl;
		// std::cout << "Conflict Found: " << c->m_state << std::endl;
		// std::cin >> test;

		if (c == nullptr)
		{
			// if no conflicts occur, then we found a solution
			auto stop = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
			auto duration2 = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
  			std::cout << "Duration: " << duration.count() << " micro seconds" << " or approx. " << duration2.count() << " seconds" << std::endl;
  			// int solCost = segmentSolution(current->m_solution);
  			int solCost = current->segmentSolution();
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

					n->m_solution = lowLevelSearch(startStates, constriants);
					// update cost of solution if it is a valid one
					if (n->m_solution.size() == m_numAgents)
					{
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

					n->m_solution = lowLevelSearch(startStates, constriants);
					// update cost of solution if it is a valid one
					if (n->m_solution.size() == m_numAgents)
					{
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
