#include "../includes/Cbs.h"


CBS::CBS(Environment *env): m_env(env)
{
	m_planner = new A_star(m_env);
	m_numAgents = m_env->getGoals().size();
	Constraint m_constraint{Constraint()};
};

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

	for (int a = 0; a < startStates.size(); a++) // (const State &st : startStates)
	{	
		// need to provide A* with constraints relevant to 
		// the agent. 

		// for each agent, cycle through c
		std::vector<Constraint*> agentRelevantCs;
		for (Constraint *c: constraints)
		{
			if (c->getVertexConstraint()->m_agent == a)
				agentRelevantCs.push_back(c);
		}

		bool success = m_planner->plan(startStates[a], singleSol, agentRelevantCs);
		// if sol found, add it --- else, return no solution
		if (success)
			sol.push_back(singleSol);
		
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
				// check that their solutions are not the same

				// first, get time both solutions
				int t1 = n->m_solution[a1].back().time;
				int t2 = n->m_solution[a2].back().time;

				// if statement to check the shortest path
				if (t1 < t2)
				{
					for (int t = 0; t < t1; t++)
					{
						// get both states at same time step
						State st1 = n->m_solution[a1][t];
						State st2 = n->m_solution[a2][t];

						if ((st1.x == st2.x) && (st1.y == st2.y) && (st1.time == st2.time))
						{
							// std::cout << "found a conflict" << std::endl;
							// std::cout << st1 << std::endl;
							// a1 and a2 are the indices of the agents
							Conflict *c = new Conflict(a1, a2, st1);
							return c;
						}
					}
				}
				else
				{
					for (int t = 0; t < t2; t++)
					{
						// get both states at same time step
						State st1 = n->m_solution[a1][t];
						State st2 = n->m_solution[a2][t];

						if ((st1.x == st2.x) && (st1.y == st2.y) && (st1.time == st2.time))
						{
							// std::cout << "found a conflict" << std::endl;
							// std::cout << st2 << std::endl;
							// a1 and a2 are the indices of the agents
							Conflict *c = new Conflict(a1, a2, st1);
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

	solution.clear();

	// init open min-heap
	std::priority_queue <conflictNode*, std::vector<conflictNode*>, myconflictComparator > open_heap;
	// used for seeing if a node is in the heap
	std::unordered_set<State, std::hash<State>> m_open_list;
	
	// find solution with no constraints
	std::vector<Constraint*> constriants;
	Solution rootSol = lowLevelSearch(startStates, constriants);

	conflictNode *rootNode;
	
	if (rootSol.size() != m_numAgents)
		return false;
	else
		rootNode = new conflictNode(rootSol);

	open_heap.emplace(rootNode);

	std::string test;

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
			solution = current->m_solution;
			return true;
		}
		else
		{
			// remove from list
			open_heap.pop();
			// for each agent in conflict (currently only two at a time)
			for (int a = 0; a < 2; a++)
			{
				// as per paper, new node is initialized with parent solution
				conflictNode *n = new conflictNode(current->m_solution);
				// new node branches from current
				n->parent = current;
				// create constraint -- only for a single agent
				VertexConstraint *v = new VertexConstraint(a, c->m_state);
				// add constraint to node
				n->m_constraint.add(v);
				// get vector of entire constraints
				std::vector<Constraint*> constriants;
				conflictNode *currNode = n;
				while (currNode->parent != nullptr)
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
			}
		}
		// break;
	}
	return 0;
}
