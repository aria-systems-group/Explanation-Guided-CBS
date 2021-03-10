#include "../includes/Cbs.h"
#include <chrono>


// Constructor
CBS::CBS(Environment *env, const int bound): m_env(env), m_bound{bound}
{
	m_planner = new A_star(m_env);
	m_planner->updateBound(m_bound);
	m_numAgents = m_env->getGoals().size();
	Constraint m_constraint{Constraint()};
};


// calculate constraint specific solution
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

	Solution sol(getAgents());
	std::vector<State*> singleSol;
	std::vector<Constraint*> agentRelevantCs;
	bool cont = true;

	if (parent.size() == 0)
	{
		// at root node
		// plan for initial agent first
		bool success = m_planner->plan(startStates[0], singleSol, 
			agentRelevantCs, parent);

		if (success)
		{
			sol[0] = singleSol;
			m_planner->getEnv()->updateAgent();
			// plan for remaining agents using Exp-A*
			for (int a = 1; a < startStates.size(); a++)
			{
				if (cont)
				{
					bool success = m_planner->plan(startStates[a], singleSol, 
						agentRelevantCs, sol);
					if (success)
						sol[a] = singleSol;
					else
					{
						cont = false;
					}
					m_planner->getEnv()->updateAgent();
				}
			}
		}
		else
		{
			std::cout << "No Solution To Problem Exists." << std::endl;
			return sol;
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

	// update agent to the correct one
	while (m_planner->getEnv()->getAgent() != 0)
	{
		m_planner->getEnv()->updateAgent();
	}


	return sol;
}

// given a solution, find conflicts
// this is where we add explainability conflicts
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
				// std::cout << "getting times" << std::endl;
				int t1 = n->m_solution[a1].size();
				int t2 = n->m_solution[a2].size();
				// std::cout << "got times" << std::endl;

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
							c->x1 = a1Curr->x ; c->y1 = a1Curr->y;
							return c;
						}

						if ((t + 1) < t2)
						{
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
	
	// find solution with no constraints
	std::vector<Constraint*> constriants;
	Solution par;

	auto t1 = std::chrono::high_resolution_clock::now();
	Solution rootSol = lowLevelSearch(startStates, constriants, par);
	auto t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
	timeAstar = timeAstar + duration.count();

	conflictNode *rootNode;
	int treeSize = 0;
	
	bool valid = true;
	for (std::vector<State*> sol: rootSol)
	{
		if (sol.size() == 0)
		{
			valid = false;
			break;
		}
	}

	if (valid)
	{
		rootNode = new conflictNode(rootSol);
		open_heap.emplace(rootNode);
		treeSize ++;
	}
	else
	{
		std::cout << "No CBS solution to problem. " << std::endl;
		return false;
	}

	while (!open_heap.empty())
	{

		// get best node
		conflictNode *current = open_heap.top();

		// validate it for conflicts
		Conflict *c = validateSolution(current);

		if (c == nullptr)
		{
			// if no conflicts occur, then we found a solution
			auto stop = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  			std::cout << "Duration: " << duration.count() << " microseconds" << " or approx. " << 
  				(duration.count() / 1000000.0) << " seconds" << std::endl;
  			
  			std::cout << "Time Spent in A*: " << timeAstar << " microseconds" << 
  				" or approx. " << (timeAstar / 1000000.0) << " seconds" << std::endl;

  			std::cout << "Solution is Satisfiable!" << std::endl;
  			solution = current->m_solution;
  			std::cout << "Number of Conflict Nodes: " << treeSize << std::endl;
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
					// vertex Conflict is defined as follows
					// as per paper, new node is initialized with parent solution
					std::vector<std::vector<State*>> empty;
					conflictNode *n = new conflictNode(empty);
					// new node branches from current
					n->parent = current;
					// create constraint
					if (a == 0)
					{
						VertexConstraint *v = new VertexConstraint(c->agent1, 
							c->time1, c->x1, c->y1);
						n->m_constraint.add(v);
					}
					else
					{
						VertexConstraint *v = new VertexConstraint(c->agent2, 
							c->time1, c->x1, c->y1);
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
					if (valid)
					{
						n->m_cost = n->calcCost();
						open_heap.emplace(n);
						treeSize ++;
					}
				}
				if (c->type == Conflict::Edge)
				{
					// as per paper, new node is initialized with parent solution
					conflictNode *n = new conflictNode(current->m_solution);
					// new node branches from current
					// create constraint -- changes based on which agent we are talking about

					if (a == 0)
					{
						EdgeConstraint *e = new EdgeConstraint(c->agent1, c->time1, 
							c->time2, c->x1, c->y1, c->x2, c->y2);
						n->m_constraint.add(e);
					}
					else
					{
						EdgeConstraint *e = new EdgeConstraint(c->agent2, c->time1, 
							c->time2, c->x2, c->y2, c->x1, c->y1);
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
						n->m_cost = n->calcCost();
						open_heap.emplace(n);
						treeSize ++;
					}
				}
			}
		}
	}
	std::cout << "No solution found..." << std::endl;
	std::cout << "Number of Conflict Nodes: " << treeSize;
	return 0;
}
