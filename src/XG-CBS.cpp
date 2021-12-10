// my includes
#include "../includes/XG-CBS.h"
// std includes
#include <filesystem>


// Constructor
XG_CBS::XG_CBS(Environment *env, const int bound, const double percent_exp): m_env(env), m_bound{bound}
{
	m_planner_A = new A_star(m_env);
	m_planner_H = new XG_Astar_H(m_env, percent_exp);
	m_planner_S = new S_Astar(m_env);
	m_planner = new XG_Astar(m_env);
	m_numAgents = m_env->getGoals().size();
	Constraint m_constraint{Constraint()};
};

bool XG_CBS::is_disjoint(const std::vector<State*> v1, 
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

void XG_CBS::clear()
{
	// clear the open heap by deleting all data within each of its elements
	while (!open_heap_.empty())
	{
		// get best node
		conflictNode *n = open_heap_.top();
		n->children.clear();
		delete n->m_constraint.getVertexConstraint();
		delete n->m_constraint.getEdgeConstraint();
		// delete all states in the solution
		for (std::vector<State*> path: n->m_solution)
		{
			// possible memory leak here
			path.clear();
		}
		(n->m_solution).clear();
		open_heap_.pop();
		delete n;
	}
	// clear the closed heap by deleting all data within each of its elements
	for (auto it = closed_set_.begin(); it != closed_set_.end(); it++) {
        // get best node
		conflictNode *n = *it;
		n->children.clear();
		delete n->m_constraint.getVertexConstraint();
		delete n->m_constraint.getEdgeConstraint();
		for (std::vector<State*> path: n->m_solution)
		{
			// possible memory leak here.
			path.clear();
		}
		(n->m_solution).clear();
		delete n;
    }
    closed_set_.erase(closed_set_.begin(), closed_set_.end());
    // clear other saved data
    delete m_planner_A;
	delete m_planner_H;
	delete m_planner_S;
}

int XG_CBS::segmentSolution(Solution sol)
{
	// get longest time of the plan
	int longTime = 0;
	for (int a = 0; a < sol.size(); a++)
	{
		int tmp = sol[a].back()->time;
		if (tmp > longTime)
			longTime = tmp;
	}
	// init a visited list for all agents and an indexing variable
	std::vector<std::vector<State*>> agentVisited(m_env->getGoals().size());
	int lastSegmentTime = 0;
	int currCost = 1;

	// for the entire length of the plan
	for (int currTime = 0; currTime <= longTime; currTime++)
	{
		// add visited state for currTime
		for (int a = 0; a < sol.size(); a++)
			if (sol[a].back()->time >= currTime)
				agentVisited[a].push_back(sol[a][currTime]);

		// see if agent visited is disjoint
		for (int a1 = 0; a1 < sol.size(); a1++)
		{
			for (int a2 = 0; a2 < sol.size(); a2++)
			{
				// only check agents that are not the same
				if (a1 != a2)
				{
					// check disjoint
					bool disjoint = is_disjoint(agentVisited[a1], agentVisited[a2]);
					if (!disjoint)
					{
						// add cost for all states prior to currTime
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

// calculate constraint specific solution
Solution XG_CBS::lowLevelSearch(const std::vector<State*>& startStates, 
		std::vector<Constraint*> constraints, Solution& parent, 
		const bool use_XG_A, const bool use_XG_A_H, const bool use_S_A, bool &isDone)
{
	// we have a list of constraints for entire CBS branch
	if (use_XG_A)
	{
		// use EG-A* object in m_planner
		// make sure we reset the agent index for good book keeping
		while (m_planner->getEnv()->getAgent() != 0)
		{
			m_planner->getEnv()->updateAgent();
		}
		// init things we need
		Solution sol(getAgents());  // soon-to-be plan
		std::vector<State*> singleSol;  // soon-to-be path 
		std::vector<Constraint*> agentRelevantCs;  // soon-to-be agent relevent constraints
		bool cont = true;

		if (parent.size() == 0)
		{
			// at root node
			// plan for initial agent first
			bool success = m_planner->plan(startStates[0], singleSol, 
				agentRelevantCs, parent, isDone);

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
							agentRelevantCs, sol, isDone);
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
					agentRelevantCs, sol, isDone);
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
						agentRelevantCs, sol, isDone);
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
		// reset the agent index
		while (m_planner->getEnv()->getAgent() != 0)
		{
			m_planner->getEnv()->updateAgent();
		}
		return sol;
	}
	else if (use_XG_A_H)
	{
		// use EG-A*-H object in m_planner_H
		while (m_planner_H->getEnv()->getAgent() != 0)
		{
			m_planner_H->getEnv()->updateAgent();
		}

		Solution sol(getAgents());
		std::vector<State*> singleSol;
		std::vector<Constraint*> agentRelevantCs;
		bool cont = true;

		if (parent.size() == 0)
		{
			// at root node
			// plan for initial agent first
			bool success = m_planner_H->plan(startStates[0], singleSol, 
				agentRelevantCs, parent, isDone);

			if (success)
			{
				sol[0] = singleSol;
				m_planner_H->getEnv()->updateAgent();
				// plan for remaining agents using Exp-A*
				for (int a = 1; a < startStates.size(); a++)
				{
					if (cont)
					{
						bool success = m_planner_H->plan(startStates[a], singleSol, 
							agentRelevantCs, sol, isDone);
						if (success)
							sol[a] = singleSol;
						else
						{
							cont = false;
						}
						m_planner_H->getEnv()->updateAgent();
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
				while (m_planner_H->getEnv()->getAgent() != currVC->agent)
				{
					m_planner_H->getEnv()->updateAgent();
				}

				bool success = m_planner_H->plan(startStates[currVC->agent], singleSol, 
					agentRelevantCs, sol, isDone);
				if (success)
				{
					sol[currVC->agent] = singleSol;
				}
				else
					cont = false;
				m_planner_H->getEnv()->updateAgent();
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
				while (m_planner_H->getEnv()->getAgent() != currEC->agent)
				{
					m_planner_H->getEnv()->updateAgent();
				}
					bool success = m_planner_H->plan(startStates[currEC->agent], singleSol, 
						agentRelevantCs, sol, isDone);
					if (success)
					{
						// sol[currEC->agent].clear();
						sol[currEC->agent] = singleSol;
					}
					else
						cont = false;
				m_planner_H->getEnv()->updateAgent();
			}
		}
		// update agent to the correct one
		while (m_planner_H->getEnv()->getAgent() != 0)
		{
			m_planner_H->getEnv()->updateAgent();
		}
		return sol;
	}
	else if (use_S_A)
	{
		// use A* object in m_planner_A
		// reset agent idx 
		while (m_planner_S->getEnv()->getAgent() != 0)
		{
			m_planner_S->getEnv()->updateAgent();
		}

		Solution sol(getAgents());
		std::vector<State*> singleSol;
		std::vector<Constraint*> agentRelevantCs;
		bool cont = true;

		if (parent.size() == 0)
		{
			// at root node
			// plan for initial agent first
			bool success = m_planner_S->plan(startStates[0], singleSol, 
				agentRelevantCs, parent, isDone);

			if (success)
			{
				sol[0] = singleSol;
				m_planner_S->getEnv()->updateAgent();
				// plan for remaining agents using Exp-A*
				for (int a = 1; a < startStates.size(); a++)
				{
					if (cont)
					{
						bool success = m_planner_S->plan(startStates[a], singleSol, 
							agentRelevantCs, sol, isDone);
						if (success)
							sol[a] = singleSol;
						else
						{
							cont = false;
						}
						m_planner_S->getEnv()->updateAgent();
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
				while (m_planner_S->getEnv()->getAgent() != currVC->agent)
				{
					m_planner_S->getEnv()->updateAgent();
				}

				bool success = m_planner_S->plan(startStates[currVC->agent], singleSol, 
					agentRelevantCs, sol, isDone);
				if (success)
				{
					sol[currVC->agent] = singleSol;
				}
				else
					cont = false;
				m_planner_S->getEnv()->updateAgent();
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
				while (m_planner_S->getEnv()->getAgent() != currEC->agent)
				{
					m_planner_S->getEnv()->updateAgent();
				}
					bool success = m_planner_S->plan(startStates[currEC->agent], singleSol, 
						agentRelevantCs, sol, isDone);
					if (success)
					{
						// sol[currEC->agent].clear();
						sol[currEC->agent] = singleSol;
					}
					else
						cont = false;
				m_planner_S->getEnv()->updateAgent();
			}
		}
		// reset
		while (m_planner_S->getEnv()->getAgent() != 0)
		{
			m_planner_S->getEnv()->updateAgent();
		}
		return sol;
	}
	else if (!use_XG_A && !use_XG_A_H && !use_S_A)
	{
		// use A* object in m_planner_A
		// reset agent idx 
		while (m_planner_A->getEnv()->getAgent() != 0)
		{
			m_planner_A->getEnv()->updateAgent();
		}

		Solution sol(getAgents());
		std::vector<State*> singleSol;
		std::vector<Constraint*> agentRelevantCs;
		bool cont = true;

		if (parent.size() == 0)
		{
			// at root node
			// plan for initial agent first
			bool success = m_planner_A->plan(startStates[0], singleSol, 
				agentRelevantCs, isDone);

			if (success)
			{
				sol[0] = singleSol;
				m_planner_A->getEnv()->updateAgent();
				// plan for remaining agents using Exp-A*
				for (int a = 1; a < startStates.size(); a++)
				{
					if (cont)
					{
						bool success = m_planner_A->plan(startStates[a], singleSol, 
							agentRelevantCs, isDone);
						if (success)
							sol[a] = singleSol;
						else
						{
							cont = false;
						}
						m_planner_A->getEnv()->updateAgent();
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
				while (m_planner_A->getEnv()->getAgent() != currVC->agent)
				{
					m_planner_A->getEnv()->updateAgent();
				}
				bool success = m_planner_A->plan(startStates[currVC->agent], singleSol, 
					agentRelevantCs, isDone);
				if (success)
				{
					sol[currVC->agent] = singleSol;
				}
				else
					cont = false;
				m_planner_A->getEnv()->updateAgent();
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
				while (m_planner_A->getEnv()->getAgent() != currEC->agent)
				{
					m_planner_A->getEnv()->updateAgent();
				}
				bool success = m_planner_A->plan(startStates[currEC->agent], singleSol, 
					agentRelevantCs, isDone);
				if (success)
				{
					// sol[currEC->agent].clear();
					sol[currEC->agent] = singleSol;
				}
				else
					cont = false;
				m_planner_A->getEnv()->updateAgent();
			}
		}
		// reset
		while (m_planner_A->getEnv()->getAgent() != 0)
		{
			m_planner_A->getEnv()->updateAgent();
		}
		if (cont)
			int cost = segmentSolution(sol);
		return sol;
	}
	else
	{
		std::cout << "Low-Level Planner not properly specified. Exiting with failure." << std::endl;
		exit(1);
	}
}

// given a solution, find conflicts
// this is where we add explainability conflicts
std::vector<Conflict*> XG_CBS::validateSolution(conflictNode *n)
{
	// this function will step through all solution and look for conflicts
	// if conflict is found, then we add it to the node
	std::vector<Conflict*> cnf;
	Conflict* c = nullptr;

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
							cnf.push_back(c);
							return cnf;
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
								cnf.push_back(c);
								return cnf;
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
							cnf.push_back(c);
							return cnf;
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
								cnf.push_back(c);
								return cnf;
							}
						}
					}
				}
			}
		}
	}
	// no actual conflicts found
	// time to check explanations
	const int cost = n->getSegCost();
	if (cost > getBound() && getBound() > 0)
	{

		// get long time
		int longTime = 0;
		int longAgent;
		for (int a = 0; a < n->m_solution.size(); a++)
		{
			std::vector<State*> sol = n->m_solution[a];
			if (sol.back()->time > longTime)
			{
				longTime = sol.back()->time;
				longAgent = a;
			}
		}
		// create vectors of conflicts
		std::vector<Conflict*> expCs;
		// cycle through long time to search for segments
		int currCost = 1;
		std::vector<std::vector<State*>> agentVisited(getAgents());
		for (int t = 0; t <= longTime; t++)
		{
			// check for segment
			if (n->m_solution[longAgent][t]->cost > currCost)
			{
				// found a new segment
				// add current time to visited
				for (int a = 0; a < getAgents(); a++)
				{
					if (n->m_solution[a].back()->time >= t)
					{
						agentVisited[a].push_back(n->m_solution[a][t]);
					}
				}
				// somewhere in agentVisited, a state is repeated
				// find it using a double for loop
				for (int a1 = 0; a1 < getAgents(); a1++)
				{
					for (int a2 = 0; a2 < getAgents(); a2++)
					{
						if (a1 != a2)
						{
							// for all agents
							// check for repeates for all visited states
							for (int k1 = 0; k1 < agentVisited[a1].size(); k1++)
							{
								for (int k2 = 0; k2 < agentVisited[a2].size(); k2++)
								{
									if (agentVisited[a1][k1]->isSameLocation(agentVisited[a2][k2]))
									{
										// return first conflict
										// times are k1 and k2
										// states are in visited a1 and a2
										c = new Conflict;
										c->type = Conflict::Explanation;
										c->time1 = agentVisited[a1][k1]->time;
										c->time2 = agentVisited[a2][k2]->time;
										c->agent1 = a1;
										c->agent2 = a2;
										c->x1 = agentVisited[a1][k1]->x ; c->y1 = agentVisited[a1][k1]->y;
										c->x2 = agentVisited[a2][k2]->x ; c->y2 = agentVisited[a2][k2]->y;
										if (!isConflictRepeat(c, expCs))
											expCs.push_back(c);
										currCost++;
									}
								}
							}
						}
					}
				}
				for (int a = 0; a < getAgents(); a++)
					agentVisited[a].clear();
			}
			else
			{
				// add states to visited
				for (int a = 0; a < getAgents(); a++)
				{
					if (n->m_solution[a].back()->time >= t)
						agentVisited[a].push_back(n->m_solution[a][t]);
				}
			}
		}
		if (expCs.size() != 0)
			return expCs;
	}
	else if (getBound() == 0)
	{
		std::cout << "A solution cannot be explained in less than one segment." << std::endl;
		std::cout << "Exiting ExpCBS prematurely. Please retry using a feasible explanation bound." << std::endl;
		exit(1);
	}	
	return cnf;
}

bool XG_CBS::isConflictRepeat(Conflict *curr, std::vector<Conflict*> vec)
{
	for (Conflict *prev: vec)
	{
		// if agents are the same
		if (prev->agent1 == curr->agent2 && prev->agent2 == curr->agent1)
		{
			// if agents are at same location
			if (prev->x1 == curr->x1 && prev->y1 == curr->y1)
				return true;
		}
	}
	return false;
}

bool XG_CBS::plan(const std::vector<State*>& startStates, Solution& solution, 
	const bool use_XG_A, const bool use_XG_A_H, const bool use_S_A)
{
	bool isOverTime = false;
	bool isSolved = false;
	std::cout << "Now Planning with XG-CBS" << std::endl;
	
	// set timer
	const std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();
	std::thread timeThread(Timer(), start, solveTime_, std::ref(isOverTime), std::ref(isSolved));
	
	// flags to store debug data
	double timeAstar = 0;
	int numExps = 0;
	int treeSize = 0;
	int nodeIdx = 1;
	std::string test;
	std::string dirName;
	Solution empty(getAgents());

	solution.clear();
	
	// find root solution using low-level search
	std::vector<Constraint*> constriants;
	Solution par;
	Solution rootSol;
	auto t1 = std::chrono::high_resolution_clock::now();
	rootSol = lowLevelSearch(startStates, constriants, par, use_XG_A, use_XG_A_H, use_S_A, isOverTime);
	auto t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
	timeAstar = timeAstar + duration.count();

	// turn root solution into root node of tree (if valid)
	conflictNode *rootNode;
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
		open_heap_.emplace(rootNode);
		rootNode->updateIdx(nodeIdx);
		nodeIdx++;
		treeSize ++;
	}
	else
	{
		printf("%s: No solution found in %0.1f seconds.\n", "XG-CBS", solveTime_);
		timeThread.join();
		return false;
	}

	// save the root solution
	m_root = rootNode;

	// plan until time limit or empty queue
	while (!open_heap_.empty() && !isOverTime && !isSolved)
	{
		// get best node
		conflictNode *current = open_heap_.top();
		current->m_eval = true;
		// update the structure of the tree and information of all nodes
		// validate it for conflicts
		std::vector<Conflict*> cnf = validateSolution(current);

		if (cnf.empty())
		{
			// if no conflicts occur, then we found a solution
  			current->m_solFlag = true;
  			m_solution = current;
  			solution = current->m_solution;
  			auto stop = std::chrono::high_resolution_clock::now();
  			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  			updateCompTime((duration.count() / 1000000.0));
  			// get final cost
  			int cost = 0;
  			for (std::vector<State*> a: solution)
  			{
  				if (a.back()->cost > cost)
  					cost = a.back()->cost;
  			}
  			// output results nicely
  			printf("%s: Solution found with cost %i.\n", "XG-CBS", cost);
  			printf("%s: Solution found in %0.4f seconds.\n", "XG-CBS", (duration.count() / 1000000.0));
  			printf("%s: Spent approximately %0.4f seconds in low-level search.\n", "XG-CBS", (timeAstar / 1000000.0));
  			printf("%s: Evaluated %lu conflict nodes.\n", "XG-CBS", (closed_set_.size() + 1));
			isSolved = true;
			timeThread.join();
			return isSolved;
		}
		else
		{
			// put current in closed
			closed_set_.insert(current);
			// remove from heap
			open_heap_.pop();
			// for each agent in conflict (currently only two at a time)
			for (Conflict* c: cnf)
			{
				for (int a = 0; a < 2; a++)
				{
					if (c->type == Conflict::Vertex)
					{
						// vertex Conflict is defined as follows
						// as per paper, new node is initialized with empty solution
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
							n->parent->m_solution, use_XG_A, use_XG_A_H, use_S_A, isOverTime);
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
							open_heap_.emplace(n);
							n->updateIdx(nodeIdx);
							nodeIdx++;
							current->children.push_back(n);
							treeSize ++;
						}
					}
					else if (c->type == Conflict::Edge)
					{
						// as per paper, new node is initialized with empty solution
						conflictNode *n = new conflictNode(empty);
						// new node branches from current
						n->parent = current;
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
							n->parent->m_solution, use_XG_A, use_XG_A_H, use_S_A, isOverTime);
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
							open_heap_.emplace(n);
							n->updateIdx(nodeIdx);
							nodeIdx++;
							current->children.push_back(n);
							treeSize ++;
						}
					}
					else if (c->type == Conflict::Explanation)
					{
						// for each explanation bound, we need to make a child node
						// and give it a vertex constraint and solve again
						conflictNode *n = new conflictNode(empty);
						// new node branches from current
						n->parent = current;
						if (a == 0)
						{
							// make sure we do not add a constraint 
							//that an agent cannot get to goal
							if (m_env->getGoals()[c->agent1]->x == c->x1 && 
								m_env->getGoals()[c->agent1]->y == c->y1)
							{
								continue;
							}
							else
							{
								VertexConstraint *v = new VertexConstraint(c->agent1, 
									c->time1, c->x1, c->y1);
								n->m_constraint.add(v);
							}
						}
						else
						{
							if (m_env->getGoals()[c->agent2]->x == c->x2 && 
								m_env->getGoals()[c->agent2]->y == c->y2)
							{
								continue;
							}
							else
							{
								VertexConstraint *v = new VertexConstraint(c->agent2, 
									c->time2, c->x2, c->y2);
								n->m_constraint.add(v);
							}
						}

						if (n->m_constraint.getVertexConstraint() != nullptr)
						{
							std::vector<Constraint*> constriants;
							conflictNode *currNode = n;

							while (currNode != nullptr)
							{
								constriants.push_back(&(currNode->m_constraint));
								currNode = currNode->parent;
							}

							auto t1 = std::chrono::high_resolution_clock::now();
							n->m_solution = lowLevelSearch(startStates, constriants, 
								n->parent->m_solution, use_XG_A, use_XG_A_H, use_S_A, isOverTime);
							auto t2 = std::chrono::high_resolution_clock::now();
							auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
							timeAstar = timeAstar + duration.count();
						}
	
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
							open_heap_.emplace(n);
							n->updateIdx(nodeIdx);
							nodeIdx++;
							current->children.push_back(n);
							treeSize ++;
						}
					}
				}
			}
		}
	}
	updateCompTime(solveTime_);
	printf("%s: No solution found in %0.1f seconds.\n", "XG-CBS", solveTime_);
	timeThread.join();
	return isSolved;
}
