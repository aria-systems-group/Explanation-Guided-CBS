#include "../includes/Cbs.h"


CBS::CBS(Environment *env): m_env(env)
{
	m_planner = new A_star(m_env);
};

Solution CBS::lowLevelSearch(const std::vector<State>& startStates, Constraints *c)
{
	Solution sol;
	std::vector<State> singleSol;

	for (const State &st : startStates)
	{	
		bool success = m_planner->plan(st, singleSol);
		// if sol found, add it --- else, return no solution
		if (success)
			sol.push_back(singleSol);
		
		m_planner->getEnv()->updateAgent();
	}
	// wrap environment idx back to first agent
	m_planner->getEnv()->updateAgent();
	return sol;
}

bool CBS::plan(const std::vector<State>& startStates, Solution& solution)
{

	solution.clear();

	// init open min-heap
	std::priority_queue <conflictNode*, std::vector<conflictNode*>, myconflictComparator > open_heap;
	// used for seeing if a node is in the heap
	std::unordered_set<State, std::hash<State>> m_open_list;

	
	Solution rootSol = lowLevelSearch(startStates, nullptr);
	
	conflictNode *rootNode = new conflictNode(rootSol);

	open_heap.emplace(rootNode);

	



	return 0;
}
