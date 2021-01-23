#include <iostream>
#include <unordered_set>
#include <fstream>
#include "yaml.h"
#include "../includes/State.h"
#include "../includes/Environment.h"
#include "../includes/Astar.h"
#include "../includes/Cbs.h"


int main() { 
	YAML::Node config = YAML::LoadFile("yaml/mapf_swap4.yaml");  // path not perfect

	std::unordered_set<Location> obstacles;
	std::vector<Location> goals;
	std::vector<State> startStates;
	std::vector<std::string> agentNames;

	// parsing yaml now
	const auto& dim = config["map"]["dimensions"];
	int dimx = dim[0].as<int>();
	int dimy = dim[1].as<int>();

	// generate set of obstacles as a location
  	for (const auto& node : config["map"]["obstacles"])
  	{
		obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
  	}

  	// for each agent, add a start state and goal location
	for (const auto& node : config["agents"])
	{
		const auto& name = node["name"];
		const auto& start = node["start"];
		const auto& goal = node["goal"];
    	startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
    	goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
    	agentNames.emplace_back(name.as<std::string>());
	}

	// create environment object
	Environment *mapf = new Environment(dimx, dimy, obstacles, goals);

	// // A* implementation!!

	// // init planner
	// A_star *planner = new A_star(mapf);

	// // init solution and empty constraints
	// std::vector<State> solution;
	// std::vector<Constraint*> constraints;

	// std::ofstream out("txt/solution.txt");
	// for (const State &a : startStates)
	// {
	// 	bool success = planner->plan(a, solution, constraints);

	// 	if (success)
	// 	{
	// 		int it = std::distance(startStates.begin(), 
	// 			std::find(startStates.begin(), startStates.end(), a));
 //    		out << agentNames[it] << std::endl;
 //    		for (State& st: solution) 
 //    		{
 //    		 	out << st << std::endl;
 //    		}
	// 	}
	// 	mapf->updateAgent();
	// }

	// CBS implementation!

	// init planner
	CBS *planner = new CBS(mapf);

	// init solution
	Solution solution;

	// plan
	bool success = planner->plan(startStates, solution);

	if (success)
	{
		std::cout << solution.size() << std::endl;
		std::ofstream out("txt/solution.txt");
		for (std::vector<State> agentSol: solution)
		{
			int it = std::distance(solution.begin(), 
				std::find(solution.begin(), solution.end(), agentSol));
			out << agentNames[it] << std::endl;
			for (State st: agentSol)
			{
				out << st << std::endl;
			}
		}
	}

    return 0;
}