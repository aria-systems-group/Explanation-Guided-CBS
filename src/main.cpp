#include <iostream>
#include <unordered_set>
#include <fstream>
#include "yaml.h"
#include "../includes/State.h"
#include "../includes/Environment.h"
#include "../includes/Astar.h"
#include "../includes/Cbs.h"


int main() { 
	YAML::Node config = YAML::LoadFile("yaml/input.yaml");  // path not perfect

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

	// A* implementation!!

	// // init planner
	// A_star *planner = new A_star(mapf);

	// // init solution
	// std::vector<State> solution;

	// std::ofstream fout("yaml/solution.yaml");
	// for (const State &a : startStates)
	// {
	// 	bool success = planner->plan(a, solution);

	// 	if (success)
	// 	{
	// 		YAML::Node node;  // starts out as null
	// 		int it = std::distance(startStates.begin(), std::find(startStates.begin(), startStates.end(), a));

	// 		node["name"] = "agent" + std::to_string(it);  // it now is a map node

	// 		fout << node << std::endl;
	// 		for (const State &st : solution)
	// 		{
	// 			// emitter << st;
	// 			YAML::Node node;
	// 			if (st.time != 0)
	// 			{
	// 				node["time" + std::to_string(st.time)] = "(" + std::to_string(st.x) + ", " + std::to_string(st.y) + ")";
	// 				fout << node << std::endl;
	// 			}
	// 		}
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





    return 0;
}