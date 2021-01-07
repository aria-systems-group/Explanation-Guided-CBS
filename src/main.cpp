#include <iostream>
#include <unordered_set>
#include "yaml.h"
#include "../includes/State.h"
#include "../includes/Location.h"
#include "../includes/Environment.h"
#include "../includes/A_star.h"


int main() { 
	YAML::Node config = YAML::LoadFile("yaml/input.yaml");  // path not perfect

	std::unordered_set<Location> obstacles;
	std::vector<Location> goals;
	std::vector<State> startStates;

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
		const auto& start = node["start"];
		const auto& goal = node["goal"];
    	startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
    	goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
	}

	// create environment object
	Environment mapf(dimx, dimy, obstacles, goals);

	// init planner
	A_star planner(mapf);

	// init solution
	 PlanResult<State, Action, Cost> solution;

	// only plan for one agent using A* (for now)
	planner.plan(startStates[0], solution);




	
    return 0;
}