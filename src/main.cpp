#include <iostream>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include "yaml.h"
#include "../includes/State.h"
#include "../includes/Location.h"
#include "../includes/Environment.h"


int main() {
	YAML::Node config = YAML::LoadFile("yaml/input.yaml");

	std::unordered_set<Location> obstacles;
	std::vector<Location> goals;
	std::vector<State> startStates;


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


	Environment mapf(dimx, dimy, obstacles, goals);

	// State *st;
	// st->setX(5);
	// st->setY(5);
	// std::cout << "State address: " << &st << std::endl;
	// std::cout << "State point:";
	// st->printState(std::cout);
    return 0;
}