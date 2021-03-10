#include <iostream>
#include <unordered_set>
#include <fstream>
#include "yaml.h"
#include "../includes/State.h"
#include "../includes/Environment.h"
#include "../includes/Astar.h"
#include "../includes/Cbs.h"


int main(int argc, char** argv) { 
	
	std::string p = argv[1];

	std::string inputYaml = argv[2];


	YAML::Node config = YAML::LoadFile(inputYaml);  // path not perfect

	std::unordered_set<Location*> obstacles;
	std::vector<Location*> goals;
	std::vector<State*> startStates;
	std::vector<std::string> agentNames;

	// parsing yaml now
	const auto& dim = config["map"]["dimensions"];
	int dimx = dim[0].as<int>();
	int dimy = dim[1].as<int>();

	// generate set of obstacles as a location
  	for (const auto& node : config["map"]["obstacles"])
  	{
		obstacles.insert(new Location(node[0].as<int>(), node[1].as<int>()));
  	}

  	// for each agent, add a start state and goal location
	for (const auto& node : config["agents"])
	{
		const auto& name = node["name"];
		const auto& start = node["start"];
		const auto& goal = node["goal"];
    	startStates.emplace_back(new State(0, start[0].as<int>(), start[1].as<int>()));
    	goals.emplace_back(new Location(goal[0].as<int>(), goal[1].as<int>()));
    	agentNames.emplace_back(name.as<std::string>());
	}

	// create environment object
	Environment *mapf = new Environment(dimx, dimy, obstacles, goals);

	if (p == "Astar")
	{



		// A* implementation!!

		// init planner
		A_star *planner = new A_star(mapf);

		// init solution and empty constraints
		std::vector<State*> solution;
		std::vector<Constraint*> constraints;

		std::ofstream out("txt/solution.txt");
		int numSucc = 0;
		for (State *a : startStates)
		{
			bool success = planner->plan(a, solution, constraints);

			if (success)
			{
				int it = std::distance(startStates.begin(), 
					std::find(startStates.begin(), startStates.end(), a));
    			out << agentNames[it] << std::endl;
    			for (State *st: solution) 
    			{
    			 	out << *st << std::endl;
    			}
			}
			mapf->updateAgent();
			numSucc ++;
		}
		if (numSucc == startStates.size())
			std::cout << "Successful planning using A*" << std::endl;
	}

	else if (p == "cbs" || p == "Cbs" || p == "CBS")
	{
		// CBS implementation!
		// int costBound;
		// std::cout << "Please enter an Explainability Bound: " << std::endl; 
		// std::cin >> costBound;
		
		// init planner
		CBS *planner = new CBS(mapf);

		// init solution
		Solution solution;

		// plan
		bool success = planner->plan(startStates, solution);

		if (success)
		{
			std::cout << "Successful planning using CBS" << std::endl;
			std::ofstream out("txt/solution.txt");
			for (std::vector<State*> agentSol: solution)
			{
				int it = std::distance(solution.begin(), 
					std::find(solution.begin(), solution.end(), agentSol));
				out << agentNames[it] << std::endl;
				for (State *st: agentSol)
				{
					out << *st << std::endl;
				}
			}
		}

	}

    return 0;
}