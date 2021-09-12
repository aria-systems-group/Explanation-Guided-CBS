// my includes
#include "../includes/State.h"
#include "../includes/Environment.h"
#include "../includes/Astar.h"
#include "../includes/Cbs.h"
#include "../includes/XG-Astar-H.h"
#include "../includes/XG-Astar.h"
#include "../includes/XG-CBS.h"
// standard includes
#include <iostream>
#include <unordered_set>
#include <fstream>
#include <string>
#include "yaml.h"


int main(int argc, char** argv) { 
	
	std::string p = argv[1];

	std::string inputYaml = argv[2];
	std::string dir2file = inputYaml.substr(inputYaml.rfind("/")+1);
	std::string fileType = ".yaml";
	std::string::size_type end = dir2file.find(fileType);
	std::string output_name = "txt/" + dir2file.erase(end, fileType.length()) + "_sol.txt";

	YAML::Node config = YAML::LoadFile(inputYaml);  // path not perfect

	std::unordered_set<Location*> obstacles;
	std::vector<Location*> goals;
	std::vector<State*> startStates;
	std::vector<std::string> agentNames;

	// parsing yaml
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
	Environment *mapf = new Environment(dimx, dimy, obstacles, goals, agentNames);

	if (p == "CBS" || p == "cbs" || p == "Cbs")
	{
		// CBS implementation!
		// init planner
		CBS *planner = new CBS(mapf);

		// init solution
		Solution solution;

		// plan
		bool success = planner->plan(startStates, solution);

		if (success)
		{
			std::ofstream out(output_name);
			std::cout << "Outputting Solution to: " << output_name << std::endl;
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
			std::cout << "Done!" << std::endl;
		}
	}
	else if (p == "XG-CBS" || p == "XG-cbs" || p == "Xg-Cbs" || p == "xg-CBS")
	{
		// Exp-CBS implementation
		int costBound;
		bool useXG;
		bool useHeuristics;
		std::string ans2;
		std::string ans3;
		std::cout << "Please enter an Explainability Bound: "; 
		std::cin >> costBound;

		// init planner and solution
		XG_CBS *planner = new XG_CBS(mapf, costBound);
		Solution solution;

		// get desired behavior set up and plan
		std::cout << "Would you like to use EG-A*? [y/n]: ";
		std::cin >> ans2;

		if (ans2 == "y")
		{
			useXG = true;
			std::cout << "Would you like to use heuristics in EG-A*? [y/n]: ";
			std::cin >> ans3;
			
			if (ans3 == "y")
				useHeuristics = true;
			else if (ans3 == "n")
				useHeuristics = false;
			else
			{
				std::cout << "Invalid Response. Terminating Program." << std::endl;
				exit(1);
			}

			// plan
			bool success = planner->plan(startStates, solution, useXG, useHeuristics);
			if (success)
			{
				std::cout << "Successful planning using EG-CBS" << std::endl;
				std::ofstream out(output_name);
				std::cout << "Outputting Solution to: " << output_name << std::endl;
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
		else if (ans2 == "n")
		{
			useXG = false;
			useHeuristics = false;
			// plan
			bool success = planner->plan(startStates, solution, useXG, useHeuristics);
			if (success)
			{
				std::cout << "Successful planning using EG-CBS" << std::endl;
				std::ofstream out(output_name);
				std::cout << "Outputting Solution to: " << output_name << std::endl;
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
		else
		{
			std::cout << "Setting Up Planner.. Please review your responses and try again." << std::endl;
			exit(1);
		}
	}
	else
	{
		std::cout << "Invalid Response. Terminating Prematurely." << std::endl;
	}

    return 0;
}