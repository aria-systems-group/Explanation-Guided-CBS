// my includes
#include "../includes/State.h"
#include "../includes/Environment.h"
#include "../includes/Astar.h"
#include "../includes/Cbs.h"
#include "../includes/EG-Astar-H.h"
#include "../includes/EG-Astar.h"
#include "../includes/EG-CBS.h"
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

	if (p == "EG-astar" || p == "EG-Astar")
	{
		// EG-A*-H implementation
		// get name of exisisting solution file
		const int yamlSize = inputYaml.size();
		std::string inputSolution = "existingSolutions/" + 
			inputYaml.substr(5, yamlSize - (2 * 5)) + ".txt";
			std::cout << "Searching for partial solution file: " << inputSolution << std::endl;
		// put solution into format that expAstar expects
		std::ifstream SolutionFile;
		SolutionFile.open(inputSolution);
		if (SolutionFile.is_open())
		{
			int numAgents = 0;
			std::string line;
			// count number of agents
			while (std::getline(SolutionFile, line))
			{
				if (line.substr(0, 5) == "agent")
					numAgents ++;	
			}
			Solution existing(numAgents);
			std::ifstream solReader;
			solReader.open(inputSolution);
			std::string test;
			// no need to check if file is open this time
			// already did that
			std::string l;
			std::getline(solReader, l); // agent0
			for (int i = 0; i < numAgents; i++)
			{
				// going to append states to existing
				std::getline(solReader, l); // initial state
				while (l.substr(0, 5) != "agent" && !solReader.eof())
				{
					// get state on on l
					// // cycle through l to get t, x, y
					int t, x, y;
					bool xFound = false, yFound = false;
					int bg = 0; int sz = 1;
					for (int i = 0; i < l.length(); i++)
					{
						std::string s = l.substr(bg, sz);
						// std::cout << s << std::endl;
						if (s.back() == ':')
						{
							t = std::stoi(s);
							bg = l.find(s.back()) + 2;
							sz = 1;
							// std::cout << "found t: " << t << std::endl;
						}
						else if (s.back() == ',' && !xFound && !yFound)
						{
							x = std::stoi(s);
							bg = bg + sz;
							sz = 1; xFound = true;
							// std::cout << "found x: " << x << std::endl;
						}
						else if (s.back() == ',' && xFound && !yFound)
						{
							y = std::stoi(s);
							bg = bg + sz;
							sz = 1; yFound = true;
							// std::cout << "found y: " << y << std::endl;
						}
						else
							sz ++;
					}
					State *st = new State(t, x, y);
					existing[i].push_back(st);
					std::getline(solReader, l);
				}
			}
			// now we have exisisting solution in the form we need
			// currently only plans for final agent
			while (mapf->getAgent() != numAgents)
			{
				mapf->updateAgent();
			}
			// initialize EG-A* and plan
			EG_Astar *planner = new EG_Astar(mapf, false); // boolean says EG-CBS not involved
			std::ofstream out(output_name);
			std::vector<State*> solution;
			std::vector<Constraint*> constraints;
			bool success = planner->plan(startStates[numAgents], solution, constraints, existing);

			if (success)
			{
				std::cout << "Outputting Solution to: " << output_name << std::endl;
				for (std::vector<State*> agentSol: existing)
				{
					int it = std::distance(existing.begin(), 
						std::find(existing.begin(), existing.end(), agentSol));
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
			std::cout << "Could not find existing solution file. Exiting without solution" << std::endl;
			exit(1);
		}
	}
	else if (p == "EG-astar-H" || p == "EG-Astar-H")
	{
		// EG-A*-H implementation
		// get name of exisisting solution file
		const int yamlSize = inputYaml.size();
		std::string inputSolution = "existingSolutions/" + 
			inputYaml.substr(5, yamlSize - (2 * 5)) + ".txt";
			std::cout << "Searching for partial solution file: " << inputSolution << std::endl;
		// put solution into format that expAstar expects
		std::ifstream SolutionFile;
		SolutionFile.open(inputSolution);
		if (SolutionFile.is_open())
		{
			int numAgents = 0;
			std::string line;
			// count number of agents
			while (std::getline(SolutionFile, line))
			{
				if (line.substr(0, 5) == "agent")
					numAgents ++;	
			}
			Solution existing(numAgents);
			std::ifstream solReader;
			solReader.open(inputSolution);
			std::string test;
			// no need to check if file is open this time
			// already did that
			std::string l;
			std::getline(solReader, l); // agent0
			for (int i = 0; i < numAgents; i++)
			{
				// going to append states to existing
				std::getline(solReader, l); // initial state
				while (l.substr(0, 5) != "agent" && !solReader.eof())
				{
					// get state on on l
					// // cycle through l to get t, x, y
					int t, x, y;
					bool xFound = false, yFound = false;
					int bg = 0; int sz = 1;
					for (int i = 0; i < l.length(); i++)
					{
						std::string s = l.substr(bg, sz);
						// std::cout << s << std::endl;
						if (s.back() == ':')
						{
							t = std::stoi(s);
							bg = l.find(s.back()) + 2;
							sz = 1;
							// std::cout << "found t: " << t << std::endl;
						}
						else if (s.back() == ',' && !xFound && !yFound)
						{
							x = std::stoi(s);
							bg = bg + sz;
							sz = 1; xFound = true;
							// std::cout << "found x: " << x << std::endl;
						}
						else if (s.back() == ',' && xFound && !yFound)
						{
							y = std::stoi(s);
							bg = bg + sz;
							sz = 1; yFound = true;
							// std::cout << "found y: " << y << std::endl;
						}
						else
							sz ++;
					}
					State *st = new State(t, x, y);
					existing[i].push_back(st);
					std::getline(solReader, l);
				}
			}
			// now we have exisisting solution in the form we need
			// currently only plans for final agent
			while (mapf->getAgent() != numAgents)
			{
				mapf->updateAgent();
			}
			// initialize EG-A*-H and plan
			EG_Astar_H *planner = new EG_Astar_H(mapf, false); // boolean says EG-CBS not involved
			std::ofstream out(output_name);
			std::vector<State*> solution;
			std::vector<Constraint*> constraints;
			bool success = planner->plan(startStates[numAgents], solution, constraints, existing);

			if (success)
			{
				std::cout << "Outputting Solution to: " << output_name << std::endl;
				for (std::vector<State*> agentSol: existing)
				{
					int it = std::distance(existing.begin(), 
						std::find(existing.begin(), existing.end(), agentSol));
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
			std::cout << "Could not find existing solution file. Exiting without solution" << std::endl;
			exit(1);
		}
	}
	else if (p == "cbs" || p == "Cbs" || p == "CBS")
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
	else if (p == "EG-cbs" || p == "EG-CBS" || p == "Eg-Cbs" || p == "eg-CBS")
	{
		// Exp-CBS implementation
		int costBound;
		bool verbose;
		bool useEG;
		bool useHeuristics;
		std::string ans1;
		std::string ans2;
		std::string ans3;
		std::cout << "Please enter an Explainability Bound: "; 
		std::cin >> costBound;

		// init planner and solution
		EG_CBS *planner = new EG_CBS(mapf, costBound);
		Solution solution;

		// get desired behavior set up and plan
		std::cout << "Would you like to output intermediate solutions? [y/n]: "; 
		std::cin >> ans1;
		std::cout << "Would you like to use EG-A*? [y/n]: ";
		std::cin >> ans2;

		if (ans1 == "y")
			verbose = true;
		else if (ans1 == "n")
			verbose = false;
		else
		{
			std::cout << "Invalid Response. Terminating Program." << std::endl;
			exit(1);
		}

		if (ans2 == "y")
		{
			useEG = true;
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
			bool success = planner->plan(startStates, solution, useEG, useHeuristics);
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
			useEG = false;
			useHeuristics = false;
			// plan
			bool success = planner->plan(startStates, solution, useEG, useHeuristics);
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

    return 0;
}