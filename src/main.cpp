#include <iostream>
#include <unordered_set>
#include <fstream>
#include <string>
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
		Solution notUsed;
		std::vector<State*> solution;
		std::vector<Constraint*> constraints;

		std::ofstream out("txt/solution.txt");
		int numSucc = 0;
		for (State *a : startStates)
		{
			bool success = planner->plan(a, solution, constraints, notUsed);

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

	else if (p == "expAstar" || p == "Exp-Astar" || p == "exp-Astar")
	{
		// get name of exisisting solution file
		// std::cout << inputYaml << std::endl;
		const int yamlSize = inputYaml.size();
		std::string inputSolution = "existingSolutions/" + 
			inputYaml.substr(5, yamlSize - (2 * 5)) + ".txt";

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
			// create instance of expA* and plan
			std::cout << "here" << std::endl;

		}
		else
		{
			std::cout << "Could not find file. Exiting..." << std::endl;
			exit(1);
		}




	}

	else if (p == "cbs" || p == "Cbs" || p == "CBS")
	{
		// CBS implementation!
		int costBound;
		std::cout << "Please enter an Explainability Bound: " << std::endl; 
		std::cin >> costBound;
		
		// init planner
		CBS *planner = new CBS(mapf, costBound);

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