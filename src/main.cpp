// my includes
#include "../includes/Astar.h"
#include "../includes/Cbs.h"
#include "../includes/XG-Astar-H.h"
#include "../includes/XG-Astar.h"
#include "../includes/XG-CBS.h"
#include "../includes/parseYAML.h"
#include <iostream>
#include <string>


int main(int argc, char** argv) { 
	
	// arguments (in order) <executable> <High-Level Algorithm> <Low-Level Algorithm> <filename>.yaml <computation time>

	if (argc >= 5)
	{
		const std::string cbsType(argv[1]);  // {CBS, XG-CBS}
		const std::string aStarType(argv[2]);  // {A, XG-A, XG-A-H}
		const std::string inputYaml(argv[3]);
		const double planningTime = atof(argv[4]);  // computation time

		if ( (cbsType == "XG-CBS" && argc == 6) || (cbsType == "CBS" && argc == 5))
		{
			// create environment from yaml file
			Environment *mapf = yaml2env(inputYaml);
		
			// init solution (to be filled by planning)
			Solution solution;
			bool success;

			if (cbsType == "CBS" && aStarType == "A")
			{
				CBS *planner = new CBS(mapf);
				planner->setSolveTime(planningTime);
				success = planner->plan(mapf->getStarts(), solution);
			}
			else if (cbsType == "XG-CBS")
			{
				const int costBound = atoi(argv[5]);  // computation time
				XG_CBS *planner = new XG_CBS(mapf, costBound);
				planner->setSolveTime(planningTime);
				// last two args are "useXG", and "useHeuristics"
				if (aStarType == "A")
					success = planner->plan(mapf->getStarts(), solution, false, false);
				else if (aStarType == "XG-A")
					success = planner->plan(mapf->getStarts(), solution, true, false);
				else if (aStarType == "XG-A-H")
					success = planner->plan(mapf->getStarts(), solution, true, true);
				else
					printf("Terminating prematurely due to invalid arguments.\n");
			}
			else
				printf("Terminating prematurely due to invalid arguments.\n");

			// we have planned, now output solution to file if applicable
			if (success)
			{
				std::string dir2file = inputYaml.substr(inputYaml.rfind("/")+1);
				std::string fileType = ".yaml";
				std::string::size_type end = dir2file.find(fileType);
				std::string output_name = "txt/" + dir2file.erase(end, dir2file.length()) + "_sol.txt";
				std::ofstream out(output_name);
				std::cout << "Outputting Solution to: " << output_name << std::endl;
				for (std::vector<State*> agentSol: solution)
				{
					int it = std::distance(solution.begin(), 
						std::find(solution.begin(), solution.end(), agentSol));
					out << mapf->getAgentNames()[it] << std::endl;
					for (State *st: agentSol)
					{
						out << *st << std::endl;
					}
				}
			}
		}
		else
			printf("Terminating prematurely due to invalid arguments.\n");
	}
	else
		printf("Terminating prematurely due to invalid arguments.\n");
    return 0;
}