// my includes
#include "../includes/Benchmark.h"
#include "../includes/Cbs.h"
#include <iostream>
#include <string>


int main(int argc, char** argv) { 
	
	// arguments (in order) <executable> <expType> <High-Level Algorithm> <Low-Level Algorithm> <filename>.yaml <computation time> <explanation cost>

	if (argc >= 6)
	{
		const std::string expType(argv[1]);  // {Plan, Benchmark, Multi-Benchmark}
		const std::string cbsType(argv[2]);  // {CBS, XG-CBS}
		const std::string aStarType(argv[3]);  // {A, XG-A, XG-A-H}
		const std::string inputYaml(argv[4]);  // {<path/fileName>.yaml, <path/to/multiple/yaml/files>}
		const double planningTime = atof(argv[5]);  // { real number > 0 }

		if ( (cbsType == "XG-CBS" && argc == 7) || (cbsType == "CBS" && argc == 6))
		{
			if (expType == "Plan")
			{
				// create environment from yaml file (assumed to be a file)
				Environment *mapf = yaml2env(inputYaml);

				std::string dir2file = inputYaml.substr(inputYaml.rfind("/")+1);
				const std::string::size_type end = dir2file.find(".yaml");
				const std::string mapName = dir2file.erase(end, dir2file.length());

				mapf->setMapName(mapName);

				// init solution (to be filled by planning)
				Solution solution;
				bool success = false;

				if (cbsType == "CBS" && aStarType == "A")
				{
					CBS *planner = new CBS(mapf);
					planner->setSolveTime(planningTime);
					success = planner->plan(mapf->getStarts(), solution);
				}
				else if (cbsType == "XG-CBS")
				{
					const int costBound = atoi(argv[6]);  // computation time
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
					
					const std::string output_name = "txt/" + mapName + "_sol.txt";
					std::ofstream out(output_name);
					std::cout << "Outputting Solution to: " << output_name << std::endl;
					for (std::vector<State*> agentSol: solution)
					{
						int it = std::distance(solution.begin(), 
							std::find(solution.begin(), solution.end(), agentSol));
						out << mapf->getAgentNames()[it] << std::endl;
						for (State *st: agentSol)
							out << *st << std::endl;
					}
				}
			}
			else if (expType == "Benchmark")
			{
				// create environment from yaml file (assumed to be a file)
				Environment *mapf = yaml2env(inputYaml);

				std::string dir2file = inputYaml.substr(inputYaml.rfind("/")+1);
				const std::string::size_type end = dir2file.find(".yaml");
				const std::string mapName = dir2file.erase(end, dir2file.length());

				mapf->setMapName(mapName);

				if (cbsType == "XG-CBS")
				{
					const int costBound = atoi(argv[6]);  // computation time
					if (aStarType == "XG-A-H")
					{
						std::vector<std::pair <std::string, std::vector<std::string>> > data = singleMapBenchmark(mapf, costBound, planningTime);
						write_csv("XG_CBS_Benchmark.csv", data);

					}
					else
						printf("No benchmark function for %s.\n", aStarType.c_str());
				}
				else
					printf("No benchmark function for %s.\n", cbsType.c_str());
			}
			else if (expType == "MultiBenchmark")
			{
				if (cbsType == "XG-CBS")
				{
					const int costBound = atoi(argv[6]);  // computation time
					if (aStarType == "XG-A-H")
					{
						std::vector<std::pair <std::string, std::vector<std::string>> > data = multiMapBenchmark(inputYaml, costBound, planningTime);
						write_csv("XG_CBS_Benchmark.csv", data);
					}
					else
						printf("No multi-benchmark function for %s.\n", aStarType.c_str());
				}
				else
					printf("No multi-benchmark function for %s.\n", aStarType.c_str());
			}
			else
				printf("Terminating prematurely due to invalid arguments.\n");
		}
		else
			printf("Terminating prematurely due to invalid arguments.\n");
	}
	else
		printf("Terminating prematurely due to invalid arguments.\n");
    return 0;
}