// my includes
#include "../includes/Benchmark.h"
#include "../includes/Cbs.h"
#include <iostream>
#include <string>


int main(int argc, char** argv) { 
	
	/*
	Tool Options:
		<executable> == Planner
		<High-Level Algorithm> = {CBS, XG-CBS}
		<Low-Level Algorithm> = {A, XG-A, XG-A-H, S-A}
		<explanation cost> = decimal in range of [0, 1]
			Note: only required when using XG-A-H

	To Plan: <executable> Plan <High-Level Algorithm> 
 		<Low-Level Algorithm> <filename>.yaml 
 		<computation time> <explanation cost (only for XG-CBS)>
 		<weight on exp. cost (only for XG-A-H)>

	To Benchmark 1 file: <executable> Benchmark 
		<Low-Level Algorithm> <filename>.yaml 
		<computation time> <result>.csv <weight on exp. cost (only for XG-A-H)>

	To Benchmark Many Files: <executable> MultiBenchmark 
		<Low-Level Algorithm> <directory> 
		<computation time> <result>.csv <weight on exp. cost (only for XG-A-H)>

	*/

	if (argc >= 4)
	{
		const std::string expType(argv[1]);  // {Plan, Benchmark, Multi-Benchmark, Match}
		
		if (expType == "Plan")
		{
			const std::string cbsType(argv[2]);  // {CBS, XG-CBS}
			const std::string aStarType(argv[3]);  // {A, XG-A, XG-A-H}
			const std::string inputYaml(argv[4]);  // <path/fileName>.yaml
			const double planningTime = atof(argv[5]);  // { real number > 0 }

			// create environment from yaml file (assumed to be a file)
			Environment *mapf = yaml2env(inputYaml);

			std::string dir2file = inputYaml.substr(inputYaml.rfind("/")+1);
			const std::string::size_type end = dir2file.find(".yaml");
			const std::string mapName = dir2file.erase(end, dir2file.length());

			mapf->setMapName(mapName);

			// init solution (to be filled by planning)
			Solution solution;
			bool success = false;

			if (cbsType == "CBS" && argc == 6)
			{
				if (aStarType == "A")
				{
					CBS *planner = new CBS(mapf);
					planner->setSolveTime(planningTime);
					success = planner->plan(mapf->getStarts(), solution);
				}
				else
					printf("Terminating prematurely due to invalid arguments.\n");
			}
			else if (cbsType == "XG-CBS")
			{
				const int costBound = atoi(argv[6]);  // cost bound
				double percent_Explanation = 0.0;

				if (aStarType == "A" && argc == 7)
				{
					XG_CBS *planner = new XG_CBS(mapf, costBound, percent_Explanation);
					planner->setSolveTime(planningTime);
					success = planner->plan(mapf->getStarts(), solution, false, false, false);
				}
				else if (aStarType == "XG-A" && argc == 7)
				{
					XG_CBS *planner = new XG_CBS(mapf, costBound, percent_Explanation);
					planner->setSolveTime(planningTime);
					success = planner->plan(mapf->getStarts(), solution, true, false, false);
				}
				else if (aStarType == "XG-A-H" && argc == 8)
				{
					percent_Explanation = atof(argv[7]);
					XG_CBS *planner = new XG_CBS(mapf, costBound, percent_Explanation);
					planner->setSolveTime(planningTime);
					success = planner->plan(mapf->getStarts(), solution, false, true, false);
				}
				else if (aStarType == "S-A")
				{
					XG_CBS *planner = new XG_CBS(mapf, costBound, percent_Explanation);
					planner->setSolveTime(planningTime);
					success = planner->plan(mapf->getStarts(), solution, false, false, true);
				}	
				else
					printf("Terminating prematurely due to invalid low-level planner.\n");
			}
			else
				printf("Terminating prematurely due to invalid high-level planner.\n");
			
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
			if (argc >= 6)
			{
				const std::string aStarType(argv[2]);
				const std::string inputYaml(argv[3]);  // <path/fileName>.yaml
				const double planningTime = atof(argv[4]);  // real number > 0
				const std::string resultName(argv[5]);
				double percent_Explanation = 0.0;
				if (aStarType == "XG-A-H")
					percent_Explanation = atof(argv[6]);
				// create environment from yaml file (assumed to be a file)
				Environment *mapf = yaml2env(inputYaml);

				std::string dir2file = inputYaml.substr(inputYaml.rfind("/")+1);
				const std::string::size_type end = dir2file.find(".yaml");
				const std::string mapName = dir2file.erase(end, dir2file.length());

				mapf->setMapName(mapName);

				std::vector<std::pair <std::string, std::vector<std::string>> > data = 
					singleMapBenchmark(mapf, planningTime, aStarType, percent_Explanation);
				write_csv(resultName, data);
			}
			else
				printf("Terminating prematurely due to invalid arguments for bemchmarking.\n");
		}
		else if (expType == "MultiBenchmark")
		{
			if (argc >= 6)
			{
				const std::string aStarType(argv[2]);
				const std::string inputYaml(argv[3]);  // path/to/*.yaml
				const double planningTime = atof(argv[4]);  // real number > 0
				const std::string resultName(argv[5]);
				double percent_Explanation = 0.0;
				if (aStarType == "XG-A-H")
					percent_Explanation = atof(argv[6]);

				std::vector<std::pair <std::string, std::vector<std::string>> > data = 
					multiMapBenchmark(inputYaml, planningTime, aStarType, percent_Explanation);
				write_csv(resultName, data);
			}
			else
				printf("Terminating prematurely due to invalid arguments for multi-bemchmarking.\n");
		}
		// else if (expType == "Match")
		// {
		// 	const std::string inputYaml(argv[2]);  // <path/fileName>.yaml
		// 	const double planningTime = atof(argv[3]);  // real number > 0
		// 	const std::string resultName(argv[4]);
		// 	const double percent_Explanation = atof(argv[5]);

		// 	// create environment from yaml file (assumed to be a file)
		// 	Environment *mapf = yaml2env(inputYaml);

		// 	std::string dir2file = inputYaml.substr(inputYaml.rfind("/")+1);
		// 	const std::string::size_type end = dir2file.find(".yaml");
		// 	const std::string mapName = dir2file.erase(end, dir2file.length());

		// 	mapf->setMapName(mapName);

		// 	std::vector<std::pair <std::string, std::vector<std::string>> > data = 
		// 		singleCostMatch(mapf, planningTime, percent_Explanation);
		// 	write_csv(resultName, data);
		// }
		// else if (expType == "MultiMatch")
		// {
		// 	const std::string inputYaml(argv[2]);  // path/to/*.yaml
		// 	const double planningTime = atof(argv[3]);  // real number > 0
		// 	const std::string resultName(argv[4]);
		// 	const double percent_Explanation = atof(argv[5]);
		// 	std::vector<std::pair <std::string, std::vector<std::string>> > data = 
		// 		multiCostMatch(inputYaml, planningTime, percent_Explanation);
		// 	write_csv(resultName, data);
		// }
		else
			printf("Terminating prematurely due to invalid experiment type.\n");
	}
	else
		printf("Please provide inputs to tool as described in the instructions.\n");
    return 0;
}