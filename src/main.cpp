// my includes
#include "../includes/Benchmark.h"
#include "../includes/Cbs.h"
#include <iostream>
#include <string>


int main(int argc, char** argv) { 
	
	// To Plan: <executable> Plan <High-Level Algorithm> 
	// 				<Low-Level Algorithm> <filename>.yaml 
	// 				<computation time> <explanation cost (if applicable)>
	//				<% explanation cost> (this is temporary)

	// To Benchmark 1 file: <executable> Benchmark <filename>.yaml <computation time> <result>.csv <% explanation cost> (this is temporary)
	// To Benchmark Many Files: <executable> MultiBenchmark <filename>.yaml <computation time> <result>.csv

	if (argc >= 4)
	{
		const std::string expType(argv[1]);  // {Plan, Benchmark, Multi-Benchmark}
		
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
			else if (cbsType == "XG-CBS" && argc == 8)
			{
				const int costBound = atoi(argv[6]);  // cost bound
				const double percent_Explanation = atof(argv[7]);

				XG_CBS *planner = new XG_CBS(mapf, costBound, percent_Explanation);
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
			const std::string inputYaml(argv[2]);  // <path/fileName>.yaml
			const double planningTime = atof(argv[3]);  // real number > 0
			const std::string resultName(argv[4]);
			const double percent_Explanation = atof(argv[5]);

			// create environment from yaml file (assumed to be a file)
			Environment *mapf = yaml2env(inputYaml);

			std::string dir2file = inputYaml.substr(inputYaml.rfind("/")+1);
			const std::string::size_type end = dir2file.find(".yaml");
			const std::string mapName = dir2file.erase(end, dir2file.length());

			mapf->setMapName(mapName);

			std::vector<std::pair <std::string, std::vector<std::string>> > data = 
				singleMapBenchmark(mapf, planningTime, percent_Explanation);
			write_csv(resultName, data);
		}
		else if (expType == "MultiBenchmark")
		{
			const std::string inputYaml(argv[2]);  // path/to/*.yaml
			const double planningTime = atof(argv[3]);  // real number > 0
			const std::string resultName(argv[4]);
			const double percent_Explanation = atof(argv[5]);
			std::vector<std::pair <std::string, std::vector<std::string>> > data = 
				multiMapBenchmark(inputYaml, planningTime, percent_Explanation);
			write_csv(resultName, data);
		}
		else
			printf("Terminating prematurely due to invalid arguments.\n");
	}
	else
		printf("Terminating prematurely due to invalid arguments.\n");
    return 0;
}