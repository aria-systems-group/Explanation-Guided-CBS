#include "../includes/Benchmark.h"


// https://www.gormanalysis.com/blog/reading-and-writing-csv-files-with-cpp/
void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<std::string>>> dataset)
{
    // Make a CSV file with one or more columns of integer values
    // Each column of data is represented by the pair <column name, column data>
    //   as std::pair<std::string, std::vector<int>>
    // The dataset is represented as a vector of these columns
    // Note that all columns should be the same size

    std::cout << "Writing Results to file." << std::endl;

    // Create an output filestream object
    std::ofstream myFile(filename);
    
    // Send column names to the stream
    for(int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j).first;
        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";

	if (!dataset.empty())
	{ 
    	// Send data to the stream
    	for(int i = 0; i < dataset.at(0).second.size(); ++i)
    	{
    	    for(int j = 0; j < dataset.size(); ++j)
    	    {
    	        myFile << dataset.at(j).second.at(i);
    	        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    	    }
    	    myFile << "\n";
    	}
	}
    // Close the file
    myFile.close();
}

std::vector<std::pair <std::string, std::vector<std::string>> > singleMapBenchmark(Environment* env, 
	const double maxCompTime, const std::string lowLevelAlg, const double perc_exp)
{
	printf("Preparing to benchmark %s.\n", env->getMapName().c_str());

	// init solution (to be filled by planning)
	Solution solution;

	// plan using CBS
	CBS *cbs = new CBS(env);
	cbs->setSolveTime(maxCompTime);
	bool success = cbs->plan(cbs->getEnv()->getStarts(), solution);

	if (success)
	{
		// init the columns of data
		std::vector<std::string> name{cbs->getEnv()->getMapName()};
		std::vector<std::string> agents{std::to_string(cbs->getEnv()->getAgentNames().size())};
		std::vector<std::string> grid{std::to_string( (cbs->getEnv()->getXdim() + 1) * (cbs->getEnv()->getYdim() + 1) )};
		std::vector<std::string> ComputationTimes{};
		std::vector<std::string> NumEvalNodes{};
		std::vector<std::string> Costs{};
		std::vector<std::string> PathLength{};
		std::vector<std::string> xgComputationTimes{};
		std::vector<std::string> xgNumEvalNodes{};
		std::vector<std::string> xgCosts{};
		std::vector<std::string> xgPathLength{};

		//cbs cost
		ComputationTimes.push_back(std::to_string(cbs->getCompTime()));
		// cbs number of evaluated nodes is equal to number of expanded nodes + 1 (the solution)
		NumEvalNodes.push_back(std::to_string(cbs->closed_set_.size()+1));
		// solution explanation cost and update bound
		Costs.push_back(std::to_string(cbs->getSolutionNode()->getSegCost()));
		// get SoC of solution
		PathLength.push_back(std::to_string(cbs->getSolutionNode()->calcCost()));
		// get beginning cost for xg-cbs
		int expCost = cbs->getSolutionNode()->getSegCost();

		// remove all cbs data
		cbs->clear();
		delete cbs;

		if (expCost >= 1)
		{
			// plan using XG-CBS
			XG_CBS *planner = new XG_CBS(env, expCost, perc_exp);
			planner->setSolveTime(maxCompTime);
			if (lowLevelAlg == "A")
				success = planner->plan(env->getStarts(), solution, false, false, false);
			else if (lowLevelAlg == "XG-A")
				success = planner->plan(env->getStarts(), solution, true, false, false);
			else if (lowLevelAlg == "XG-A-H")
				success = planner->plan(env->getStarts(), solution, false, true, false);
			else if (lowLevelAlg == "S-A")
				success = planner->plan(env->getStarts(), solution, false, false, true);
			else
				printf("Unable to benchmark do to invalid low-level algorithm.");

			// is successful, begin benchmarking loop
			while (success)
			{
				// computation time
				xgComputationTimes.push_back(std::to_string(planner->getCompTime()));
				// number of evaluated nodes is equal to number of expanded nodes + 1 (the solution)
				xgNumEvalNodes.push_back(std::to_string(planner->closed_set_.size()+1));
				// solution explanation cost and update bound
				xgCosts.push_back(std::to_string(planner->getSolutionNode()->getSegCost()));
				// get SoC
				xgPathLength.push_back(std::to_string(planner->getSolutionNode()->calcCost()));

				expCost = planner->getSolutionNode()->getSegCost() - 1;

				if (expCost >= 1)
				{
					// clear planner data
					planner->clear();

					// update planner
					delete planner;
					planner = new XG_CBS(env, expCost, perc_exp);
					planner->setSolveTime(maxCompTime);

					if (lowLevelAlg == "A")
						success = planner->plan(env->getStarts(), solution, false, false, false);
					else if (lowLevelAlg == "XG-A")
						success = planner->plan(env->getStarts(), solution, true, false, false);
					else if (lowLevelAlg == "XG-A-H")
						success = planner->plan(env->getStarts(), solution, false, true, false);
					else if (lowLevelAlg == "S-A")
						success = planner->plan(env->getStarts(), solution, false, false, true);
					else
						printf("Unable to benchmark do to invalid low-level algorithm.");
				}
				else
					success = false;
			}
		}
		// make map data same length as planning data
		if (xgComputationTimes.size() > 0)
		{
			for (int i=1; i < xgComputationTimes.size(); i++)
			{
				// map data
				name.push_back(std::string());
				agents.push_back(std::string());
				grid.push_back(std::string());
				// cbs data
				ComputationTimes.push_back(std::string());
				NumEvalNodes.push_back(std::string());
				Costs.push_back(std::string());
				PathLength.push_back(std::string());
			}
		}
		else
		{
			// failed to get solution, make xg-cbs data same length as map data
			xgComputationTimes.push_back("-");
			xgNumEvalNodes.push_back("-");
			xgCosts.push_back("-");
			xgPathLength.push_back("-");
		}
		// col 1: map name
		std::pair< std::string, std::vector<std::string> > mapName{"Map", name};
		// col 2: number of agents
		std::pair< std::string, std::vector<std::string> > numAgents{"# of Agents", agents};
		// col 3: grid size
		std::pair< std::string, std::vector<std::string> > size{"Grid Size", grid};
		// col 4: cbs comp time
		std::pair< std::string, std::vector<std::string> > times{"CBS Time (s)", ComputationTimes};
		// col 5: cbs evaluated nodes
		std::pair< std::string, std::vector<std::string> > trees{"CBS # of Evalidated Nodes", NumEvalNodes};
		// col 6: cbs exp costs
		std::pair< std::string, std::vector<std::string> > costs{"CBS Cost", Costs};
		// col 7: cbs SoC
		std::pair< std::string, std::vector<std::string> > SoC{"CBS SoC", PathLength};
		// col 8: xg-cbs computation time
		std::pair< std::string, std::vector<std::string> > xgTimes{"XG-CBS Time (s)", xgComputationTimes};
		// col 9: xg-cbs evaluated nodes
		std::pair< std::string, std::vector<std::string> > xgTrees{"XG-CBS # of Evalidated Nodes", xgNumEvalNodes};
		// col 10: xg-cbs exp costs
		std::pair< std::string, std::vector<std::string> > xgExps{"XG-CBS Cost", xgCosts};
		// col 11: xg-cbs SoC
		std::pair< std::string, std::vector<std::string> > xgSoC{"XG-CBS SoC", xgPathLength};

		// create dataset and return it
		std::vector<std::pair<std::string, std::vector<std::string>>> results = {mapName, numAgents, size, times, trees, costs, SoC, xgTimes, xgTrees, xgExps, xgSoC};
		return results;
	}
	else
	{
		// init the columns of data
		std::vector<std::string> name{cbs->getEnv()->getMapName()};
		std::vector<std::string> agents{std::to_string(cbs->getEnv()->getAgentNames().size())};
		std::vector<std::string> grid{std::to_string( (cbs->getEnv()->getXdim() + 1) * (cbs->getEnv()->getYdim() + 1) )};
		std::vector<std::string> ComputationTimes{};
		std::vector<std::string> NumEvalNodes{};
		std::vector<std::string> Costs{};
		std::vector<std::string> PathLength{};
		std::vector<std::string> xgComputationTimes{};
		std::vector<std::string> xgNumEvalNodes{};
		std::vector<std::string> xgCosts{};
		std::vector<std::string> xgPathLength{};

		//cbs cost
		ComputationTimes.push_back(std::to_string(cbs->getCompTime()));
		// cbs number of evaluated nodes is equal to number of expanded nodes + 1 (the solution)
		NumEvalNodes.push_back("-");
		// solution explanation cost and update bound
		Costs.push_back("-");
		// get SoC of solution
		PathLength.push_back("-");
		// get beginning cost for xg-cbs
		int expCost = 100000;

		// remove all cbs data
		cbs->clear();
		delete cbs;

		// plan using XG-CBS
		XG_CBS *planner = new XG_CBS(env, expCost, perc_exp);
		planner->setSolveTime(maxCompTime);
		if (lowLevelAlg == "A")
			success = planner->plan(env->getStarts(), solution, false, false, false);
		else if (lowLevelAlg == "XG-A")
			success = planner->plan(env->getStarts(), solution, true, false, false);
		else if (lowLevelAlg == "XG-A-H")
			success = planner->plan(env->getStarts(), solution, false, true, false);
		else if (lowLevelAlg == "S-A")
			success = planner->plan(env->getStarts(), solution, false, false, true);
		else
			printf("Unable to benchmark do to invalid low-level algorithm.");

		// is successful, begin benchmarking loop
		while (success)
		{
			// computation time
			xgComputationTimes.push_back(std::to_string(planner->getCompTime()));
			// number of evaluated nodes is equal to number of expanded nodes + 1 (the solution)
			xgNumEvalNodes.push_back(std::to_string(planner->closed_set_.size()+1));
			// solution explanation cost and update bound
			xgCosts.push_back(std::to_string(planner->getSolutionNode()->getSegCost()));
			// get SoC
			xgPathLength.push_back(std::to_string(planner->getSolutionNode()->calcCost()));

			expCost = planner->getSolutionNode()->getSegCost() - 1;

			if (expCost >= 1)
			{
				// clear planner data
				planner->clear();

				// update planner
				delete planner;
				planner = new XG_CBS(env, expCost, perc_exp);
				planner->setSolveTime(maxCompTime);

				if (lowLevelAlg == "A")
					success = planner->plan(env->getStarts(), solution, false, false, false);
				else if (lowLevelAlg == "XG-A")
					success = planner->plan(env->getStarts(), solution, true, false, false);
				else if (lowLevelAlg == "XG-A-H")
					success = planner->plan(env->getStarts(), solution, false, true, false);
				else if (lowLevelAlg == "S-A")
					success = planner->plan(env->getStarts(), solution, false, false, true);
				else
					printf("Unable to benchmark do to invalid low-level algorithm.");
			}
			else
				success = false;
		}
		// make map data same length as planning data
		if (xgComputationTimes.size() > 0)
		{
			for (int i=1; i < xgComputationTimes.size(); i++)
			{
				// map data
				name.push_back(std::string());
				agents.push_back(std::string());
				grid.push_back(std::string());
				// cbs data
				ComputationTimes.push_back(std::string());
				NumEvalNodes.push_back(std::string());
				Costs.push_back(std::string());
				PathLength.push_back(std::string());
			}
		}
		else
		{
			xgComputationTimes.push_back(std::to_string(planner->getCompTime()));
			xgNumEvalNodes.push_back("-");
			xgCosts.push_back("-");
			xgPathLength.push_back("-");
		}
		// col 1: map name
		std::pair< std::string, std::vector<std::string> > mapName{"Map", name};
		// col 2: number of agents
		std::pair< std::string, std::vector<std::string> > numAgents{"# of Agents", agents};
		// col 3: grid size
		std::pair< std::string, std::vector<std::string> > size{"Grid Size", grid};
		// col 4: cbs comp time
		std::pair< std::string, std::vector<std::string> > times{"CBS Time (s)", ComputationTimes};
		// col 5: cbs evaluated nodes
		std::pair< std::string, std::vector<std::string> > trees{"CBS # of Evalidated Nodes", NumEvalNodes};
		// col 6: cbs exp costs
		std::pair< std::string, std::vector<std::string> > costs{"CBS Cost", Costs};
		// col 7: cbs SoC
		std::pair< std::string, std::vector<std::string> > SoC{"CBS SoC", PathLength};
		// col 8: xg-cbs computation time
		std::pair< std::string, std::vector<std::string> > xgTimes{"XG-CBS Time (s)", xgComputationTimes};
		// col 9: xg-cbs evaluated nodes
		std::pair< std::string, std::vector<std::string> > xgTrees{"XG-CBS # of Evalidated Nodes", xgNumEvalNodes};
		// col 10: xg-cbs exp costs
		std::pair< std::string, std::vector<std::string> > xgExps{"XG-CBS Cost", xgCosts};
		// col 11: xg-cbs SoC
		std::pair< std::string, std::vector<std::string> > xgSoC{"XG-CBS SoC", xgPathLength};

		// create dataset and return it
		std::vector<std::pair<std::string, std::vector<std::string>>> results = {mapName, numAgents, size, times, trees, costs, SoC, xgTimes, xgTrees, xgExps, xgSoC};
		
		for (auto elem: results)
		{
			printf("Size: %lu \n", elem.second.size());
		}
		// exit(1);



		return results;
	}
}

std::vector<std::pair <std::string, std::vector<std::string>> > multiMapBenchmark(const std::string files, 
	const double maxCompTime, const std::string lowLevelAlg, const double perc_exp)
{
	// get vector of file names from directory (and sub-directories)
    std::vector<std::pair<const std::string, const std::string>> mapInfo;
    for (const auto & entry: fs::recursive_directory_iterator(files))
    {
    	std::string dir2file = std::string(entry.path()).substr(std::string(entry.path()).rfind("/")+1);
		const std::string::size_type end = dir2file.find(".yaml");
		if (end < dir2file.size())
		{
			const std::string mapPath = std::string(entry.path());
			const std::string mapName = dir2file.erase(end, dir2file.length());
			mapInfo.push_back( {mapPath, mapName} );
		}
    }

    // for each map in maps, run singleMapBenchmark
    std::vector<std::pair <std::string, std::vector<std::string>> > dataset;
    for (std::pair<const std::string, const std::string> map: mapInfo)
    {
    	// create environment from yaml file (assumed to be a file)
		Environment *mapf = yaml2env(map.first);
		mapf->setMapName(map.second);
    	std::vector<std::pair <std::string, std::vector<std::string>> > dat = 
    		singleMapBenchmark(mapf, maxCompTime, lowLevelAlg, perc_exp);
    	if (!dat.empty())
    	{
    		// save new information
    		if (dataset.empty())
    			dataset.insert(dataset.end(), dat.begin(), dat.end());
    		else
    			for (int c=0; c < dat.size(); c++)
    				dataset[c].second.insert(dataset[c].second.end(), dat[c].second.begin(), dat[c].second.end());
    	}
    	delete mapf;
    }	
	return dataset;
}


// std::vector<std::pair <std::string, std::vector<std::string>> > singleCostMatch(Environment* env, 
// 	const double maxCompTime, const double perc_exp)
// {
// 	// init solution (to be filled by planning)
// 	Solution solution;

// 	// plan using CBS
// 	CBS *cbs = new CBS(env);
// 	cbs->setSolveTime(maxCompTime);
// 	bool success = cbs->plan(cbs->getEnv()->getStarts(), solution);

// 	if (success)
// 	{
// 		// init the columns of data
// 		std::vector<std::string> name{cbs->getEnv()->getMapName()};
// 		std::vector<std::string> agents{std::to_string(cbs->getEnv()->getAgentNames().size())};
// 		std::vector<std::string> grid{std::to_string( (cbs->getEnv()->getXdim() + 1) * (cbs->getEnv()->getYdim() + 1) )};
// 		std::vector<std::string> ComputationTimes{};
// 		std::vector<std::string> NumEvalNodes{};
// 		std::vector<std::string> Costs{};
// 		std::vector<std::string> xgComputationTimes{};
// 		std::vector<std::string> xgNumEvalNodes{};
// 		std::vector<std::string> xgCosts{};

// 		//cbs cost
// 		ComputationTimes.push_back(std::to_string(cbs->getCompTime()));
// 		// cbs number of evaluated nodes is equal to number of expanded nodes + 1 (the solution)
// 		NumEvalNodes.push_back(std::to_string(cbs->closed_set_.size()+1));
// 		// solution explanation cost and update bound
// 		Costs.push_back(std::to_string(cbs->getSolutionNode()->getSegCost()));
// 		// get beginning cost for xg-cbs
// 		int expCost = cbs->getSolutionNode()->getSegCost();

// 		// remove all cbs data
// 		cbs->clear();
// 		delete cbs;

// 		if (expCost >= 1)
// 		{
// 			// plan using XG-CBS
// 			printf("ExpCbs Given Bound: %d \n", expCost);
// 			XG_CBS *planner = new XG_CBS(env, expCost, perc_exp);
// 			planner->setSolveTime(maxCompTime);
// 			success = planner->plan(planner->getEnv()->getStarts(), solution, true, true);

// 			// is successful, begin benchmarking loop
// 			if (success)
// 			{
// 				// computation time
// 				xgComputationTimes.push_back(std::to_string(planner->getCompTime()));
// 				// number of evaluated nodes is equal to number of expanded nodes + 1 (the solution)
// 				xgNumEvalNodes.push_back(std::to_string(planner->closed_set_.size()+1));
// 				// solution explanation cost and update bound
// 				xgCosts.push_back(std::to_string(planner->getSolutionNode()->getSegCost()));
// 				expCost = planner->getSolutionNode()->getSegCost() - 1;
// 			}
// 		}
// 		// make map data same length as planning data
// 		if (xgComputationTimes.size() > 0)
// 		{
// 			for (int i=1; i < xgComputationTimes.size(); i++)
// 			{
// 				// map data
// 				name.push_back(std::string());
// 				agents.push_back(std::string());
// 				grid.push_back(std::string());
// 				// cbs data
// 				ComputationTimes.push_back(std::string());
// 				NumEvalNodes.push_back(std::string());
// 				Costs.push_back(std::string());
// 			}
// 		}
// 		else
// 		{
// 			xgComputationTimes.push_back("-");
// 			xgNumEvalNodes.push_back("-");
// 			xgCosts.push_back("-");
// 		}
// 		// col 1: map name
// 		std::pair< std::string, std::vector<std::string> > mapName{"Map", name};
// 		// col 2: number of agents
// 		std::pair< std::string, std::vector<std::string> > numAgents{"# of Agents", agents};
// 		// col 3: grid size
// 		std::pair< std::string, std::vector<std::string> > size{"Grid Size", grid};
// 		// col 4: cbs comp time
// 		std::pair< std::string, std::vector<std::string> > times{"CBS Time (s)", ComputationTimes};
// 		// col 5: cbs evaluated nodes
// 		std::pair< std::string, std::vector<std::string> > trees{"CBS # of Evalidated Nodes", NumEvalNodes};
// 		// col 6: cbs exp costs
// 		std::pair< std::string, std::vector<std::string> > costs{"CBS Cost", Costs};
// 		// col 7: xg-cbs computation time
// 		std::pair< std::string, std::vector<std::string> > xgTimes{"XG-CBS Time (s)", xgComputationTimes};
// 		// col 8: xg-cbs evaluated nodes
// 		std::pair< std::string, std::vector<std::string> > xgTrees{"XG-CBS # of Evalidated Nodes", xgNumEvalNodes};
// 		// col 9: xg-cbs exp costs
// 		std::pair< std::string, std::vector<std::string> > xgExps{"XG-CBS Cost", xgCosts};

// 		// create dataset and return it
// 		std::vector<std::pair<std::string, std::vector<std::string>>> results = {mapName, numAgents, size, times, trees, costs, xgTimes, xgTrees, xgExps};
// 		return results;
// 	}
// 	else
// 	{
// 		// cbs did not find a solution, return an empty result
// 		std::vector<std::pair<std::string, std::vector<std::string>>> empty;
// 		return empty;
// 	}
// }



// std::vector<std::pair <std::string, std::vector<std::string>> > multiCostMatch(const std::string files, 
// 	const double maxCompTime, const double perc_exp)
// {
// 	// get vector of file names from directory (and sub-directories)
//     std::vector<std::pair<const std::string, const std::string>> mapInfo;
//     for (const auto & entry: fs::recursive_directory_iterator(files))
//     {
//     	std::string dir2file = std::string(entry.path()).substr(std::string(entry.path()).rfind("/")+1);
// 		const std::string::size_type end = dir2file.find(".yaml");
// 		if (end < dir2file.size())
// 		{
// 			const std::string mapPath = std::string(entry.path());
// 			const std::string mapName = dir2file.erase(end, dir2file.length());
// 			mapInfo.push_back( {mapPath, mapName} );
// 		}
//     }

//     // for each map in maps, run singleMapBenchmark
//     std::vector<std::pair <std::string, std::vector<std::string>> > dataset;
//     for (std::pair<const std::string, const std::string> map: mapInfo)
//     {
//     	// create environment from yaml file (assumed to be a file)
// 		Environment *mapf = yaml2env(map.first);
// 		mapf->setMapName(map.second);
//     	std::vector<std::pair <std::string, std::vector<std::string>> > dat = singleCostMatch(mapf, maxCompTime, perc_exp);
//     	if (!dat.empty())
//     	{
//     		// save new information
//     		if (dataset.empty())
//     			dataset.insert(dataset.end(), dat.begin(), dat.end());
//     		else
//     			for (int c=0; c < dat.size(); c++)
//     				dataset[c].second.insert(dataset[c].second.end(), dat[c].second.begin(), dat[c].second.end());
//     	}
//     	delete mapf;
//     }	
// 	return dataset;
// }












