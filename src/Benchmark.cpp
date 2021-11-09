#include "../includes/Benchmark.h"


// https://www.gormanalysis.com/blog/reading-and-writing-csv-files-with-cpp/
void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<std::string>>> dataset){
    // Make a CSV file with one or more columns of integer values
    // Each column of data is represented by the pair <column name, column data>
    //   as std::pair<std::string, std::vector<int>>
    // The dataset is represented as a vector of these columns
    // Note that all columns should be the same size
    
    // Create an output filestream object
    std::ofstream myFile(filename);
    
    // Send column names to the stream
    for(int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j).first;
        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";
    
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
    
    // Close the file
    myFile.close();
}

std::vector<std::pair <std::string, std::vector<std::string>> > singleMapBenchmark(Environment* env, int expCost, const double maxCompTime)
{
	printf("%s: Preparing to benchmark %s.\n", "XG-CBS", env->getMapName().c_str());

	XG_CBS *planner = new XG_CBS(env, expCost);
	planner->setSolveTime(maxCompTime);

	// init solution (to be filled by planning)
	Solution solution;

	// init the columns of data
	std::vector<std::string> name{planner->getEnv()->getMapName()};
	std::vector<std::string> agents{std::to_string(planner->getEnv()->getAgentNames().size())};
	std::vector<std::string> grid{std::to_string( (planner->getEnv()->getXdim() + 1) * (planner->getEnv()->getYdim() + 1) )};
	std::vector<std::string> computationTimes{};
	std::vector<std::string> numEvalNodes{};
	std::vector<std::string> costs{};

	// plan 
	bool success = planner->plan(planner->getEnv()->getStarts(), solution, true, true);
	
	// is successful, begin benchmarking loop
	while (success)
	{
		// computation time
		computationTimes.push_back(std::to_string(planner->getCompTime()));
		// number of evaluated nodes is equal to number of expanded nodes + 1 (the solution)
		numEvalNodes.push_back(std::to_string(planner->closed_set_.size()+1));
		// solution explanation cost and update bound
		costs.push_back(std::to_string(planner->getSolutionNode()->getSegCost()));
		expCost = planner->getSolutionNode()->getSegCost() - 1;

		// clear planner data
		planner->clear();

		// update planner
		delete planner;
		planner = new XG_CBS(env, expCost);
		planner->setSolveTime(maxCompTime);

		// plan again with lower cost
		success = planner->plan(planner->getEnv()->getStarts(), solution, true, true);
	}
	// make map data same length as planning data
	for (int i=1; i < computationTimes.size(); i++)
	{
		name.push_back(std::string());
		agents.push_back(std::string());
		grid.push_back(std::string());
	}

	// col 1: map name
	std::pair< std::string, std::vector<std::string> > mapName{"Map", name};
	// col 2: number of agents
	std::pair< std::string, std::vector<std::string> > numAgents{"# of Agents", agents};
	// col 3: grid size
	std::pair< std::string, std::vector<std::string> > size{"Grid Size", grid};
	// col 4: computation time
	std::pair< std::string, std::vector<std::string> > times{"Time (s)", computationTimes};
	// col 5: evaluated nodes
	std::pair< std::string, std::vector<std::string> > tree{"# of Evalidated Nodes", numEvalNodes};
	// col 6: explanation costs
	std::pair< std::string, std::vector<std::string> > Costs{"Exp. Cost", costs};

	// create dataset
	std::vector<std::pair<std::string, std::vector<std::string>>> results = {mapName, numAgents, size, times, tree, Costs};

	return results;
}

std::vector<std::pair <std::string, std::vector<std::string>> > multiMapBenchmark(const std::string files, int expCost, const double maxCompTime)
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
    	std::vector<std::pair <std::string, std::vector<std::string>> > dat = singleMapBenchmark(mapf, expCost, maxCompTime);
    	// save new information
    	if (dataset.empty())
    		dataset.insert(dataset.end(), dat.begin(), dat.end());
    	else
    		for (int c=0; c < dat.size(); c++)
    			dataset[c].second.insert(dataset[c].second.end(), dat[c].second.begin(), dat[c].second.end());
    }	
	return dataset;
}
