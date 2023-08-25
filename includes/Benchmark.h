#pragma once
#include "../includes/Environment.h"
#include "../includes/XG-Astar-H.h"
#include "../includes/XG-CBS.h"
#include "../includes/Cbs.h"
#include "../includes/parseYAML.h"
#include <iostream>
#include <string>
#include <fstream>
#include <utility> // std::pair
#include <filesystem>
namespace fs = std::filesystem;



// https://www.gormanalysis.com/blog/reading-and-writing-csv-files-with-cpp/
void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<std::string>>> dataset);

// run a benchmark run until failure, return data
std::vector<std::pair <std::string, std::vector<std::string>> > singleMapBenchmark(Environment* env, 
	const double maxCompTime, const std::string AstarType, const double perc_exp);

// run a benchmark run until failure for multiple files
std::vector<std::pair <std::string, std::vector<std::string>> > multiMapBenchmark(const std::string files, 
	const double maxCompTime, const std::string AstarType, const double perc_exp);

// // run CBS and have XG-CBS match cost
// std::vector<std::pair <std::string, std::vector<std::string>> > singleCostMatch(Environment* env, 
// 	const double maxCompTime, const double perc_exp);

// // run CBS and have XG-CBS match cost for multiple files
// std::vector<std::pair <std::string, std::vector<std::string>> > multiCostMatch(const std::string files, 
// 	const double maxCompTime, const double perc_exp);
