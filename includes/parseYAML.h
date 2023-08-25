#pragma once
#include "../includes/State.h"
#include "../includes/Environment.h"
#include <unordered_set>
#include <string>
#include <yaml-cpp/yaml.h>
#include <fstream>


Environment* yaml2env(const std::string fileName);
