#pragma once

#include <string>
#include <vector>

#include "debug_tools.h"

//struct to define the data retained for a active launch
struct LaunchData{
    std::string launch_name;

    double cpu_usage;
    double mem_usage;

    int launch_status;
    int launch_progress;
};

void update_launch_data_from_string(LaunchData* data, std::vector<std::string> launch_data);