#pragma once

#include <string>
#include <vector>

#include "launch_data.h"
#include "communication_signals.h"

//struct to define the data retained for a central node
struct CentralData{

    std::string hostname;
    std::string ip_addr;
    std::string domain;
    std::string status;
    
    std::vector<std::string> availble_launches;
    std::vector<LaunchData> launch_data;
};

void update_central_data_from_string(CentralData* data, std::string signal_type, std::vector<std::string> launch_data);