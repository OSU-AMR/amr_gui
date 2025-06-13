#include "launch_data.h"

void update_launch_data_from_string(LaunchData* data, std::vector<std::string> launch_data){
    
    if(launch_data.size() != 5){
        write_to_console("Failed to convert launch data, wrong size!\n");
        return;
    }

    try{
        data->cpu_usage = std::stod(launch_data.at(1));
        data->mem_usage = std::stod(launch_data.at(2));
        data->launch_status = std::stoi(launch_data.at(3));
        data->launch_progress = std::stoi(launch_data.at(4));
    }catch(...){
        write_to_console("Failed to convert launch data!\n");
    }



}