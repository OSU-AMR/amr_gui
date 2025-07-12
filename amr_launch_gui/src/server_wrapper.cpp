#include "server_wrapper.h"

ServerWrapper::ServerWrapper(){
    //create the server wrapper

}

ServerWrapper::~ServerWrapper(){

}


void ServerWrapper::run(){
    //run for the server wrapper -- basic behavoir is come online and then spin

    int server_failure = 0;
    while (server_failure == 0){
        server_failure = server.spin();

        fflush(stdout);

        //get any available data
        while(server.isDataAvailable()){

            GuiUpdateData data = server.getNextData();

            direct_data(data);
        }

        sleep(.01);
    }

    write_to_console("Launch server failed with code: " + std::to_string(server_failure));
}

int ServerWrapper::direct_data(GuiUpdateData data){
    //direct the data where it needs to go

    if(data.update_type == APPEND_CENTRAL){
        //add a new central to the list
        central_hosts.push_back(data.target);

        //emit the signal so the main window can further redirect
        emit central_data_available(data);
    }

    if(data.update_type == APPEND_AMR){
        //add a new amr to the list
        amr_hosts.push_back(data.target);

        //emit the signal so the main window can further redirect
        emit amr_data_available(data);
    }

    for(int i = 0; i < amr_hosts.size(); i++){
        //send to a amr host

        if(amr_hosts.at(i) == data.target){
            //emit the signal so the main window can further redirect
            emit amr_data_available(data);

            return 0;
        }
    }

    for(int i = 0; i < central_hosts.size(); i++){
        //send to a central host

        if(central_hosts.at(i) == data.target){
            //emit the signal so the main window can further redirect
            emit central_data_available(data);

            return 0;
        }
    }

    return 0;
}

void ServerWrapper::handleCentralHostnameChange(QString hostname){
    //set the central hostname
    central_hostname = std::string(hostname.toStdString());
}

void ServerWrapper::handleCentralLaunchBegin(QString filename){
    //tell the central to run a launch file

    if(central_hostname != ""){
        GuiUpdateData data;
        data.target = central_hostname;
        data.update_type = START_ROS_LAUNCH;
        std::vector<std::string> args;
        args.push_back(std::string(filename.toStdString()));
        data.update_data = args;

        //send the data to the central

        write_to_console("Sending Launch to Central\n");
        if(server.distribute_message(data) < 0){
            write_to_console("Could not find target when handling central launch!\n");
        }
    }
}

void ServerWrapper::handleCentralLaunchHalt(QString filename){
    //tell the central to stop a launch file

    if(central_hostname != ""){
        GuiUpdateData data;
        data.target = central_hostname;
        data.update_type = STOP_ROS_LAUNCH;
        std::vector<std::string> args;
        args.push_back(std::string(filename.toStdString()));
        data.update_data = args;

        if(server.distribute_message(data) < 0){
            write_to_console("Could not find target when handling central launch!");
        }
    }
}


#include "moc_server_wrapper.cpp"

