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

        //emit the signal so the main window can further redirect
        emit amr_data_available(data);
    }

    for(int i = 0; i < central_hosts.size(); i++){
        //send to a central host

        //emit the signal so the main window can further redirect
        emit central_data_available(data);
    }
}

#include "moc_server_wrapper.cpp"

