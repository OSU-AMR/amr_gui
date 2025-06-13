#include "central_tab.h"

CentralTab::CentralTab(QObject *parent) : DeviceTab("/ui/test2.ui", parent)
{
    find_central_children();
}



CentralTab::~CentralTab()
{

}

void CentralTab::find_central_children(){
    selectCombo = child_widget->findChild<QComboBox *>("centralSelectCombo");
}

void CentralTab::handle_data(GuiUpdateData data){
    //update the data from the GUI

    if(data.update_type == APPEND_CENTRAL){
        //add a new central node

        bool central_not_listed = true;

        //check to ensure the central is not already listed
        for(auto it = central_data.begin(); it != central_data.end(); ++it){
            if(it->first == data.target){
                central_not_listed = false;
            }
        }

        //add the central to the list
        if(central_not_listed){
            CentralData cd;
            cd.hostname = data.target;
            central_data[data.target] = cd;

            write_to_console("Added new central");
        } else {
            write_to_console("Not adding central already on the list!");
        }

        //update the combo
        update_central_combo();


    }else if(data.update_type == CLEAR_LAUNCH_FILES){
        //remove all current launch files for the target

        for(auto it = central_data.begin(); it != central_data.end(); ++it){
            if(it->first == data.target){
                it->second.availble_launches.clear();

                return;
            }
        }

        write_to_console("Could not find target! CLEAR_LAUNCH_FILES - " + std::string(data.target));
    } else if(data.update_type == APPEND_LAUNCH_FILES){
        //add to current launch files for the target

        for(auto it = central_data.begin(); it != central_data.end(); ++it){
            if(it->first == data.target){
                for(int i = 0; i < data.update_data.size(); i++){
                    it->second.availble_launches.push_back(data.update_data.at(i));
                }

                return;
            }
        }

        write_to_console("Could not find target! APPEND_LAUNCH_FILES - " + std::string(data.target));
    }
}

CentralData* CentralTab::get_current_central_data(){

    //returns the data for the currently selected central data
    for(int i = 0; i < central_data.size(); i++){
        if(central_data.at(i) == selectCombo->currentText().toStdString()){
            //matches

            return &(central_data.at(i));
        }
    }

    return nullptr;
}

void CentralTab::update_launch_bars(){
    

}

void CentralTab::update_central_combo(){
    //refresh the list of centrals in the combo box
    
    std::vector<std::string> centrals;

    for(auto it = central_data.begin(); it != central_data.end(); ++it){
        centrals.push_back(it->first);
    }

    setComboBoxItems(selectCombo, centrals);
}