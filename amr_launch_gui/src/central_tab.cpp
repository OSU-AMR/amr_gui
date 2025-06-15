#include "central_tab.h"

CentralTab::CentralTab(QObject *parent) : DeviceTab("/ui/test2.ui", parent)
{
    find_central_children();

    //connect the central select combo
    connect(selectCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CentralTab::handleCentralComboChange);
}



CentralTab::~CentralTab()
{

}

void CentralTab::find_central_children(){
    selectCombo = child_widget->findChild<QComboBox *>("centralSelectCombo");
}

void CentralTab::handle_data(GuiUpdateData data){
    //update the data from the GUI

    write_to_console("New Data: " + data.update_type);

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

                update_launch_bars();

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

                update_launch_bars();

                return;
            }
        }

        write_to_console("Could not find target! APPEND_LAUNCH_FILES - " + std::string(data.target));
    }
}

CentralData* CentralTab::get_current_central_data(){

    //returns the data for the currently selected central data
    for(auto it = central_data.begin(); it != central_data.end(); ++it){
        if(it->first == selectCombo->currentText().toStdString()){
            //matches

            return &(it->second);
        }
    }

    return nullptr;
}

void CentralTab::update_launch_bars(){
    
    write_to_console("Here2");


    int empty_index = -1;

    std::vector<int> stripsToRemove;
    std::vector<std::string> usedLaunchOptions;
    std::vector<std::string> validLaunchOptions;

    if(get_current_central_data() == nullptr){
        //no central selected - remove all launches
        for(int i = getLaunchStripsCount() - 1; i >= 0; i--){
            removeLaunchStrip(i);
        }

        return;
    }

    write_to_console("Here8");


    for(auto it = get_current_central_data()->availble_launches.begin(); it != get_current_central_data()->availble_launches.end(); ++it){
        validLaunchOptions.push_back(*it);
    }

    write_to_console("Here7");


    //go through and update each launch bar
    for(int i = 0; i < getLaunchStripsCount(); i++){

        //check to see if there is a launch strip without a launch selected
        if(getLaunchStripCurrentLaunch(i) == std::string("")){

            //if an unselected launch strip has been found
            if(empty_index >= 0){
                stripsToRemove.push_back(i);
            }

            empty_index = i;

            continue;
        }

        //check to make sure a valid option is selected
        bool valid = false;
        for(int j = 0; j < validLaunchOptions.size(); j++){
            if(getLaunchStripCurrentLaunch(i) == validLaunchOptions.at(j)){
                valid = true;
            }
        }

        if(!valid){
            //remove the invalid launch
            stripsToRemove.push_back(i);
            continue;
        }

        //check to make sure no duplicated
        for(int j = 0; j < usedLaunchOptions.size(); j++){
            if(getLaunchStripCurrentLaunch(i) == usedLaunchOptions.at(j)){
                //remove duplicate
                stripsToRemove.push_back(i);
            }
        }

        //add to the list of current launches
        usedLaunchOptions.push_back(getLaunchStripCurrentLaunch(i));
    }

    write_to_console("Here6");


    //remove the strips in decending order
    for(int i = stripsToRemove.size() - 1; i >= 0; i--){
        removeLaunchStrip(i);
    }

    write_to_console("Here5");


    //determine the launch options still remaining
    std::vector<std::string> remaining_options;
    for(int i = 0; i < validLaunchOptions.size(); i++){

        //determine if the launch option has been used
        bool option_used = false;
        for(int j = 0; j < usedLaunchOptions.size(); j++){
            if(usedLaunchOptions.at(j) == validLaunchOptions.at(i)){
                option_used = true;

                break;
            }
        }

        if(!option_used){
            remaining_options.push_back(validLaunchOptions.at(i));
        }
    }

    write_to_console("Here4");


    //set the launch options
    for(int i = 0; i < getLaunchStripsCount(); i++){

        //ensure the current launch is an option
        std::vector<std::string> temp = remaining_options;
        temp.push_back(getLaunchStripCurrentLaunch(i));

        setAvailableLaunchOptions(i, temp);
    }

    write_to_console("Here3");


    if(empty_index == -1){

        //always one empty launch strip
        loadLaunchStrip();

        setAvailableLaunchOptions(getLaunchStripsCount() - 1, remaining_options);
    }   

}

void CentralTab::update_central_combo(){
    //refresh the list of centrals in the combo box
    
    std::vector<std::string> centrals;

    for(auto it = central_data.begin(); it != central_data.end(); ++it){
        centrals.push_back(it->first);
    }

    setComboBoxItems(selectCombo, centrals);

    update_launch_bars();
}

void CentralTab::handleCentralComboChange(){
    //update the launch bars
    update_launch_bars();
}

#include "moc_central_tab.cpp"