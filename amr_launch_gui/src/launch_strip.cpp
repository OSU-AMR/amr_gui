
#include "launch_strip.h"

LaunchStrip::LaunchStrip(std::string ui_file_path, QObject *parent) : QObject(parent)
{
    //load in from layout file
    QUiLoader loader;

    //get qt file path
    std::string share_dir_path = ament_index_cpp::get_package_share_directory("amr_launch_gui");

    //load in the launch strip
    std::string launch_strip_path = std::string(share_dir_path + ui_file_path);
    QFile file(launch_strip_path.c_str());
    if(!file.open(QFile::ReadOnly)){
        qFatal("Cannot open tab layout ui! %s", launch_strip_path.c_str());
    }

    child_widget = loader.load(&file);
    file.close();

    //set the tab's layout to its child layout --IMPORTANT
    child_widget->setLayout(child_widget->findChild<QLayout *>());

    //load all the children
    findChildren();

    //connect the buttons
    connect(launch_button, &QPushButton::clicked, this, &LaunchStrip::handleLaunchPress);
    connect(halt_button, &QPushButton::clicked, this, &LaunchStrip::handleHaltPress);
    connect(apply_button, &QPushButton::clicked, this, &LaunchStrip::handleApplyPress);

    //connect the combos
    connect(launch_select, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &LaunchStrip::handleLaunchSelect);
    connect(arg_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &LaunchStrip::handleLaunchArgSelect);

    //shouldn't halt by default
    halt_button->setDisabled(true);

}

void LaunchStrip::findChildren(){
    //find all the childern and add pointers to make them easily accessible
    launch_progress_bar = child_widget->findChild<QProgressBar *>("launch_progress_bar");
    uptime_label = child_widget->findChild<QLabel *>("uptime_label");
    launch_select = child_widget->findChild<QComboBox *>("launchSelect");
    launch_button = child_widget->findChild<QPushButton *>("launch_button");
    halt_button = child_widget->findChild<QPushButton *>("halt_button");
    apply_button = child_widget->findChild<QPushButton *>("applylauncharg");
    arg_combo = child_widget->findChild<QComboBox *>("launchargcombo");
    arg_value_box = child_widget->findChild<QLineEdit *>("launchargvalue");
}

AvailableLaunches LaunchStrip::getSelectedLaunch(){

    std::string launch_name = launch_select->currentText().toStdString();

    for(int i = 0; i < launches.size(); i++){
        if(launch_name == launches.at(i).launch_name){
            return launches.at(i);
        }
    }

    write_to_console("Could not find launch with name in launch strip!");
    return AvailableLaunches();
}

void LaunchStrip::setLaunchArgValue(std::string str, int arg_index){

    std::string launch_name = launch_select->currentText().toStdString();

    for(int i = 0; i < launches.size(); i++){
        if(launch_name == launches.at(i).launch_name){
            launches.at(i).launch_arg_values.at(arg_index) = str;
            return;
        }
    }

    write_to_console("Could not find launch with name in launch strip!");
    return;
}

void LaunchStrip::setLaunchOptions(std::vector<AvailableLaunches> options){
    std::vector<std::string> name_vector;
    for(int i = 0; i < options.size(); i++){
        name_vector.push_back(options.at(i).launch_name);
    }

    launches = options;

    setComboBoxItems(launch_select, name_vector);
}

QComboBox *LaunchStrip::getLaunchSelectPtr(){
    return launch_select;
}
    
QPushButton *LaunchStrip::getLaunchButtonPtr(){
    return launch_button;
}

QPushButton *LaunchStrip::getHaltButtonPtr(){
    return halt_button;
}

void LaunchStrip::setComboBoxItems(QComboBox* comboBox, std::vector<std::string> items) {
    if (!comboBox) return;  // Safety check

    comboBox->clear();  // Remove existing items

    for (const auto& item : items) {
        comboBox->addItem(QString::fromStdString(item));
    }
}

void LaunchStrip::handleLaunchPress(){

    //get the launch string
    std::string launch_string = formulate_launch_request_string(getSelectedLaunch());

    //disable the combo
    launch_select->setDisabled(true);
    launch_button->setDisabled(true);
    halt_button->setDisabled(false);

    emit beginLaunch(QString::fromStdString(launch_string));
}

void LaunchStrip::handleHaltPress(){
    if(launch_select->currentText() == ""){
        return;
    }

    launch_select->setDisabled(false);
    launch_button->setDisabled(false);
    halt_button->setDisabled(true);

    emit haltLaunch(launch_select->currentText());
}

void LaunchStrip::handleApplyPress(){
    //update the currently selected launch arguments

    //get the current launch
    AvailableLaunches launch = getSelectedLaunch();

    if(arg_combo->currentIndex() < launch.launch_arg_values.size()){
        //get default arg value
        setLaunchArgValue(arg_value_box->text().toStdString(), arg_combo->currentIndex()); 
    }
}

void LaunchStrip::handleLaunchSelect(){

    //get the currently selected launch
    //set the values in the combo
    setComboBoxItems(arg_combo, getSelectedLaunch().launch_arg_names);
}

void LaunchStrip::handleLaunchArgSelect(){
    
    //get the current launch
    AvailableLaunches launch = getSelectedLaunch();

    if(arg_combo->currentIndex() < launch.launch_arg_values.size()){
        //get default arg value
        std::string default_value = launch.launch_arg_values.at(arg_combo->currentIndex()); 

        //update the default value
        arg_value_box->setText(QString::fromStdString(default_value));
    } else {
        arg_value_box->setText(QString::fromStdString(""));
    }
}

std::string LaunchStrip::formulate_launch_request_string(AvailableLaunches launch){
    //formulate the launch request string - not fully just add the arguments with spacers...
    //doing this here only to not have to pass all the data as serpate units
    //need to register as a q object and that would take time

    std::string launch_string = launch.launch_name;

    for(int i = 0; i < launch.launch_arg_names.size(); i++){
        launch_string += ARGUMENT_SERPARATION_SEQUENCE + launch.launch_arg_names.at(i) + ARGUMENT_SERPARATION_SEQUENCE + launch.launch_arg_values.at(i);
    }

    return launch_string;
}


LaunchStrip::~LaunchStrip()
{

}

#include "moc_launch_strip.cpp"