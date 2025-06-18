
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

    //connec5 5h3 buttons
    connect(launch_button, &QPushButton::clicked, this, &LaunchStrip::handleLaunchPress);
    connect(halt_button, &QPushButton::clicked, this, &LaunchStrip::handleHaltPress);
}

void LaunchStrip::findChildren(){
    //find all the childern and add pointers to make them easily accessible
    launch_progress_bar = child_widget->findChild<QProgressBar *>("launch_progress_bar");
    uptime_label = child_widget->findChild<QLabel *>("uptime_label");
    launch_select = child_widget->findChild<QComboBox *>("launchSelect");
    launch_button = child_widget->findChild<QPushButton *>("launch_button");
    halt_button = child_widget->findChild<QPushButton *>("halt_button");
}

std::string LaunchStrip::getSelectedLaunch(){

    return launch_select->currentText().toStdString();
}

void LaunchStrip::setLaunchOptions(std::vector<std::string> options){
    setComboBoxItems(launch_select, options);
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

void LaunchStrip::setComboBoxItems(QComboBox* comboBox, std::vector<std::string>& items) {
    if (!comboBox) return;  // Safety check

    comboBox->clear();  // Remove existing items

    for (const auto& item : items) {
        comboBox->addItem(QString::fromStdString(item));
    }
}

void LaunchStrip::handleLaunchPress(){
    if(launch_select->currentText() == ""){
        return;
    }

    //disable the combo
    launch_select->setDisabled(true);

    emit beginLaunch(launch_select->currentText());
}

void LaunchStrip::handleHaltPress(){
    if(launch_select->currentText() == ""){
        return;
    }

    launch_select->setDisabled(false);

    emit haltLaunch(launch_select->currentText());
}

LaunchStrip::~LaunchStrip()
{

}

#include "moc_launch_strip.cpp"