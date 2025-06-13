
#include "launch_strip.h"

LaunchStrip::LaunchStrip(std::string ui_file_path)
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
}

void LaunchStrip::findChildren(){
    //find all the childern and add pointers to make them easily accessible
    launch_progress_bar = child_widget->findChild<QProgressBar *>("launch_progress_bar");

    launch_select = child_widget->findChild<QComboBox *>("launchSelect");
}

LaunchStrip::~LaunchStrip()
{

}