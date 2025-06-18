#include "device_tab.h"

DeviceTab::DeviceTab(std::string ui_file_path, QObject *parent) : QObject(parent)
{
    //load in from layout file
    QUiLoader loader;

    //get qt file path
    std::string share_dir_path = ament_index_cpp::get_package_share_directory("amr_launch_gui");
    
    std::string device_tab_widget = std::string(share_dir_path + ui_file_path);
    QFile file(device_tab_widget.c_str());
    if(!file.open(QFile::ReadOnly)){
        qFatal("Cannot open tab layout ui! %s", device_tab_widget.c_str());
    }

    child_widget = loader.load(&file);
    file.close();

    //set the tab's layout to its child layout --IMPORTANT
    child_widget->setLayout(child_widget->findChild<QLayout *>());

    //load all the children
    findChildren();
}

void DeviceTab::findChildren(){
    //find all the childern and add pointers to make them easily accessible
    shellOutput = child_widget->findChild<QTextEdit *>("shelloutput");
    shellCursor = shellOutput->textCursor();
    text_edit_width = shellOutput->width();

    launchScrollArea = child_widget->findChild<QScrollArea *>("launchscrollarea");
    launchScrollAreaWidget = launchScrollArea->widget();
    verticalLayoutScrollArea = new QVBoxLayout();
    launchScrollAreaWidget->findChild<QHBoxLayout *>()->addLayout(verticalLayoutScrollArea);

    rebootDeviceButton = child_widget->findChild<QPushButton *>("rebootdevice");
    connect(rebootDeviceButton, &QPushButton::clicked, this, &DeviceTab::onRebootPress);
}


void DeviceTab::handle_data(GuiUpdateData data){

}


int DeviceTab::getLaunchStripsCount(){
    return launch_strips.size();
}

std::string DeviceTab::getLaunchStripCurrentLaunch(int index){
    //get the currently selected launch options
    return launch_strips.at(index)->getSelectedLaunch();
}

void DeviceTab::setAvailableLaunchOptions(int index, std::vector<std::string> options){
    //set the launch options
    launch_strips.at(index)->setLaunchOptions(options);
}


void DeviceTab::onRebootPress(){
    renderShellText("The button has been pressed! The button has been pressed! The button has been pressed! The button has been pressed!The button has been pressed! The button has been pressed! The button has been pressed! The button has been pressed! The button has been pressed! The button has been pressed!The button has been pressed!The button has been pressed!The button has been pressed!The button has been pressed!The button has been pressed!The button has been pressed!The button has been pressed!The button has been pressed!The button has been pressed!The button has been pressed!The button has been pressed!\n\n\n\n\n\n\n\n\n\n");
}

void DeviceTab::reRender(bool isShown){
    //reRender the tab
    child_widget->update();

    //re update th width of the text edit
    text_edit_width = shellOutput->width();

    if(isShown){
        rendered = true;
    }

    write_to_console(text_edit_width);
}

void DeviceTab::renderShellText(std::string str){
    //add text to the terminal text box

    if(!rendered){
        //don't add text if the tab is yet to be rendered!
        return;
    }

    //do colorization
    if(str.find("[INFO]") != std::string::npos){
        shellOutput->setTextColor("#FFFFFF");
    } else if(str.find("[WARNING]") != std::string::npos){
        shellOutput->setTextColor("#FFD09A");
    } else if(str.find("[ERROR]") != std::string::npos){
        shellOutput->setTextColor("#D2042D");
    } else if(str.find("[FATAL]") != std::string::npos){
        shellOutput->setTextColor("#D2042D");
    } else {
        shellOutput->setTextColor("#00FF00");
    }

    shellOutput->append(str.c_str());

    //enforce max number of lines
    current_shell_lines += countNewlines(str) + 1;
    write_to_console(current_shell_lines);
    while(current_shell_lines > MAX_SHELL_LINES){
        //remvoe up to the latest newline
        shellCursor.setPosition(0);
        shellCursor.movePosition(QTextCursor::Down, QTextCursor::KeepAnchor, 1);
        shellCursor.removeSelectedText();
        shellCursor.setPosition(shellOutput->toPlainText().size());

        current_shell_lines--;
    }
}

int DeviceTab::countNewlines(const std::string& str) {
    int count = 0;
    int char_width = 0;

    QFontMetrics shellFontMetrics(shellOutput->currentFont());

    for (char ch : str) {
        if (ch == '\n') {
            //reset the character count between new lines
            char_width = 0;
            ++count;
        } 

        if(char_width > text_edit_width){
            ++count;
            char_width = 0;
        }

        char_width += shellFontMetrics.boundingRect(ch).width();
    }

    return count;
}

LaunchStrip* DeviceTab::loadLaunchStrip(){
    //loads in the launch strip ui
    launch_strips.push_back(new LaunchStrip(LAUNCH_STRIP_SUBPATH, this));

    //add to the scroll view
    verticalLayoutScrollArea->addWidget(launch_strips.at(launch_strips.size() - 1)->child_widget);

    return launch_strips.at(launch_strips.size() - 1);
}

void DeviceTab::removeLaunchStrip(int index){

    //remove from the scroll area
    delete launch_strips.at(index)->child_widget;

    //remove from the array
    launch_strips.erase(launch_strips.begin() + index);
}

void DeviceTab::setComboBoxItems(QComboBox* comboBox, std::vector<std::string>& items) {
    if (!comboBox) return;  // Safety check

    //add in the default item
    items.push_back(std::string(""));

    comboBox->clear();  // Remove existing items

    for (const auto& item : items) {
        comboBox->addItem(QString::fromStdString(item));
    }
}

DeviceTab::~DeviceTab()
{

}