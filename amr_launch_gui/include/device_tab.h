#pragma once

#include <QtWidgets>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtUiTools/QUiLoader>
#include <QTextCursor>
#include <QFontMetrics>
#include <QFile>
#include <string>


#include <ament_index_cpp/get_package_share_directory.hpp>

#include "launch_strip.h"
#include "debug_tools.h"
#include "update_data.h"


#define LAUNCH_STRIP_SUBPATH "/ui/LaunchStrip.ui"
#define MAX_SHELL_LINES 500

class DeviceTab : public QObject{

public:
    DeviceTab(std::string ui_file_path, QObject *parent);
    ~DeviceTab();

    QWidget *child_widget;

    void renderShellText(std::string str);

    void reRender(bool isShown);

    virtual void handle_data(GuiUpdateData data);

public slots:
    void onRebootPress();


private:
    void findChildren();

    int countNewlines(const std::string& str);

    // --- CHILDREN
    QTextEdit *shellOutput;
    QScrollArea *launchScrollArea;
    QWidget *launchScrollAreaWidget;
    QVBoxLayout *verticalLayoutScrollArea;
    QTextCursor shellCursor;
    QPushButton *rebootDeviceButton;
    //-------------

    bool rendered = false;
    int text_edit_width = 0;

    int current_shell_lines = 0;

    std::vector<LaunchStrip> launch_strips;

protected:
    void setComboBoxItems(QComboBox* comboBox, const std::vector<std::string>& items);

    void loadLaunchStrip();
    void removeLaunchStrip(int index);

};

