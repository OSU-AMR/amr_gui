#pragma once

#include <QtWidgets>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtUiTools/QUiLoader>
#include <QFile>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "debug_tools.h"

class LaunchStrip{
public:
    LaunchStrip(std::string ui_file_path);
    ~LaunchStrip();

    QWidget *child_widget;

    std::string getSelectedLaunch();
    void setLaunchOptions(std::vector<std::string> options);

    QComboBox *getLaunchSelectPtr();

public slots:

private:

    void findChildren();


    //children----------------
    QProgressBar *launch_progress_bar;
    QComboBox *launch_select;
    QLabel *uptime_label;
    //-----------------------

    void setComboBoxItems(QComboBox* comboBox, std::vector<std::string>& items);

};