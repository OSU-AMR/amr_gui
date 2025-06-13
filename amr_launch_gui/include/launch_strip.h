#pragma once

#include <QtWidgets>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtUiTools/QUiLoader>
#include <QFile>

#include <ament_index_cpp/get_package_share_directory.hpp>

class LaunchStrip{
public:
    LaunchStrip(std::string ui_file_path);
    ~LaunchStrip();

    QWidget *child_widget;


public slots:

private:

    void findChildren();


    //children----------------
    QProgressBar *launch_progress_bar;
    QComboBox *launch_select;
    //-----------------------

};