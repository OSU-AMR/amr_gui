#pragma once

#include <QtWidgets>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtUiTools/QUiLoader>
#include <QFile>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "debug_tools.h"
#include "central_data.h"
#include "communication_signals.h"


class LaunchStrip : public QObject{
    Q_OBJECT

public:
    LaunchStrip(std::string ui_file_path, QObject *parent);
    ~LaunchStrip();

    QWidget *child_widget;

    AvailableLaunches getSelectedLaunch();
    void setLaunchArgValue(std::string str, int arg_index);
    void setLaunchOptions(std::vector<AvailableLaunches> options);

    QComboBox *getLaunchSelectPtr();

    QPushButton *getLaunchButtonPtr();
    QPushButton *getHaltButtonPtr();

signals:

    void haltLaunch(QString file_name);
    void beginLaunch(QString file_name);

public slots:

    void handleLaunchPress();
    void handleHaltPress();
    void handleApplyPress();
    void handleLaunchSelect();
    void handleLaunchArgSelect();

private:

    void findChildren();

    //children----------------
    QProgressBar *launch_progress_bar;
    QComboBox *launch_select;
    QLabel *uptime_label;
    QPushButton *launch_button;
    QPushButton *halt_button;
    QPushButton *apply_button;
    QComboBox *arg_combo;
    QLineEdit *arg_value_box;
    //-----------------------

    void setComboBoxItems(QComboBox* comboBox, std::vector<std::string> items);

    std::vector<AvailableLaunches> launches; 

    std::string formulate_launch_request_string(AvailableLaunches launch);   

};