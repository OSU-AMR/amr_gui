#pragma once

#include <QtWidgets>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtUiTools/QUiLoader>
#include <QFile>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "debug_tools.h"

class LaunchStrip : public QObject{
    Q_OBJECT

public:
    LaunchStrip(std::string ui_file_path, QObject *parent);
    ~LaunchStrip();

    QWidget *child_widget;

    std::string getSelectedLaunch();
    void setLaunchOptions(std::vector<std::string> options);

    QComboBox *getLaunchSelectPtr();

    QPushButton *getLaunchButtonPtr();
    QPushButton *getHaltButtonPtr();

signals:

    void haltLaunch(QString file_name);
    void beginLaunch(QString file_name);

public slots:

    void handleLaunchPress();
    void handleHaltPress();

private:

    void findChildren();

    //children----------------
    QProgressBar *launch_progress_bar;
    QComboBox *launch_select;
    QLabel *uptime_label;
    QPushButton *launch_button;
    QPushButton *halt_button;
    //-----------------------

    void setComboBoxItems(QComboBox* comboBox, std::vector<std::string>& items);

    

};