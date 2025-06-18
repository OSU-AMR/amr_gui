#pragma once

#include <QtWidgets>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtUiTools/QUiLoader>
#include <QFile>
#include <QApplication>
#include <QComboBox>
#include <QWidget>
#include <QVBoxLayout>

#include "device_tab.h"
#include "update_data.h"
#include "central_data.h"
#include "debug_tools.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

class CentralTab : public DeviceTab{
    Q_OBJECT

public:
    CentralTab(QObject *parent);
    ~CentralTab();

    void handle_data(GuiUpdateData data) override;

signals:

    void emitCentralHostname(QString hostname);
    void emitCentralLaunchBegin(QString filename);
    void emitCentralLaunchHalt(QString filename);

public slots:

    //handle when the central combo is changed
    void handleCentralComboChange();

    void handleLaunchBeginPressed(QString filename);
    void handleLaunchHaltPressed(QString filename);

private:
    //data structure
    std::map<std::string, CentralData> central_data;

    void update_central_combo();

    void find_central_children();

    void update_launch_bars();

    void load_and_connect_launch_strip();

    CentralData* get_current_central_data();

    //Children ------------------
    QComboBox *selectCombo;

    // ---------------------
    


};

