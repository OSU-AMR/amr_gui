#pragma once

#include <QtWidgets>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtUiTools/QUiLoader>
#include <QFile>

#include "device_tab.h"
#include "update_data.h"
#include "central_data.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

class CentralTab : public DeviceTab{
public:
    CentralTab(QObject *parent);
    ~CentralTab();

    void handle_data(GuiUpdateData data) override;

public slots:

private:
    //data structure
    std::map<std::string, CentralData> central_data;

    void update_central_combo();

    void find_central_children();

    void update_launch_bars();

    CentralData* get_current_central_data();

    //Children ------------------
    QComboBox *selectCombo;

    // ---------------------
    


};

