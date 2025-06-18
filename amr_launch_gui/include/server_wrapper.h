#pragma once

#include <QThread>
#include <QtWidgets>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtUiTools/QUiLoader>
#include <QFile>

#include <string>
#include <vector>

#include "update_data.h"
#include "launch_server.h"
#include "debug_tools.h"
#include "communication_signals.h"

class ServerWrapper : public QThread {
    //class defining the thread used to drive the launch server

    Q_OBJECT

public:
    ServerWrapper();
    ~ServerWrapper();

    void run() override;

public slots:
    void handleCentralHostnameChange(QString hostname);
    void handleCentralLaunchBegin(QString filename);
    void handleCentralLaunchHalt(QString filename);

signals:
    void amr_data_available(GuiUpdateData data);
    void central_data_available(GuiUpdateData data);

private:
    Launch_Server server;

    std::vector<std::string> central_hosts;
    std::vector<std::string> amr_hosts;

    int direct_data(GuiUpdateData data);

    std::string central_hostname;
};