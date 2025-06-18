#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtUiTools/QUiLoader>
#include <QFile>
#include <Qt>

#include <central_tab.h>

#include "device_tab.h"
#include "debug_tools.h"
#include "server_wrapper.h"
#include "update_data.h"

Q_DECLARE_METATYPE(GuiUpdateData)

#include <ament_index_cpp/get_package_share_directory.hpp>

#define MAIN_UI_FILE_SUBPATH "/ui/mainwindow.ui"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent=0);
    ~MainWindow();

public slots:
    void write_to_console(std::string str);

    void handleCentralLaunchBegin(QString filename);
    void handleCentralLaunchHalt(QString filename);

signals:
    void centralLaunchBegin(QString filename);
    void centralLaunchHalt(QString filename);

private:

    //render the main frame objects
    void renderMainContent();

    //render the central tab
    void renderCentralTab();

    //main widget the display is tied to
    QWidget *main_widget;

    QLayout *layout;

    CentralTab *central_tab;

    void resizeEvent(QResizeEvent* event);

    //all the device tabs
    std::vector<DeviceTab *> tabs;

    //launch server as QT thread
    ServerWrapper* sw;

    //direct amr data
    void handleCentralData(GuiUpdateData data);

    //direct central data
    void handleAMRData(GuiUpdateData data);

    int central_tab_index = 0;

    std::map<std::string, int> amr_tab_map;
};

#endif // MAINWINDOW_H