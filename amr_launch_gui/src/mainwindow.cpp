#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{

    //create the serverwrapper
    sw = new ServerWrapper();

    renderMainContent();
    // m_button = new QPushButton("Store Content", this);

    // setCentralWidget(m_button);
    // connect(m_button, &QPushButton::clicked, this, &MainWindow::storeContent);

    //regiser the metatype
    qRegisterMetaType<GuiUpdateData>();
}

MainWindow::~MainWindow()
{

}

void MainWindow::renderMainContent(){
    //load in ui file
    QUiLoader loader;

    //get qt file path
    std::string share_dir_path = ament_index_cpp::get_package_share_directory("amr_launch_gui");
    
    std::string main_widget_filepath = std::string(share_dir_path + MAIN_UI_FILE_SUBPATH);
    QFile file(main_widget_filepath.c_str());
    if(!file.open(QFile::ReadOnly)){
        qFatal("Cannot open main layout ui! %s", main_widget_filepath.c_str());
    }
    main_widget = loader.load(&file);
    layout = main_widget->layout();
    file.close();

    //render in the central tab
    renderCentralTab();

    main_widget->sizePolicy().setVerticalPolicy(QSizePolicy::Expanding);
    main_widget->sizePolicy().setHorizontalPolicy(QSizePolicy::Expanding);

    setCentralWidget(main_widget);

    write_to_console("Completed file rendering");


    //connnect things to the server wrapper
    connect(sw, &ServerWrapper::amr_data_available, this, &MainWindow::handleAMRData, Qt::QueuedConnection);
    connect(sw, &ServerWrapper::central_data_available, this, &MainWindow::handleCentralData, Qt::QueuedConnection);
    sw->start();

    write_to_console("Started launch server!");
    
}

void MainWindow::renderCentralTab(){
    //get the tab widget
    QTabWidget *tabs_widget  = main_widget->findChild<QTabWidget *>("device_tabs");

    //create the tab object
    //CentralTab tab(this);
    CentralTab *tab = new CentralTab(this);

    //add to tab list
    tabs.push_back(tab);
    tabs_widget->addTab(tabs.at(tabs.size() - 1)->child_widget, "Central");

    //remove the first tab as its a dummy
    tabs_widget->removeTab(0);

    //upda the index
    central_tab_index = tabs.size() - 1;

}

void MainWindow::resizeEvent(QResizeEvent* event)
{
   QMainWindow::resizeEvent(event);
   // Your code here.
    int shown_index = main_widget->findChild<QTabWidget *>("device_tabs")->currentIndex() - 1; //may need to remvoe the one if the default tab is removed
    for(int i = 0; i < tabs.size(); i++){

        tabs.at(i)->reRender(shown_index == i);
    }

}

//direct central data
void MainWindow::handleCentralData(GuiUpdateData data){

    write_to_console("I have new central data");

    //send this to the central tab

    tabs.at(central_tab_index)->handle_data(data);
}

//direct amr data
void MainWindow::handleAMRData(GuiUpdateData data){

    write_to_console("I have new amr data");


}

void MainWindow::write_to_console(std::string str)
{
    //write to the console
    QString message(str.c_str());
    qDebug() << message;
}

#include "moc_mainwindow.cpp"