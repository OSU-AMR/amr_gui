#include <QCoreApplication>
#include <QString>
#include <QFile>
#include <QDir>
#include <QTextStream>
#include <QDebug>

#include <stdio.h>

#include "mainwindow.h"


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MainWindow win;
    
    //go fullscreen
    win.setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
    win.resize(1000, 1500);
    win.setVisible(true);

    return app.exec();
}