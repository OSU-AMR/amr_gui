#pragma once

#include <string>
#include <QtWidgets>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtUiTools/QUiLoader>
#include <QFile>
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <iostream>


void write_to_console(std::string str);
void write_to_console(int str);