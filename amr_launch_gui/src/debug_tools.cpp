#include "debug_tools.h"

void write_to_console(std::string str)
{
    //write to the console
    std::string comp_string = str;
    QString message(comp_string.c_str());
    qDebug() << message;
}

void write_to_console(int str)
{
    //write to the console
    std::string comp_string = std::to_string(str);
    QString message(comp_string.c_str());
    qDebug() << message;
}