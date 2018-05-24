#include <QApplication>
#include "mainwindow.hpp"
#include "ros/ros.h"


int main(int argc, char** argv) {
    QApplication app(argc, argv);

    ros::Time::init();
    dviz::MainWindow win;
    win.show();


    return app.exec();
}
