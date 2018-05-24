#include "mainwindow.hpp"
#include "render.hpp"
#include "dconfig/dconstant.hpp"
#include "item_manager.hpp"
#include "streamer.hpp"
#include "control_widget.hpp"
#include <QGraphicsView>
#include <QDockWidget>
#include <QTimer>
#include <QDebug>

namespace dviz {
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
//    int w = dconstant::geometry::wholeWidth;
//    int h = dconstant::geometry::wholeHeight;
//    resize(w, h);

    QDockWidget* control_panel = new QDockWidget("Contorl");
    ControlWidget* c = new ControlWidget(control_panel);
    control_panel->setWidget(c);

    auto s = new Streamer();

    this->addDockWidget(Qt::LeftDockWidgetArea, control_panel);
    this->addDockWidget(Qt::LeftDockWidgetArea, s);


    auto render = new Render(this);
    auto itemManager = new ItemManager(this, render, c);
    itemManager->Init();

    auto view = new QGraphicsView(this);
    view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->setScene(render);

    setCentralWidget(view);

    auto timer = new QTimer(this);

    connect(timer, &QTimer::timeout, [=](){
        view->update();
        view->scene()->update();
        //s->update();
    });
    timer->start(1000 / 30.0);
}

MainWindow::~MainWindow() {
}
}
