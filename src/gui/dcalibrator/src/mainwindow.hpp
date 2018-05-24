#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include <QApplication>
#include <QKeyEvent>
#include "mymodel.hpp"
#include "clabel.hpp"
#include "dvision/parameters.hpp"
#include "dvision/distortionModel.hpp"
#include "undist.hpp"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);

    inline void setApp(QApplication* app) { m_app = app; }
private:
    void init();
    void connectSignals();

public slots:
    void onOriginalClicked(QMouseEvent*);

    // update
    void updateView();

private slots:
    void on_actionOpen_triggered();
    void on_actionSave_triggered();

    void on_currentChanged(QModelIndex current);
    void keyReleaseEvent(QKeyEvent* ev) override;

    void appendText();

private:
    Ui::MainWindow* m_ui;

    MyModel* m_listmodel;
    CLabel* m_orignial;
    Undist* m_undist;

    QVector<QPoint> m_realPoints;
    QApplication* m_app;

    double m_pitch = -9999;
    double m_yaw = -9999;
};

#endif // MAINWINDOW_H
