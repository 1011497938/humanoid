#include "mainwindow.hpp"
#include "ui_mainwindow.h"
#include "clabel.hpp"
#include <QtCore>
#include <QFileDialog>
#include <QListView>

using namespace dvision;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), m_ui(new Ui::MainWindow), m_listmodel(new MyModel(this)), m_undist(new Undist(this))
{
    m_ui->setupUi(this);
    init();
    m_orignial->setModel(m_listmodel);
    m_undist->setModel(m_listmodel);
    m_ui->listView->setModel(m_listmodel);
    m_ui->listView->setViewMode(QListView::IconMode);
    m_undist->setVisible(false);

}

void MainWindow::init()
{
    m_orignial = m_ui->imagePanel;

    m_realPoints << QPoint(10, 10);
    m_realPoints << QPoint(10, -10);

    m_realPoints << QPoint(20, 10);
    m_realPoints << QPoint(20, -10);

    m_realPoints << QPoint(75, 0); // circle front
    m_realPoints << QPoint(0, -75);// circle right
    m_realPoints << QPoint(0, 75); // circle left

    m_realPoints << QPoint(240, 0); // penalty point


    m_realPoints << QPoint(0, 300); // T left
    m_realPoints << QPoint(0, -300); // T right

    m_realPoints << QPoint(450, 130); // left goal
    m_realPoints << QPoint(450, -130); // right goal

    m_realPoints << QPoint(450, 300); // left corner
    m_realPoints << QPoint(450, -300); // right corner


    auto* box = m_ui->candidateReal;
    foreach(const QPoint& p, m_realPoints){
       box->addItem(QString("%1, %2").arg(p.x()).arg(p.y()));
    }


}

void MainWindow::connectSignals()
{
    connect(m_ui->listView->selectionModel(), &QItemSelectionModel::currentChanged,
            m_listmodel, &MyModel::onCurrentIndexChanged);

    connect(m_ui->listView->selectionModel(), &QItemSelectionModel::currentChanged,
            m_orignial, &CLabel::updateView);

    connect(m_ui->listView->selectionModel(), &QItemSelectionModel::currentChanged,
            m_undist, &Undist::updateView);

    connect(m_ui->listView->selectionModel(), &QItemSelectionModel::currentChanged,
            this, &MainWindow::on_currentChanged);

    connect(m_ui->imagePanel, &CLabel::clicked,
            this, &MainWindow::onOriginalClicked);

    connect(m_ui->imagePanel, &CLabel::clicked,
            m_undist, &Undist::onOriginalClicked);

}

void MainWindow::onOriginalClicked(QMouseEvent *ev)
{
    int u = ev->x();
    int v = ev->y();

    QString imgpos = QString("%1, %2").arg(u).arg(v);
    m_ui->labelImagePosition->setText(imgpos);

    cv::Point2f real;
    m_undist->projection()->getOnRealCoordinate(cv::Point(u, v), real);

    QString realpos = QString("%1 %2").arg(real.x).arg(real.y);
    m_ui->labelReal->setText(realpos);
}

void MainWindow::updateView()
{
   // ?! update label text
}


void MainWindow::on_currentChanged(QModelIndex current)
{
    // update H and V
    auto pitchyaw = m_listmodel->getPlatAngle(current.row());

    m_pitch = pitchyaw.x();
    m_yaw = pitchyaw.y();

    m_undist->projection()->updateExtrinsic(m_pitch, m_yaw);
//    m_undist->projection()->updateExtrinsic(60, 0);

    m_ui->pitch->setText(QString::number(m_pitch));
    m_ui->yaw->setText(QString::number(m_yaw));

    updateView();
}

void MainWindow::keyReleaseEvent(QKeyEvent *ev)
{
    if(ev->isAutoRepeat())
        return;

    if(ev->key() == Qt::Key_Space) {
        appendText();
    } else if (ev->key() == Qt::Key_Escape) {
//        qDebug() << "Exit";
//        m_app->closeAllWindows();
    }
}

void MainWindow::appendText()
{
    auto* textEdit = m_ui->textEdit;
    auto real = m_realPoints.at(m_ui->candidateReal->currentIndex());
    // !? todo, calc undistorted img position
//    int x = m_orignial->x();
//    int y = m_orignial->y();
//    auto p = undistPoint(x, y);
//    x = p.x();
//    y = p.y();

    textEdit->append(QString("%1 %2 %3 %4 %5 %6")
                     .arg(m_yaw)
                     .arg(m_pitch)
                     .arg(m_undist->x())
                     .arg(m_undist->y())
                     .arg(real.x())
                     .arg(real.y())

//            .arg(x)
//            .arg(y)
//            .arg(real.x())
//            .arg(real.y())
//            .arg(m_pitch)
//            .arg(m_yaw)
                     );
}


void MainWindow::on_actionSave_triggered()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                               QDir::homePath(),
                               tr("Plain text (*.txt)"));
    if(fileName == NULL)
        return;

    QFile file(fileName);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return;
    }

    QTextStream in(&file);
    in << m_ui->textEdit->toPlainText();
    file.close();
}

void MainWindow::on_actionOpen_triggered()
{
    connectSignals();
    QString filename = QFileDialog::getOpenFileName(this, tr("Open Image"), "/home/", tr("Image files (*.png *.jpg)"));
    if(filename != NULL) {
        m_listmodel->loadImages(filename);
        m_ui->listView->setCurrentIndex(m_listmodel->index(0, 0));
    }
}
