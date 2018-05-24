#include <QDebug>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include "undist.hpp"
#include "ui_undist.h"
#include "dvision/frame.hpp"
#include "dvision/projection.hpp"
#include "dvision/parameters.hpp"

using namespace dvision;
Undist::Undist(QWidget *parent) : QDialog(parent), ui(new Ui::Undist) {
    ui->setupUi(this);

    ros::NodeHandle nh("~");
    parameters.init(&nh);
    m_projection.init(&nh);
}

Undist::~Undist() {
}

void Undist::setModel(MyModel *model)
{
   m_listmodel = model;
}

void Undist::onOriginalClicked(QMouseEvent *ev)
{
   m_x = ev->x();
   m_y = ev->y();
   updateView();
}

void Undist::updateView()
{
    // get undist image
    auto frame = m_listmodel->getFrame();
    cv::Mat mat;
    m_projection.dist()->undistortImage2(frame.getRGB(), mat);
    m_undistImg = QPixmap::fromImage(QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888));

    this->resize(m_undistImg.width(), m_undistImg.height());
    ui->img->resize(m_undistImg.width(), m_undistImg.height());

    // undist point
    auto point = m_projection.dist()->undistort(m_x, m_y);
    m_x = point.x;
    m_y = point.y;

//    qDebug() << "Undist: (" << m_x << ", " << m_y << ")";


    // paint point
    QPainter painter(&m_undistImg);
    QPen pen(Qt::red);
    pen.setWidth(10);

    QPoint p1;
    p1.setX(m_x);
    p1.setY(m_y);

    painter.setPen(pen);
    painter.drawPoint(p1);

    ui->img->setPixmap(m_undistImg);
    setVisible(true);
}
