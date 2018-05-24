#pragma once
#include <QDialog>
#include <QPixmap>
#include <QDockWidget>
#include "dvision/frame.hpp"
#include "dvision/projection.hpp"
#include "mymodel.hpp"

namespace Ui {
   class Undist;
}

class Undist : public QDialog {
   Q_OBJECT

public:
    Undist(QWidget* parent);
    ~Undist();
    void setModel(MyModel*);

    inline int x() const { return m_x; }
    inline int y() const { return m_y; }

    inline dvision::Projection* projection() {
       return &m_projection;
    }

 public slots:
    void onOriginalClicked(QMouseEvent* ev);
    void updateView();

private:
    Ui::Undist* ui;
    int m_x = 0;
    int m_y = 0;

    dvision::Projection m_projection;
    QPixmap m_undistImg;
    MyModel* m_listmodel;
};

