#pragma once
#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QRadioButton>
#include <QComboBox>
#include <QCheckBox>
#include <vector>
#include "render.hpp"

namespace dviz {

class ControlWidget : public QWidget {
    Q_OBJECT
public:
    ControlWidget(QWidget* parent = 0);

    std::vector<QCheckBox*> showRobot;
    QComboBox* mode;
    QCheckBox* showViewRange;
    QCheckBox* showParticle;
private:
};

}
