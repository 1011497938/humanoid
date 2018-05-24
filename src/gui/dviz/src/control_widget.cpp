#include "control_widget.hpp"
#include <QLabel>
#include "model.hpp"
#include <QDebug>

namespace dviz {

ControlWidget::ControlWidget(QWidget *parent)
    : QWidget(parent)
{

    auto layout = new QVBoxLayout();
    auto vlayout = new QVBoxLayout();

    // set up mode selection
    {
        mode = new QComboBox();
        mode->addItem("Monitor");
        mode->addItem("Simulator");

        auto topLayout = new QHBoxLayout();
        QLabel* txt = new QLabel("Mode:");
        topLayout->addWidget(txt);
        topLayout->addStretch();
        topLayout->addWidget(mode);
        vlayout->addLayout(topLayout);

        connect(mode, &QComboBox::currentTextChanged, [](QString s){
            MODE mode = s == "Monitor" ? MONITOR : SIMULATOR;
            Model::setMode(mode);
        });
    }

    {
        showViewRange = new QCheckBox();
        auto hlayout = new QHBoxLayout();
        QLabel* txt = new QLabel("viewRange");
        hlayout->addWidget(txt);
        hlayout->addStretch();
        hlayout->addWidget(showViewRange);
        vlayout->addLayout(hlayout);

        connect(showViewRange, &QCheckBox::toggled, [](bool checked){
            Model::showViewRange = checked;
        });
    }

    {
        showParticle = new QCheckBox();
        auto hlayout = new QHBoxLayout();
        QLabel* txt = new QLabel("Particles");
        hlayout->addWidget(txt);
        hlayout->addStretch();
        hlayout->addWidget(showParticle);
        vlayout->addLayout(hlayout);

        connect(showParticle, &QCheckBox::toggled, [](bool checked){
            Model::showParticles = checked;
        });
    }

    showRobot.resize(NUM_ROBOT+ 1);
    for(int i = 1; i <= NUM_ROBOT; ++i) {
        auto enableRobot = new QCheckBox();
        enableRobot->setDisabled(true);
        showRobot[i] = enableRobot;
        auto hlayout = new QHBoxLayout();
        auto txt = new QLabel(QString("Robot %1").arg(i));
        hlayout->addWidget(txt);
        hlayout->addStretch();
        hlayout->addWidget(enableRobot);
        vlayout->addLayout(hlayout);

        connect(enableRobot, &QCheckBox::toggled, Model::getInstance(i),
                &Model::setEnable);

        connect(mode, &QComboBox::currentTextChanged, [=](QString s){
            enableRobot->setChecked(false);
        });


        connect(Model::getInstance(i), &Model::robotConnectionStatusChanged, [=](int id, bool connected){
            if(!connected)
                enableRobot->setChecked(false);
            enableRobot->setEnabled(connected);
        });
    }

    layout->addLayout(vlayout);
    this->setLayout(layout);

}



}
