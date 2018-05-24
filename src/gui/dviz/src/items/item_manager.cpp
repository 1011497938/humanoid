#include "item_manager.hpp"
#include "ball.hpp"
#include "circle.hpp"
#include "control_widget.hpp"
#include "dest.hpp"
#include "field.hpp"
#include "goals.hpp"
#include "obstacles.hpp"
#include "particles.hpp"
#include "robot.hpp"
#include "viewrange.hpp"
#include "whitelines.hpp"
#include "whitepoints.hpp"
#include "world.hpp"
#include <algorithm>

namespace dviz {
ItemManager::ItemManager(QObject* parent, QGraphicsScene* scene, ControlWidget* control)
  : QObject(parent)
  , m_scene(scene)
  , m_control(control)
{
}

ItemManager::~ItemManager()
{
}

void
ItemManager::Init()
{
    m_scene->addItem(new FieldItem());
    // m_scene->addItem(new World());

    auto simBall = new Ball(false, 0);
    simBall->setVisible(false);
    m_scene->addItem(simBall);
    auto simObstacle = new Obstacles(false, 0);
    simObstacle->setVisible(false);
    m_scene->addItem(simObstacle);

    auto vecSimRobot = std::vector<Robot*>();

    for (int i = 1; i <= NUM_ROBOT; ++i) {

        /* @breif set up simulated robot,
         * should not appear in MONITOR mode
         */
        // Monitoring & Simulating
        auto simRobot = new Robot(i, false);
        vecSimRobot.push_back(simRobot);

        auto locRobot = new Robot(i, true);
        auto locBall = new Ball(true, i);
        auto p = new Particles(i);
        auto locCircle = new Circle(i);
        auto locLines = new WhiteLines(i);
        auto locWhitePoints = new WhitePoints(i);
        auto viewRange = new ViewRange(i);
        auto goals = new Goals(i);
        auto obstacles = new Obstacles(true, i);
        auto dest = new Dest(i);

        m_scene->addItem(locRobot);
        m_scene->addItem(simRobot);
        m_scene->addItem(locBall);
        m_scene->addItem(p);
        m_scene->addItem(locCircle);
        m_scene->addItem(viewRange);
        m_scene->addItem(locWhitePoints);
        m_scene->addItem(goals);
        m_scene->addItem(obstacles);
        m_scene->addItem(dest);

        connect(m_control->mode, &QComboBox::currentTextChanged, [=](QString s) {
            MODE mode = s == "Monitor" ? MONITOR : SIMULATOR;
            if (mode == MONITOR) {
                simRobot->setVisible(false);
                simBall->setVisible(false);
                simObstacle->setVisible(false);
            } else {
                simBall->setVisible(true);
                simObstacle->setVisible(true);
            }
        });

        connect(m_control->showRobot[i], &QCheckBox::toggled, [=](bool checked) {
            locRobot->setVisible(checked);
            locBall->setVisible(checked);
            locCircle->setVisible(checked);
            locLines->setVisible(checked);
            locWhitePoints->setVisible(checked);
            viewRange->setVisible(checked);
            goals->setVisible(checked);
            obstacles->setVisible(checked);
            dest->setVisible(checked);

            if (!checked) {
                p->setVisible(false);
            } else {
                if (m_control->showParticle->checkState() == Qt::Checked)
                    p->setVisible(true);
            }

            if (Model::getMode() == SIMULATOR) {
                simRobot->setVisible(checked);
            }
        });

        connect(m_control->showParticle, &QCheckBox::toggled, [=](bool checked) {
            if (m_control->showRobot[i]->checkState() == Qt::Checked) {
                p->setVisible(checked);
            }
        });
    }

    // 30 fps rendering
    auto t = new QTimer(this);
    connect(t, &QTimer::timeout, [=]() {

        //        qDebug() << "scene update" << cycle_++;
        //        qDebug() << m_scene->items().size();
        //        m_scene->update();
        //        auto items = m_scene->items();
        //        for(auto& it : items) {
        //            it->update();
        //        }
        // handle collide
        if (simBall->isVisible()) {
            auto ballx = simBall->x();
            auto bally = simBall->y();
            for (size_t i = 0; i < vecSimRobot.size(); ++i) {
                auto r = vecSimRobot[i];
                if (!r->isVisible())
                    continue;

                auto model = Model::getInstance(i + 1);
                auto simRobotPos = model->getSimRobotPos();
                auto ballField = getFieldPosition(simRobotPos, ballx, bally);

                if (fabs(ballField.x()) > 10 || fabs(ballField.y()) > 10) {
                    return;
                }

                ballField.setX(ballField.x() + ballField.x() > 0 ? 10 : -10);
                // ballField.setY(ballField.y() + ballField.y() > 0 ? 15 : -15);

                auto nb = getGlobalPosition(simRobotPos, QPointF(ballField.x(), ballField.y()));

                simBall->setX(nb.x());
                simBall->setY(nb.y());
            }
        }
        // if (simObstacle->isVisible()) {
        //     for (size_t i = 0; i < vecSimRobot.size(); ++i) {
        //         auto model = Model::getInstance(i + 1);
        //         auto r = vecSimRobot[i];
        //         if (!r->isVisible())
        //             continue;
        //
        //         for (auto& obstacle : model->getSimObstacles()) {
        //
        //             auto simRobotPos = model->getSimRobotPos();
        //             auto obstacleField = getFieldPosition(simRobotPos, obstacle.x, obstacle.y);
        //
        //             if (fabs(obstacleField.x()) > 10 || fabs(obstacleField.y()) > 10) {
        //                 return;
        //             }
        //
        //             simRobotPos.setX(simRobotPos.x() + obstacleField.x() > 0 ? 10 : -10);
        //             // simRobotPos.setY(simRobotPos.y() + obstacleField.y() > 0 ? 15 : -15);
        //
        //             model->setSimRobotPos(simRobotPos);
        //         }
        //     }
        // }
    });
    t->start(1000 / 30.f);
}
}
