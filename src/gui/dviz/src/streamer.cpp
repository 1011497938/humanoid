#include "streamer.hpp"
#include "common.hpp"
#include <QDebug>
#include <QFile>
#include <QTimer>
#include <ctime>

namespace dviz {

Streamer::Streamer(QWidget* parent)
  : QDockWidget("Camera")
{
    QWidget* fuck = new QWidget();
    m_label = new QLabel();
    m_save_button = new QPushButton("save", this);

    m_save_button->setShortcut(QKeySequence("s"));

    dvision::Frame::initEncoder();

    auto layout = new QVBoxLayout();
    auto select = new QComboBox();
    connect(select, &QComboBox::currentTextChanged, [this](QString s) {
        m_current_id = s.toInt();
        qDebug() << "current id:" << m_current_id;
    });
    connect(m_save_button, &QPushButton::clicked, [this]() {
        // set path of image
        std::time_t t = std::time(0);
        std::string pixmap_path = std::to_string(t) + ".png";
        // save pixmap to file
        QFile pixmap_file(pixmap_path.c_str());
        pixmap_file.open(QIODevice::WriteOnly);
        // auto pixmap = m_label->pixmap();
        if (!pixmap_.isNull()) {
            pixmap_.save(&pixmap_file, "PNG");
            std::cout << "save to " << pixmap_path << std::endl;
        } else {
            std::cerr << "saving error: no pixmap in m_label" << std::endl;
        }
    });

    for (int i = 1; i <= NUM_ROBOT; ++i) {
        select->addItem(QString("%1").arg(i));

        auto d = new dtransmit::DTransmit("255.255.255.255");
        d->addRawRecv(dconstant::network::robotGuiBase + i, [this, i](void* buffer, std::size_t size) {
            if (m_current_id != i)
                return;
            dvision::Frame f;
            try {
                f.decode(buffer);
                auto img = f.getRGB();
                std::lock_guard<std::mutex> lk(lock);
                pixmap_ = QPixmap::fromImage(QImage(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888));
            } catch (std::exception& e) {
                std::cerr << "decode buffer error: " << e.what() << std::endl;
            }
        });
        d->startService();
        m_transmitters.push_back(d);
    }

    layout->addWidget(select);
    layout->addWidget(m_label);
    layout->addWidget(m_save_button);

    fuck->setLayout(layout);
    this->setWidget(fuck);

    auto t = new QTimer(this);
    connect(t, &QTimer::timeout, [&]() {
        std::lock_guard<std::mutex> lk(lock);
        if (pixmap_.width() > 0 && pixmap_.height() > 0)
            m_label->setPixmap(pixmap_.scaled(320, 240));
        update();
    });
    t->start(1000.0 / 30.f);
}

Streamer::~Streamer()
{
    for (auto& d : m_transmitters) {
        delete d;
    }
}
}
