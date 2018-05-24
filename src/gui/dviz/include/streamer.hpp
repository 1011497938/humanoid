#pragma once
#include <QComboBox>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <mutex>
#include <vector>

#include "dconfig/dconstant.hpp"
#include "dtransmit/dtransmit.hpp"
#include "dvision/frame.hpp"

namespace dviz {

class Streamer : public QDockWidget
{
    Q_OBJECT
  public:
    Streamer(QWidget* parent = 0);
    ~Streamer();

  private:
    QLabel* m_label;
    std::mutex lock;
    QPushButton* m_save_button;
    QPixmap pixmap_;
    int m_current_id = 0;
    int cnt_ = 0;
    std::vector<dtransmit::DTransmit*> m_transmitters;
};
}
