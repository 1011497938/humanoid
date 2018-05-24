#ifndef MYMODEL_H
#define MYMODEL_H
#include <QAbstractListModel>
#include <QtCore>
#include <QPixmap>
#include <QIcon>
#include "dvision/frame.hpp"

class MyModel : public QAbstractListModel
{
    Q_OBJECT
public:
    MyModel(QObject* parent);

    int rowCount(const QModelIndex &parent) const;

    // implement this for insert items in the view
    bool insertRows(int row, int count, const QModelIndex &parent);

    QVariant data(const QModelIndex &index, int role) const;

    void loadImages(QString filename);

    const QPixmap getImage(int row) const;

    const QPixmap getImage() const;

    dvision::Frame getFrame() const;

    void setCurrentIndex(int);

    QPointF getPlatAngle(int index);

public slots:
    void onCurrentIndexChanged(QModelIndex current);

private:
    QVector<QPixmap> m_pictures;
    QVector<QIcon> m_pics;
    QVector<QString> m_picname;
    QFileInfoList m_allfilesInfo;
    int m_currentIndex;
};

#endif // MYMODEL_H
