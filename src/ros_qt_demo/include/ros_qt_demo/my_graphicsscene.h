#ifndef GRAPHICSSCENE_H
#define GRAPHICSSCENE_H
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QPainterPath>
#include <QPixmap>
#include <QDebug>
namespace class1_ros_qt_demo {
class my_graphicsScene : public QGraphicsScene
{
    Q_OBJECT
public:
    my_graphicsScene();
    ~my_graphicsScene();

protected:
 void drawBackground(QPainter *painter, const QRectF &rect);
};

}  // namespace class1_ros_qt_demo

#endif // GRAPHICSSCENE_H
