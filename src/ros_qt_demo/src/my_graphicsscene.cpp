#include "../include/ros_qt_demo/my_graphicsscene.h"
namespace class1_ros_qt_demo {
my_graphicsScene::my_graphicsScene(){}

my_graphicsScene::~my_graphicsScene(){}

void my_graphicsScene::drawBackground(QPainter *painter, const QRectF &rect)
{
    if(views().count()==0)return;

    // 计算视窗的大小,消除图元拖动时出现的残影
    QGraphicsView* pView=views().first();
    QRect contentRect=pView->viewport()->contentsRect();
    QRectF sceneRect =pView->mapToScene(contentRect).boundingRect();

    //绘制指定图片作为背景
    QPixmap pixmap(":/images/234.jpg");
    painter->drawPixmap(sceneRect,pixmap,QRect());
}
}  // namespace class1_ros_qt_demo
