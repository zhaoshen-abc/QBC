/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include <QQuickView>
#include <QQmlContext>
#include "../include/ros_qt_demo/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt :/qmls/qmls/showgif.qml
    **********************/
    QApplication app(argc, argv);

    QQuickView viwer;
    //无边框，背景透明
    viwer.rootContext()->setContextProperty("wndCtrl",&viwer);
    viwer.setFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
    viwer.setColor(QColor(Qt::transparent));
    //加载qml
    viwer.setSource(QUrl("qrc:/qmls/qmls/showgif.qml")); //:/qmls/qmls/showgif.qml
    viwer.show();

    class1_ros_qt_demo::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
