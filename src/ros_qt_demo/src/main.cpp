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
#include <QSplashScreen>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QTimer>
#include <QQmlContext>
#include "../include/ros_qt_demo/main_window.hpp"


/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);

    QQuickView viwer;
    //无边框，背景透明
    viwer.rootContext()->setContextProperty("wndCtrl",&viwer);
    viwer.setFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
    viwer.setColor(QColor(Qt::transparent));
    //加载qml
    viwer.setSource(QUrl("qrc:/qmls/qmls/showgif.qml"));
    viwer.show();
    //将viewer设置为main.qml属性

//    HomePage hm();
//    hm.show();
//    FlightWindow fw();
//    fw.show();

    class1_ros_qt_demo::MainWindow mw(argc,argv);
    mw.showFullScreen();
//    mw.show();

    int result = app.exec();

	return result;
}
