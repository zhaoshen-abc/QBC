/**
 * @file /include/class1_ros_qt_demo/main_window.hpp
 *
 * @brief Qt based gui for class1_ros_qt_demo.
 *
 * @date November 2010
 **/
#ifndef class1_ros_qt_demo_MAIN_WINDOW_H
#define class1_ros_qt_demo_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include "ui_qbc.h"
#include "connectdialog.h"
#include "qnode.hpp"
#include "service_pose.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace class1_ros_qt_demo {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

  void ReadSettings(); // Load up qt program settings at startup
  void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

signals:
  void ROSstart();

public Q_SLOTS:
    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
    void on_button_connect_clicked();
    void on_checkbox_use_environment_stateChanged(int state);

      /******************************************
      ** Manual connections
      *******************************************/
    void DisplayData();
    void updatemap();
    void updateLoggingView(); // no idea why this can't connect automatically

private slots:
    void on_Button_Quit_clicked();

    void on_Button_ChooseType_clicked();



    void on_Button_page2_type1_clicked();

    void on_Button_page3_switch_clicked();

    void on_Button_page4_quit_clicked();

    void on_Button_page5_quit_clicked();

    void on_Button_page3_quit_clicked();

    void on_Button_page4_gather_clicked();

    void on_Button_page4_disperse_clicked();

    void on_Button_page5_switch_clicked();

    void on_Button_page2_connect_clicked();

private:
  Ui::MainWindow QBCui;
  ConnectDialog* dialog;

	QNode qnode;
  Service_Pose service_pose;
  Service_Pose service_pose_2;

  QStringListModel my_logging_model;
  QGraphicsPixmapItem *pixmapItem;
};

}  // namespace class1_ros_qt_demo

#endif // class1_ros_qt_demo_MAIN_WINDOW_H
