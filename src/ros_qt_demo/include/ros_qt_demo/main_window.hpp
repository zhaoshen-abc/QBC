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
#include "ui_main_window.h"
#include "ui_test.h"
#include "qnode.hpp"
#include "service_pose.hpp"
#include "my_graphicsscene.h"

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
    void test();

public Q_SLOTS:

	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void TEST();
    void updatemap();
private slots:

    void on_button_connect_clicked();

    void on_button_Mission_clicked(bool checked);

    void on_button_Formation_clicked(bool checked);

    void on_button_Mode_clicked(bool checked);

    void on_button_Pose_clicked(bool checked);

    void on_Button_flight_Takeoff_clicked(bool checked);

    void on_Button_flight_Landing_clicked(bool checked);

    void on_Button_flight_Formation_clicked(bool checked);

    void on_Button_flight_Mission_clicked(bool checked);

    void on_Button_flight_Switch_clicked(bool checked);

    void on_Button_flight_trajmode_clicked(bool checked);

    void on_Button_flight_hit_clicked(bool checked);

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_Button_Quit_clicked();

    void on_Button_ChooseType_clicked();

private:
	Ui::MainWindowDesign ui;
    Ui::MainWindow QBCui;
    QNode qnode;
    QStringListModel my_logging_model;
    my_graphicsScene* scene;
    QGraphicsPixmapItem *pixmapItem;

    Service_Pose service_pose;
    Service_Pose service_pose_2;
    QImage image_roll;
};

}  // namespace class1_ros_qt_demo

#endif // class1_ros_qt_demo_MAIN_WINDOW_H
